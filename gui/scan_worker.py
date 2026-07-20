"""
gui/scan_worker.py

Runs ScanManager.run() on a background thread and turns its progress
into messages on a queue.Queue for the GUI (Tk mainloop) thread to
drain and display.

Threading contract (see memory: carat_scanner_gui_scan_connection):
  - run_scan() is meant to be the `target` of a threading.Thread, never
    called directly from the GUI thread -- it blocks for the duration
    of the whole scan.
  - This module never touches a tkinter object. It only touches the
    ScanManager/hardware objects, the queue.Queue, and the
    threading.Event passed in -- all thread-safe by construction.
  - q is unbounded (queue.Queue() with no maxsize), so put() here never
    blocks the worker thread waiting on the GUI to catch up.
"""

import traceback

from scan_manager import ScanManager


def run_scan(config, q, stop_event):
    """
    Entry point for the worker thread.

    Puts one of the following onto q as the scan progresses:
      ("point", record)  -- once per point, forwarded straight from
                             ScanManager's on_point callback
      ("done", None)      -- scan finished normally, ran every point
      ("aborted", None)   -- stop_event was set; scan stopped early
      ("error", message)  -- setup or the scan loop raised; message is
                             a pre-formatted string (context + traceback)
                             ready to hand to a messagebox / log widget

    Any exception -- hardware init failing, a read blowing past its
    retries and re-raising, anything -- is caught here. An uncaught
    exception inside a threading.Thread target just prints a traceback
    to stderr and the thread quietly dies; without this try/except the
    GUI would sit there looking like the scan is still running with no
    indication anything went wrong.
    """
    try:
        manager = ScanManager(config)
    except Exception as exc:
        q.put(("error", _format_error("Failed to initialize scan hardware", exc)))
        return

    try:
        manager.run(
            on_point=lambda record: q.put(("point", record)),
            stop_event=stop_event,
        )
    except Exception as exc:
        q.put(("error", _format_error("Scan failed", exc)))
        return

    # run() returns normally in both the "ran to completion" and the
    # "stop_event fired between points" cases (see scan_manager.run --
    # it returns early rather than raising on abort). stop_event is the
    # only way to tell which one happened after the fact.
    if stop_event.is_set():
        q.put(("aborted", None))
    else:
        q.put(("done", None))


def _format_error(context, exc):
    """
    Build a display-ready error string with the full traceback attached.
    Worth keeping the traceback even though the GUI will likely only
    show the first line in a messagebox -- hardware failures (USB
    disconnects, PAC timeouts) often only reproduce on the instrument
    PC, so the traceback belongs in the on-screen log, not just stderr.
    """
    tb = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
    return f"{context}: {exc}\n\n{tb}"
