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
      ("error", message)  -- setup raised, the scan loop raised, OR
                             ScanManager.run() stopped itself after an
                             axis fault (AxisStateUnknown) -- that last
                             case is reported as an "error", not "done",
                             since it means a hardware fault needs a
                             manual check before scanning again. message
                             is a pre-formatted string ready to hand to
                             a messagebox / log widget.

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
        status = manager.run(
            # ix/iy come from scan_manager as separate args (they're grid
            # indices, not part of the persisted record dict -- see
            # memory: carat_scanner_gui_scan_connection). Folded into a
            # copy of record here so downstream (status/live-map) only
            # has to deal with one dict per point, not a 3-tuple.
            on_point=lambda record, ix, iy: q.put(("point", {**record, "ix": ix, "iy": iy})),
            stop_event=stop_event,
        )
    except Exception as exc:
        q.put(("error", _format_error("Scan failed", exc)))
        return

    # status is "completed" / "aborted" / "axis_fault" (see
    # ScanManager.run's docstring). Trust the return value rather than
    # re-deriving it from stop_event.is_set() -- an axis fault stops the
    # scan WITHOUT stop_event ever being set, so that inference would
    # have silently mislabeled a hardware fault as a normal "done".
    if status == "aborted":
        q.put(("aborted", None))
    elif status == "axis_fault":
        q.put(("error",
               "Scan stopped: motion axis state unknown after a fault "
               "(AxisStateUnknown). The stage may still be physically "
               "moving. Manual hardware check required (mechanical "
               "binding, cabling, 8742 connection) before scanning again. "
               "See the log for the point/position where this happened."))
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
