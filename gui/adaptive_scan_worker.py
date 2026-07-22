"""
gui/adaptive_scan_worker.py

Runs AdaptiveRasterScanner.run() on a background thread and turns its
progress into messages on a queue.Queue for the GUI (Tk mainloop) thread
to drain and display — the adaptive-scan counterpart to
gui/scan_worker.py. Same threading contract (see memory:
carat_scanner_gui_scan_connection, written for scan_worker.py but equally
binding here):

  - run_adaptive_scan() is meant to be the `target` of a threading.Thread,
    never called directly from the GUI thread -- it blocks for the
    duration of the whole scan (potentially many rows).
  - This module never touches a tkinter object. It only touches the
    AdaptiveRasterScanner/hardware objects, the queue.Queue, and the
    threading.Event passed in -- all thread-safe by construction.
  - q is unbounded (queue.Queue() with no maxsize), so put() here never
    blocks the worker thread waiting on the GUI to catch up.
"""

import traceback

from adaptive_scan.adaptive_scan import AdaptiveRasterScanner


def run_adaptive_scan(config, params, motion, ir_reader, spectrometer, output_path, q, stop_event):
    """
    Entry point for the worker thread.

    motion, ir_reader, spectrometer: already-constructed instances --
    unlike scan_worker.run_scan(), this does NOT build its own motion
    controller or readers. AdaptiveScanPanel owns that connection (so the
    operator can jog/see a live signal readout before starting), and hands
    the SAME objects here rather than letting this function open a second
    connection -- see AdaptiveScanPanel for why reusing the same,
    already-positioned motion object matters (the scan starts from
    wherever the operator just jogged to, per spec §3 step 1).

    Puts one of the following onto q as the scan progresses:
      ("row", {"row_summary": RowSummary, "n_readings": int})
                            -- once per completed row, forwarded from
                               AdaptiveRasterScanner's on_row callback.
      ("done", AdaptiveScanResult)   -- scan finished (status=="completed").
      ("aborted", AdaptiveScanResult) -- stop_event was set; scan stopped
                               early. Still carries whatever rows/readings
                               were completed before the abort -- same
                               "partial results are real results" spirit
                               as ScanManager's per-point crash safety.
      ("error", message)    -- setup raised or the scan loop raised.
                               message is a pre-formatted string ready to
                               hand to a messagebox / log widget.

    Any exception is caught here -- an uncaught exception inside a
    threading.Thread target just prints a traceback to stderr and the
    thread quietly dies; without this try/except the GUI would sit there
    looking like the scan is still running with no indication anything
    went wrong.
    """
    try:
        scanner = AdaptiveRasterScanner(
            config, params, motion, ir_reader, spectrometer=spectrometer, output_path=output_path,
        )
    except Exception as exc:
        q.put(("error", _format_error("Failed to initialize adaptive scan", exc)))
        return

    try:
        result = scanner.run(
            on_row=lambda row_summary, readings: q.put(
                ("row", {"row_summary": row_summary, "n_readings": len(readings)})
            ),
            stop_event=stop_event,
        )
    except Exception as exc:
        q.put(("error", _format_error("Adaptive scan failed", exc)))
        return

    if result.status == "aborted":
        q.put(("aborted", result))
    else:
        q.put(("done", result))


def _format_error(context, exc):
    """
    Build a display-ready error string with the full traceback attached.
    Deliberately a standalone copy of scan_worker._format_error rather
    than an import from it -- same one-tiny-helper-not-worth-a-shared-
    module call as adaptive_scan_signal.extract_features (see that
    module's docstring) -- keeps this file independent of scan_worker.py.
    """
    tb = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
    return f"{context}: {exc}\n\n{tb}"
