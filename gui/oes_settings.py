"""
gui/oes_settings.py -- OES acquisition settings block shared by the Scan
and Line Scan tabs (Batch 1, 2026-10-14).

Shows integration time, number of averages and boxcar half-width,
initialized from config.yaml's oes: block. Like every other field on
those tabs, edits apply to THIS run only (folded into effective_config)
and are never written back to config.yaml; the values actually applied
to the spectrometer are recorded in the scan's metadata.yaml and oes.h5.
"""

import tkinter as tk
from tkinter import ttk

from scan.scan_params import (
    BOXCAR_WIDTH_MAX, BOXCAR_WIDTH_MIN,
    INTEGRATION_TIME_MAX_US, INTEGRATION_TIME_MIN_US,
    NUM_AVERAGES_MAX, NUM_AVERAGES_MIN,
    oes_acquisition_time_s,
    validate_boxcar_width, validate_integration_time_us, validate_num_averages,
)


def _fmt_ms(us):
    ms = float(us) / 1000.0
    return f"{ms:g}"


class OESSettingsFrame(ttk.LabelFrame):
    def __init__(self, parent, config):
        super().__init__(parent, text="OES acquisition", padding=6)
        self.integration_ms_var = tk.StringVar()
        self.averages_var = tk.StringVar()
        self.boxcar_var = tk.StringVar()
        self.summary_var = tk.StringVar()

        row = 0
        ttk.Label(self, text=(f"Integration time (ms)  [{INTEGRATION_TIME_MIN_US / 1000:g}-"
                              f"{INTEGRATION_TIME_MAX_US / 1000:g}]")).grid(
            row=row, column=0, sticky="w")
        ttk.Entry(self, textvariable=self.integration_ms_var, width=10).grid(
            row=row, column=1, sticky="w")
        row += 1
        ttk.Label(self, text=f"Averages  [{NUM_AVERAGES_MIN}-{NUM_AVERAGES_MAX}]").grid(
            row=row, column=0, sticky="w")
        ttk.Entry(self, textvariable=self.averages_var, width=10).grid(
            row=row, column=1, sticky="w")
        row += 1
        ttk.Label(self, text=(f"Boxcar half-width  [{BOXCAR_WIDTH_MIN}-{BOXCAR_WIDTH_MAX}, "
                              "0 = off, 1 = 3-pixel]")).grid(row=row, column=0, sticky="w")
        ttk.Entry(self, textvariable=self.boxcar_var, width=10).grid(
            row=row, column=1, sticky="w")
        row += 1
        ttk.Label(self, textvariable=self.summary_var, foreground="#555555",
                  wraplength=320, justify="left").grid(
            row=row, column=0, columnspan=2, sticky="w")

        for var in (self.integration_ms_var, self.averages_var, self.boxcar_var):
            var.trace_add("write", lambda *_: self._update_summary())
        self.load_config(config)

    def load_config(self, config):
        oes = config.get("oes", {})
        self.integration_ms_var.set(_fmt_ms(oes.get("integration_time_us", 20000)))
        self.averages_var.set(str(oes.get("num_averages", 1)))
        self.boxcar_var.set(str(oes.get("boxcar_width", 0)))
        self._update_summary()

    def values(self):
        """Validated settings dict; raises ValueError with an operator message."""
        try:
            integration_us = round(float(self.integration_ms_var.get()) * 1000)
        except ValueError:
            raise ValueError(
                f"Integration time must be a number of ms, got {self.integration_ms_var.get()!r}")
        return {
            "integration_time_us": validate_integration_time_us(integration_us),
            "num_averages": validate_num_averages(self.averages_var.get()),
            "boxcar_width": validate_boxcar_width(self.boxcar_var.get()),
        }

    def apply_to(self, effective_config):
        effective_config.setdefault("oes", {}).update(self.values())

    def _update_summary(self):
        try:
            v = self.values()
        except ValueError:
            self.summary_var.set("")
            return
        t = oes_acquisition_time_s(v["integration_time_us"], v["num_averages"])
        smooth = ("no smoothing" if v["boxcar_width"] == 0 else
                  f"{2 * v['boxcar_width'] + 1}-pixel smoothing on the processed copy")
        self.summary_var.set(
            f"OES read takes ~{t:.2f} s per point ({smooth}; raw spectra always saved). "
            "With IR on, this adds to the dwell; with IR off it runs within it.")
