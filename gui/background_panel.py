"""
gui/background_panel.py -- "OES Background" tab (Batch 2, 2026-10-19).

Capture and save OES backgrounds before a scan:
  * True dark  -- block the light input (cap the fiber / close the shutter)
  * Plasma off -- normal optical path, plasma off; capture with room lights
                  on and with them off to see their effect
Each capture averages N readings at the acquisition settings shown here
(default: the same config.yaml values as the Scan tab -- use "Copy from
Scan tab" to match exactly) and is saved to oes.background_dir
(default ./backgrounds) with its settings and timestamps. The saved list
below feeds the "Background" picker on the Scan and Line Scan tabs.

Hardware: opens its OWN spectrometer connection only for the duration of a
capture and always closes it afterwards. app.py refuses a capture while a
scan is running or another tab holds the spectrometer (can_capture hook).
"""

import queue
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from gui.oes_settings import OESSettingsFrame
from readers.spectrometer_reader_base import get_spectrometer_reader
from scan import background as bgmod

POLL_MS = 150


class BackgroundPanel(ttk.Frame):
    def __init__(self, parent, config, can_capture=None, get_scan_settings=None,
                 on_saved=None):
        super().__init__(parent, padding=8)
        self.config_ = config
        self.can_capture = can_capture or (lambda: True)
        self.get_scan_settings = get_scan_settings
        self.on_saved = on_saved
        self.q = queue.Queue()
        self.worker = None
        self.running = False
        self._cancel = threading.Event()
        self._rows = []

        self.columnconfigure(1, weight=1)
        self.rowconfigure(1, weight=1)

        form = ttk.LabelFrame(self, text="Capture a background", padding=8)
        form.grid(row=0, column=0, sticky="nw")
        r = 0
        ttk.Label(form, text=(
            "True dark: block the light input.  Plasma off: normal optical path,\n"
            "plasma off -- capture once with room lights on and once with them off."),
            foreground="#555555", justify="left").grid(row=r, column=0, columnspan=3, sticky="w")
        r += 1
        ttk.Label(form, text="Type").grid(row=r, column=0, sticky="w")
        self.kind_var = tk.StringVar(value=bgmod.KINDS["plasma_off"])
        ttk.Combobox(form, textvariable=self.kind_var, state="readonly", width=32,
                     values=list(bgmod.KINDS.values())).grid(row=r, column=1, sticky="w")
        r += 1
        ttk.Label(form, text="Room lights").grid(row=r, column=0, sticky="w")
        self.lights_var = tk.StringVar(value="off")
        ttk.Combobox(form, textvariable=self.lights_var, state="readonly", width=8,
                     values=list(bgmod.ROOM_LIGHTS)).grid(row=r, column=1, sticky="w")
        r += 1
        ttk.Label(form, text=(f"Readings to average  [{bgmod.N_READINGS_MIN}-"
                              f"{bgmod.N_READINGS_MAX}]")).grid(row=r, column=0, sticky="w")
        self.n_var = tk.StringVar(value=str(bgmod.N_READINGS_DEFAULT))
        ttk.Entry(form, textvariable=self.n_var, width=8).grid(row=r, column=1, sticky="w")
        r += 1
        ttk.Label(form, text="Notes").grid(row=r, column=0, sticky="w")
        self.notes_var = tk.StringVar()
        ttk.Entry(form, textvariable=self.notes_var, width=36).grid(
            row=r, column=1, columnspan=2, sticky="we")
        r += 1
        self.oes_settings = OESSettingsFrame(form, config)
        self.oes_settings.grid(row=r, column=0, columnspan=3, sticky="we", pady=(6, 0))
        r += 1
        if get_scan_settings is not None:
            ttk.Button(form, text="Copy settings from Scan tab",
                       command=self._copy_scan_settings).grid(row=r, column=0, sticky="w",
                                                              pady=(4, 0))
            r += 1
        self.capture_btn = ttk.Button(form, text="Capture background", command=self._start)
        self.capture_btn.grid(row=r, column=0, pady=(10, 0), sticky="we")
        self.cancel_btn = ttk.Button(form, text="Cancel", command=self._cancel.set,
                                     state="disabled")
        self.cancel_btn.grid(row=r, column=1, pady=(10, 0), sticky="w")
        r += 1
        self.status_var = tk.StringVar(value="")
        ttk.Label(form, textvariable=self.status_var, wraplength=380, justify="left").grid(
            row=r, column=0, columnspan=3, sticky="w", pady=(6, 0))

        fig_frame = ttk.Frame(self)
        fig_frame.grid(row=0, column=1, sticky="nsew", padx=(10, 0))
        self.figure = Figure(figsize=(6, 3.2), dpi=90)
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=fig_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self._plot(None)

        lst = ttk.LabelFrame(self, text="Saved backgrounds (newest first)", padding=6)
        lst.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(8, 0))
        lst.columnconfigure(0, weight=1)
        lst.rowconfigure(0, weight=1)
        cols = ("captured", "type", "lights", "integration", "averages", "boxcar", "n",
                "mean", "notes")
        self.tree = ttk.Treeview(lst, columns=cols, show="headings", height=7)
        widths = (130, 90, 50, 80, 65, 55, 40, 70, 260)
        for c, w in zip(cols, widths):
            self.tree.heading(c, text=c)
            self.tree.column(c, width=w, anchor="w")
        self.tree.grid(row=0, column=0, sticky="nsew")
        self.tree.bind("<<TreeviewSelect>>", self._on_select)
        ttk.Button(lst, text="Refresh", command=self.refresh).grid(row=1, column=0, sticky="w")
        self.refresh()

    # ------------------------------------------------------------------
    def load_config(self, config):
        self.config_ = config
        self.oes_settings.load_config(config)
        self.refresh()

    def _copy_scan_settings(self):
        try:
            v = self.get_scan_settings()
        except ValueError as exc:
            messagebox.showerror("Scan tab settings invalid", str(exc))
            return
        self.oes_settings.integration_ms_var.set(f"{v['integration_time_us'] / 1000:g}")
        self.oes_settings.averages_var.set(str(v["num_averages"]))
        self.oes_settings.boxcar_var.set(str(v["boxcar_width"]))

    def refresh(self):
        self._rows = bgmod.list_saved(bgmod.background_dir(self.config_))
        self.tree.delete(*self.tree.get_children())
        for i, row in enumerate(self._rows):
            rec, acq = row["record"], row["record"].get("acquisition", {})
            it = acq.get("integration_time_us")
            self.tree.insert("", "end", iid=str(i), values=(
                rec.get("captured_at", "").replace("T", " "),
                rec.get("kind", ""), rec.get("room_lights", ""),
                f"{it / 1000:g} ms" if isinstance(it, (int, float)) else "?",
                acq.get("num_averages", "?"), acq.get("boxcar_width", "?"),
                rec.get("n_readings", "?"),
                f"{rec.get('summary_counts', {}).get('mean_of_mean', float('nan')):.1f}",
                rec.get("notes", "")))

    def _on_select(self, _event=None):
        sel = self.tree.selection()
        if not sel:
            return
        try:
            self._plot(bgmod.load(self._rows[int(sel[0])]["path"]))
        except Exception as exc:
            self.status_var.set(f"Could not load background: {exc}")

    def _plot(self, bg):
        ax = self.ax
        ax.clear()
        ax.set_xlabel("Wavelength (nm)")
        ax.set_ylabel("Counts")
        if bg is None:
            ax.set_title("Select or capture a background", fontsize=9)
        else:
            w = bg.wavelengths
            ax.fill_between(w, bg.mean - bg.std, bg.mean + bg.std, alpha=0.25, linewidth=0,
                            label="± 1 std across readings")
            ax.plot(w, bg.mean, linewidth=0.8, label="mean")
            ax.legend(fontsize=7, loc="upper right")
            ax.set_title(bg.label(), fontsize=7)
        self.figure.tight_layout()
        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    def _start(self):
        if self.running:
            return
        try:
            n = bgmod.validate_n_readings(self.n_var.get())
            settings = self.oes_settings.values()
        except ValueError as exc:
            messagebox.showerror("Invalid background settings", str(exc))
            return
        if not self.can_capture():
            return
        kind = {v: k for k, v in bgmod.KINDS.items()}[self.kind_var.get()]
        est = n * settings["integration_time_us"] * 1e-6 * settings["num_averages"]
        self.status_var.set(f"Capturing {n} readings (~{est:.1f} s)...")
        self._cancel.clear()
        self.running = True
        self.capture_btn.config(state="disabled")
        self.cancel_btn.config(state="normal")
        self.worker = threading.Thread(
            target=self._worker,
            args=(dict(self.config_), settings, n, kind, self.lights_var.get(),
                  self.notes_var.get().strip()),
            daemon=True)
        self.worker.start()
        self.after(POLL_MS, self._poll)

    def _worker(self, config, settings, n, kind, lights, notes):
        reader = None
        try:
            reader = get_spectrometer_reader(config)
            if reader.wavelengths is None:
                raise RuntimeError("Spectrometer did not connect"
                                   + (f" ({reader._init_error})"
                                      if getattr(reader, "_init_error", None) else ""))
            reader.apply_settings(**settings)
            bg = bgmod.capture(reader, n, kind, lights, notes,
                               progress=lambda k, t: self.q.put(("progress", (k, t))),
                               stop=self._cancel.is_set)
            path = bgmod.save(bg, bgmod.background_dir(config))
            self.q.put(("done", (bg, path)))
        except Exception as exc:
            self.q.put(("error", str(exc)))
        finally:
            if reader is not None:
                try:
                    reader.close()
                except Exception:
                    pass

    def _poll(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "progress":
                    self.status_var.set(f"Reading {payload[0]}/{payload[1]}...")
                elif kind == "done":
                    bg, path = payload
                    self._finish()
                    rec = bg.record
                    sat = (f"  WARNING: {rec['saturated_readings']} reading(s) saturated."
                           if rec["saturated_readings"] else "")
                    self.status_var.set(
                        f"Saved {path}\nmean {rec['summary_counts']['mean_of_mean']:.1f} counts, "
                        f"median per-pixel std {rec['summary_counts']['median_pixel_std']:.2f}."
                        + sat)
                    self._plot(bg)
                    self.notes_var.set("")     # notes describe ONE capture
                    self.refresh()
                    if self.on_saved is not None:
                        self.on_saved()
                    return
                elif kind == "error":
                    self._finish()
                    self.status_var.set(f"Capture failed: {payload}")
                    messagebox.showerror("Background capture failed", payload)
                    return
        except queue.Empty:
            pass
        self.after(POLL_MS, self._poll)

    def _finish(self):
        self.running = False
        self.capture_btn.config(state="normal")
        self.cancel_btn.config(state="disabled")

    def shutdown(self):
        if self.running:
            self._cancel.set()
            if self.worker is not None:
                self.worker.join(timeout=5)


class BackgroundSelector(ttk.Frame):
    """Scan-start picker: None or one saved background (Batch 2)."""

    NONE = "None (no background subtraction)"

    def __init__(self, parent, config):
        super().__init__(parent)
        self.config_ = config
        self._paths = {}
        ttk.Label(self, text="Background").grid(row=0, column=0, sticky="w")
        self.var = tk.StringVar(value=self.NONE)
        self.combo = ttk.Combobox(self, textvariable=self.var, state="readonly", width=48)
        self.combo.grid(row=0, column=1, sticky="we", padx=(4, 4))
        ttk.Button(self, text="Refresh", command=self.refresh, width=8).grid(row=0, column=2)
        self.columnconfigure(1, weight=1)
        self.refresh()

    def load_config(self, config):
        self.config_ = config
        self.refresh()

    def refresh(self):
        rows = bgmod.list_saved(bgmod.background_dir(self.config_))
        self._rows = {r["label"]: r for r in rows}
        labels = [self.NONE] + [r["label"] for r in rows]
        self.combo.config(values=labels)
        if self.var.get() not in labels:
            self.var.set(self.NONE)

    def selected(self):
        return self._rows.get(self.var.get())

    def confirm_and_apply(self, effective_config, scan_settings) -> bool:
        """
        Put the choice into effective_config["oes"]["background_file"].
        If the picked background's settings differ from the scan's, ask
        before starting (the scan also records the mismatch). Returns False
        if the operator backs out.
        """
        row = self.selected()
        oes = effective_config.setdefault("oes", {})
        if row is None or not oes.get("enabled", True):
            oes["background_file"] = None
            return True
        acq = row["record"].get("acquisition", {})
        diffs = [f"{k}: background {acq.get(k)}, scan {scan_settings.get(k)}"
                 for k in bgmod.COMPARED_SETTINGS
                 if acq.get(k) is not None and int(acq.get(k)) != int(scan_settings.get(k))]
        if diffs and not messagebox.askyesno(
                "Background settings differ",
                "The selected background was captured with different acquisition "
                "settings:\n  - " + "\n  - ".join(diffs)
                + "\n\nIt will still be subtracted, and the mismatch is recorded "
                  "with the scan. Start anyway?"):
            return False
        oes["background_file"] = row["path"]
        return True
