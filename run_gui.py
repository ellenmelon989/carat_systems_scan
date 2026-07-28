"""
run_gui.py

Entry point for the carat_scanner GUI. Mirrors scan_manager.py's own
--config argparse pattern. Run from the repo root:

    python run_gui.py [--config config.yaml]

so the flat `from motion...` / `from readers...` / `from gui...`
imports resolve exactly the way they already do for every other
script in this repo (no package install, no sys.path hacking --
namespace packages resolved relative to cwd).
"""

import argparse

import yaml

from gui.app import App


def main():
    parser = argparse.ArgumentParser(
        description="Launch the carat_scanner GUI, seeded from a config YAML.",
    )
    parser.add_argument(
        "--config", default="config.yaml",
        help="Path to the config YAML to load as the GUI's starting values (default: config.yaml).",
    )
    args = parser.parse_args()

    # utf-8-sig: tolerates (and strips) a UTF-8 BOM if one is present,
    # transparent no-op if not -- config.yaml is a hand-edited file an
    # operator may open/save in Notepad on Windows, which can add a BOM.
    # A BOM landing on the first key (e.g. "﻿scan:") makes
    # yaml.safe_load either error or silently misparse the top-level key,
    # so config["scan"] etc. below would KeyError. gui/calibration_panel.py
    # and scan/calibrate_scan_area.py already guard against this when
    # re-reading config.yaml after a Calibrate-tab write; this is the same
    # guard applied to every other place config.yaml gets read, including
    # this one -- the GUI's own entry point, which previously did not have it.
    with open(args.config, encoding="utf-8-sig") as f:
        config = yaml.safe_load(f)

    # config_path is threaded through to the Calibrate tab (see
    # gui/calibration_panel.py) so it can patch the SAME file that was
    # just loaded here, exactly like calibrate_scan_area.py's own
    # `python calibrate_scan_area.py [config.yaml]` usage.
    app = App(config, args.config)
    app.mainloop()


if __name__ == "__main__":
    main()
