"""
capture_background.py -- capture and save an OES background without the GUI
(same code path as the GUI's "OES Background" tab). Scanner GUI CLOSED.

    python tools/capture_background.py --kind true_dark --n 20
    python tools/capture_background.py --kind plasma_off --lights off --n 20 \
        --integration-time-us 500000 --averages 3 --notes "after window clean"

Settings default to config.yaml's oes: block. Saved to oes.background_dir
(default ./backgrounds) as bg_<timestamp>_<kind>.h5 + .csv.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml

from readers.spectrometer_reader_base import get_spectrometer_reader
from scan import background as bgmod
from scan.scan_params import (validate_boxcar_width, validate_integration_time_us,
                              validate_num_averages)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--kind", choices=sorted(bgmod.KINDS), required=True)
    ap.add_argument("--lights", choices=bgmod.ROOM_LIGHTS, default="n/a")
    ap.add_argument("--n", type=int, default=bgmod.N_READINGS_DEFAULT)
    ap.add_argument("--integration-time-us", type=int)
    ap.add_argument("--averages", type=int)
    ap.add_argument("--boxcar", type=int)
    ap.add_argument("--notes", default="")
    args = ap.parse_args()

    with open(args.config, encoding="utf-8-sig") as f:
        config = yaml.safe_load(f)
    oes = config.get("oes", {})
    settings = {
        "integration_time_us": validate_integration_time_us(
            args.integration_time_us or oes.get("integration_time_us", 20000)),
        "num_averages": validate_num_averages(args.averages or oes.get("num_averages", 1)),
        "boxcar_width": validate_boxcar_width(
            args.boxcar if args.boxcar is not None else oes.get("boxcar_width", 0)),
    }
    reader = get_spectrometer_reader(config)
    try:
        if reader.wavelengths is None:
            raise SystemExit(f"Spectrometer did not connect: {getattr(reader, '_init_error', '')}")
        reader.apply_settings(**settings)
        bg = bgmod.capture(reader, args.n, args.kind, args.lights, args.notes,
                           progress=lambda k, t: print(f"\r  reading {k}/{t}", end=""))
        print()
        path = bgmod.save(bg, bgmod.background_dir(config))
    finally:
        reader.close()
    sc = bg.record["summary_counts"]
    print(f"Saved {path}\n  {bg.label()}\n  mean {sc['mean_of_mean']:.1f} counts, "
          f"min {sc['min']:.0f}, max {sc['max']:.0f}, median per-pixel std "
          f"{sc['median_pixel_std']:.2f}, saturated readings {bg.record['saturated_readings']}")


if __name__ == "__main__":
    main()
