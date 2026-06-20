"""
Pen 13 (substrate temperature) readings from a PAC Display
Historical SuperTrend file (.T0001 etc).

- would need access to the files then

Usage:
    python ir_reader.py [RD260608.T0001] [output.csv]
"""

import sys
import csv
from datetime import datetime

PEN_ID = "13"


def parse_trend_file(in_path, out_path):
    rows = []
    with open(in_path, "r", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("//"):
                continue

            parts = line.split()
            if len(parts) != 4:
                continue

            pen, date_str, time_str, value_str = parts
            if pen != PEN_ID:
                continue

            try:
                dt = datetime.strptime(
                    f"{date_str} {time_str}", "%m/%d/%Y %H:%M:%S.%f"
                )
                value = float(value_str)
            except ValueError:
                continue

            rows.append((dt, value))

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["datetime", "timestamp", "temperature"])
        for dt, value in rows:
            writer.writerow([dt.isoformat(sep=" "), dt.isoformat(), value])

    print(f"Extracted {len(rows)} Pen {PEN_ID} readings -> {out_path}")
    return rows


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python extract_pen13_temp.py <input file> [output.csv]")
        sys.exit(1)

    in_path = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else "pen13_temperature.csv"

    parse_trend_file(in_path, out_path)