"""
list_pac_strategy_vars.py — RUN THIS to find the REST strategy variable
name for a tag you only know by its PAC Display Float Table address
(e.g. "TrendWater[4]", "PyroValues[6]").

Why this is needed: TrendWater[4] / PyroValues[6] are PAC DISPLAY
addresses -- an index into a Float Table tag that PAC Display's
SuperTrend pens read from. The REST API used by rest_ir_reader.py
(and this script) instead addresses points by their PAC CONTROL
strategy variable NAME (e.g. "iai_PYRO_TEMP") -- a different naming
scheme for potentially the same underlying signal. There is no
formula that converts one to the other; it has to be looked up.

Two ways to find the name that feeds a given Float Table slot:

  1. PAC Control (the strategy file), on the machine used to build/edit
     it: open the chart, find the Float Table named "TrendWater" (or
     "PyroValues"), and see what writes to index [4] (or [6]). Debug >
     Inspect on that source variable shows its exact strategy variable
     name -- this is the authoritative source, use it if you have
     access to PAC Control.

  2. This script, if you don't have PAC Control access but do have
     REST API credentials: dumps every float (and int32) strategy
     variable the controller exposes, so you can:
       a) grep the names for likely candidates (dilution/water/pyro), or
       b) run it twice -- once at a known dilution setting, once after
          changing it -- and diff the two dumps to see which variable's
          VALUE moved. This works even if the name gives no hint.

Requires: pip install requests pyyaml
Usage:    python list_pac_strategy_vars.py [--config config.yaml] [--filter dilut,water,pyro]
"""

import argparse
import sys

import requests
import yaml


def fetch_list(base_url, kind, auth, verify_ssl, timeout):
    """
    GET the "list all" endpoint for a var kind ("floats" or "int32s").
    Response shape isn't nailed down in code yet (only the single-var
    /floats/{name} endpoint is exercised elsewhere in this repo) -- so
    this normalizes a few plausible shapes rather than assuming one.
    Returns a dict of {name: value}; falls back to {name: None} if only
    names come back.
    """
    url = f"{base_url}/{kind}"
    resp = requests.get(url, auth=auth, timeout=timeout, verify=verify_ssl)
    resp.raise_for_status()
    data = resp.json()

    result = {}
    if isinstance(data, dict):
        # e.g. {"varname": 123.4, ...} or {"floats": [...]}
        if all(isinstance(v, (int, float, type(None))) for v in data.values()):
            result = dict(data)
        else:
            for key in ("floats", "int32s", "vars", "value", "values"):
                if key in data and isinstance(data[key], list):
                    data = data[key]
                    break
    if isinstance(data, list):
        for item in data:
            if isinstance(item, dict):
                name = item.get("name") or item.get("varName") or item.get("tag")
                value = item.get("value")
                if name is not None:
                    result[name] = value
            elif isinstance(item, str):
                result[item] = None

    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--config", default="config.yaml",
                         help="Path to config YAML with ir.pac credentials (default: config.yaml)")
    parser.add_argument("--filter", default="dilut,water,pyro,iai",
                         help="Comma-separated, case-insensitive substrings to highlight "
                              "(default: dilut,water,pyro,iai). Pass '' to disable filtering "
                              "and print every variable.")
    args = parser.parse_args()

    with open(args.config) as f:
        config = yaml.safe_load(f)

    pac = config["ir"]["pac"]
    if not pac.get("ip"):
        sys.exit("ir.pac.ip is blank in the config — fill in the controller IP first.")

    scheme = "https" if pac.get("use_https") else "http"
    base_url = f"{scheme}://{pac['ip']}/api/v1/device/strategy/vars"
    auth = (pac["api_key_id"], pac["api_key_value"])
    verify_ssl = pac.get("verify_ssl", False)
    timeout = pac.get("timeout", 5.0)

    keywords = [k.strip().lower() for k in args.filter.split(",") if k.strip()]

    all_vars = {}
    for kind in ("floats", "int32s"):
        try:
            all_vars.update(fetch_list(base_url, kind, auth, verify_ssl, timeout))
        except requests.RequestException as e:
            print(f"WARNING: couldn't fetch {kind}: {e}", file=sys.stderr)

    if not all_vars:
        sys.exit(
            "No variables returned. Check ir.pac.ip/api_key_id/api_key_value in "
            "the config, and that the controller's REST API is enabled "
            "(see rest_ir_reader.py's module docstring for the one-time setup)."
        )

    print(f"{len(all_vars)} strategy variable(s) found.\n")

    if keywords:
        matches = {
            name: value for name, value in all_vars.items()
            if any(kw in name.lower() for kw in keywords)
        }
        print(f"Matching filter {keywords}:")
        if matches:
            for name, value in sorted(matches.items()):
                print(f"  {name} = {value}")
        else:
            print("  (none — try --filter '' to see everything, or diff two full dumps "
                  "while toggling the dilution setting to spot the variable that moves)")
        print()

    print("All variables:")
    for name, value in sorted(all_vars.items()):
        print(f"  {name} = {value}")


if __name__ == "__main__":
    main()
