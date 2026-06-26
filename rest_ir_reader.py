"""
PAC firmware confirmed 10.0 and 10.4 - use this file

rest_ir_reader.py — Opto 22 SNAP PAC REST API backend. PREFERRED PATH.

Endpoint pattern confirmed from Opto 22's own published SNAP PAC code
samples (developer.opto22.com/rest/pac/examples). Ready to run once the
controller-side setup below is done.

One-time setup on the controller (developer.opto22.com/rest/pac/quickstart):
  1. PAC Manager: Tools > Inspect > Communications > Network Security ->
     enable the HTTP/HTTPS server, set a listen port, store to flash, reboot.
  2. Browser: https://<ip>/admin/creds -> log in admin/password (defaults)
     -> set a real admin username/password.
  3. Browser: https://<ip>/admin/keys -> create a read-only API key.
  4. In PAC Control, find the exact strategy variable name that holds the
     converted IR temperature (Debug > Inspect on that float variable).

Confirmed endpoint (returns ALL float strategy variables as a JSON array):
    /api/v1/device/strategy/vars/floats
This class fetches that list and filters by name each read. If you open
    https://<ip>/api/v1/device/strategy/vars/floats
in a browser (Swagger UI, once the API's enabled) and find a single-variable
endpoint, switch to that -- lower latency than pulling the whole list every
point.

Requires firmware R9.5a or higher. Check this before relying on this path.

Requires: pip install requests
"""

import time
import requests

from ir_reader_base import IRReader, IRReading


class RestApiIRReader(IRReader):
    def __init__(self, controller_ip: str, api_key_id: str, api_key_value: str,
                variable_name: str, use_https: bool = False, verify_ssl: bool = False,
                timeout: float = 2.0):
        scheme = "https" if use_https else "http"
        base = f"{scheme}://{controller_ip}/api/v1/device/strategy/vars"
        self.url_floats  = f"{base}/floats"
        self.url_int32s  = f"{base}/int32s"
        self.auth        = (api_key_id, api_key_value)
        self.variable_name = variable_name
        self.verify_ssl  = verify_ssl
        self.timeout     = timeout
        self.session     = requests.Session()

    def _get_float(self, name: str) -> float:
        resp = self.session.get(f"{self.url_floats}/{name}", auth=self.auth,
                                timeout=self.timeout, verify=self.verify_ssl)
        resp.raise_for_status()
        return float(resp.json()["value"])

    def _get_int32(self, name: str) -> int:
        resp = self.session.get(f"{self.url_int32s}/{name}", auth=self.auth,
                                timeout=self.timeout, verify=self.verify_ssl)
        resp.raise_for_status()
        return int(resp.json()["value"])

    def read(self) -> IRReading:
        read_time = time.time()
        try:
            invalid = self._get_int32("PyroTempInvalid")
            if invalid != 0:
                return IRReading(value_c=float("nan"), emissivity=float("nan"),
                                pac_timestamp=None, read_time=read_time, stale=True,
                                error="PyroTempInvalid flag set")
            temp  = self._get_float(self.variable_name)      # iai_PYRO_TEMP
            emis  = self._get_float("iai_PYRO_EMISSIVITY")
            return IRReading(value_c=temp, emissivity=emis,
                            pac_timestamp=read_time, read_time=read_time, stale=False)
        except requests.RequestException as e:
            return IRReading(value_c=float("nan"), emissivity=float("nan"),
                            pac_timestamp=None, read_time=read_time, stale=True,
                            error=str(e))

if __name__ == "__main__":
    import yaml
    with open("config.yaml") as f:
        config = yaml.safe_load(f)
    pac = config["ir"]["pac"]
    reader = RestApiIRReader(
        controller_ip=pac["ip"],
        api_key_id=pac["api_key_id"],
        api_key_value=pac["api_key_value"],
        variable_name=pac["tag_name"],
    )
    print(reader.read())
