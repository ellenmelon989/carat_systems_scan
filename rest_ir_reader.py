"""
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

Confirmed endpoint (returns ALL float32 strategy variables as a JSON array):
    /api/v1/device/strategy/vars/float32s
This class fetches that list and filters by name each read. If you open
    https://<ip>/api/v1/device/strategy/vars/float32s
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
                 variable_name: str, use_https: bool = True, verify_ssl: bool = False,
                 emissivity: float = 0.0, timeout: float = 2.0):
        scheme = "https" if use_https else "http"
        self.url = f"{scheme}://{controller_ip}/api/v1/device/strategy/vars/float32s"
        self.auth = (api_key_id, api_key_value)
        self.variable_name = variable_name
        self.emissivity = emissivity
        self.verify_ssl = verify_ssl
        self.timeout = timeout
        self.session = requests.Session()

    def read(self) -> IRReading:
        read_time = time.time()
        try:
            resp = self.session.get(self.url, auth=self.auth,
                                     timeout=self.timeout, verify=self.verify_ssl)
            resp.raise_for_status()
            variables = resp.json()
            for v in variables:
                if v.get("name") == self.variable_name:
                    return IRReading(
                        value_c=float(v["value"]),
                        emissivity=self.emissivity,
                        pac_timestamp=read_time,
                        read_time=read_time,
                        stale=False,
                    )
            return IRReading(value_c=float("nan"), emissivity=self.emissivity,
                              pac_timestamp=None, read_time=read_time, stale=True,
                              error=f"variable '{self.variable_name}' not found in response")
        except requests.RequestException as e:
            return IRReading(value_c=float("nan"), emissivity=self.emissivity,
                              pac_timestamp=None, read_time=read_time, stale=True,
                              error=str(e))


if __name__ == "__main__":
    # Fill these in once REST is enabled on the controller, then run directly
    # to sanity-check a single read before wiring it into scan_manager.py.
    reader = RestApiIRReader(
        controller_ip="CONTROLLER_IP_HERE",
        api_key_id="API_KEY_ID_HERE",
        api_key_value="API_KEY_VALUE_HERE",
        variable_name="STRATEGY_VARIABLE_NAME_HERE",
    )
    print(reader.read())
