#!/usr/bin/env python3
"""moza-rudder-sim: a simulated rudder with Slipway's steering gear, for testing the wheel. See README.md."""

import sys

from moza_connector.rudder_sim_app import main

if __name__ == "__main__":
    sys.exit(main())
