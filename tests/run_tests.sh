#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
make
./bin/qnx_rt_watchdog --cycles 1000
python3 -m unittest discover -s tests -v
