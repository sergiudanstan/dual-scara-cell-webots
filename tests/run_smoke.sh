#!/usr/bin/env bash
# Headless cell test. SMOKE_T is simulated seconds (default 55).
#   tests/run_smoke.sh
set -eu
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WEBOTS=${WEBOTS:-/Applications/Webots.app/Contents/MacOS/webots}
T=${SMOKE_T:-55}
tmp="$ROOT/worlds/.smoke.wbt"
sed "s/controller \"cell_plc\"/controller \"cell_plc\" controllerArgs [ \"--smoke=$T\" ]/" \
  "$ROOT/worlds/dual_scara_cell.wbt" > "$tmp"
log="$ROOT/tests/.smoke.log"
status=0
perl -e 'alarm shift; exec @ARGV' "${SMOKE_TIMEOUT:-900}" \
  "$WEBOTS" --batch --mode=fast --minimize --no-rendering --stdout --stderr "$tmp" > "$log" 2>&1 || status=$?
rm -f "$tmp"
grep -E "\[SMOKE\]|\[PLC|Traceback|Error" "$log" | tail -50
grep -q "\[SMOKE\] PASS" "$log" && [ "$status" = 0 ] && { echo "smoke test: PASS"; exit 0; }
echo "smoke test: FAIL (webots exit $status, log $log)"
exit 1
