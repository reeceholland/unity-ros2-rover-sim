#!/usr/bin/env bash
set -eo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
logs="${CI_LOG_DIR:-$HOME/rover-debug/collision-validation-$(date +%Y%m%d-%H%M%S)}"
for fixture in baseline wall missing; do
  status=0
  CI_LOG_DIR="$logs/$fixture" CI_COLLISION_FIXTURE="$fixture" \
    xvfb-run -a bash "$here/run_headless.sh" collision || status=$?
  python3 - "$logs/$fixture/collision-results.json" "$fixture" "$status" <<'CHECK'
import json, sys
from pathlib import Path
path, fixture, status = Path(sys.argv[1]), sys.argv[2], int(sys.argv[3])
if not path.exists():
    raise SystemExit('Validation failed: no report (setup or process failure)')
r = json.loads(path.read_text())
if fixture == 'baseline':
    valid = status == 0 and r['passed'] and r['samples'] > 0
elif fixture == 'wall':
    valid = status == 1 and not r['passed'] and 'CI_IntentionalContactWall' in (r['failure'] or '')
else:
    valid = status == 1 and not r['passed'] and r['failure'] == 'Missing collision telemetry'
if not valid:
    raise SystemExit(f'{fixture}: unexpected outcome: {r}')
print(f'{fixture}: expected CI outcome verified')
CHECK
done
echo "Collision validation evidence: $logs"
