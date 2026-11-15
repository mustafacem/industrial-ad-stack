#!/usr/bin/env bash
# Independence check: builds and tests every subsystem package ALONE, in an empty workspace that
# contains nothing but that package (no iad_* siblings, no install/ of this repo). A package that
# still needs code or messages from another iad_* package fails here.
#   scripts/isolation_check.sh [pkg ...]
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKGS=("$@")
[ ${#PKGS[@]} -eq 0 ] && PKGS=(iad_perception iad_localization iad_calibration iad_planning iad_control)
OUT="$ROOT/output/isolation"
rm -rf "$OUT" && mkdir -p "$OUT"
status=0
for p in "${PKGS[@]}"; do
  ws="$OUT/ws_$p"
  mkdir -p "$ws/src" && cp -r "$ROOT/src/$p" "$ws/src/" && rm -f "$ws/src/$p/COLCON_IGNORE"
  deps=$(grep -oE "<(build_|exec_|test_)?depend>iad_[a-z_]+" "$ws/src/$p/package.xml" | sed 's/.*>//' | sort -u | tr '\n' ' ')
  if [ -n "$deps" ]; then echo "FAIL $p: package.xml depends on $deps"; status=1; continue; fi
  if grep -rqE '#include [<"]iad_(common|msgs|sim|gz_sim)/' "$ws/src/$p"; then echo "FAIL $p: includes another iad_* package"; status=1; continue; fi
  rel="output/isolation/ws_$p"
  if IAD_GPU=0 "$ROOT/scripts/dev.sh" bash -c "cd /ws/$rel && unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH; source /opt/ros/humble/setup.bash && \
      colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON > build.log 2>&1 && \
      colcon test --return-code-on-test-failure > test.log 2>&1 && colcon test-result --verbose > result.log 2>&1"; then
    echo "PASS $p  ($(grep -oE '[0-9]+ tests, [0-9]+ errors, [0-9]+ failures, [0-9]+ skipped' "$ws/result.log" | tail -1))"
  else
    echo "FAIL $p  (see $rel/{build,test,result}.log)"; status=1
  fi
done
exit $status
