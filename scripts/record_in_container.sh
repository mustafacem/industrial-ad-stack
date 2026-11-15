#!/bin/bash
# Inside the container: virtual X screen -> ros2 launch (Gazebo + stack + RViz) -> two recordings of one run:
#   output/raw/<name>.mp4         RViz 3D panel (ffmpeg x11grab, wall clock)
#   output/raw/<name>_gz.mp4      Gazebo chase camera (GPU, CameraVideoRecorder, sim time)   [REC_GZ=1]
#   output/raw/<name>_clock.txt   wall/sim time pairs, to put the RViz capture on the sim-time axis
#   record_in_container.sh <name> <sim seconds> <package> <launch file> [launch args...]
set -u
NAME=$1; DUR=$2; shift 2
OUT=/ws/output/raw; mkdir -p "$OUT"
W=1970; H=1189                                     # RViz window incl. menu/tool/status bars
CW=${REC_W:-1920}; CH=${REC_H:-1080}; CX=${REC_X:-25}; CY=${REC_Y:-71}
LAYOUT=${REC_LAYOUT:-single}
FILTER=""
if [ "$LAYOUT" = split ]; then   # two 960x1080 RViz panels side by side -> one 1920x1080 video
  W=2020; CW=2020; CH=1189; CX=0; CY=0
  FILTER="-filter_complex [0:v]crop=960:1080:25:71[a];[0:v]crop=960:1080:1035:71[b];[a][b]hstack=inputs=2,drawbox=x=958:y=0:w=4:h=1080:color=0x0d0f14@1:t=fill[v] -map [v]"
fi
FPS=${REC_FPS:-30}; WARM=${REC_WARMUP:-14}
Xvfb :99 -screen 0 ${W}x${H}x24 -nolisten tcp >/dev/null 2>&1 &
XPID=$!
export DISPLAY=:99
sleep 1
GZCAM=false; [ "${REC_GZ:-1}" = 1 ] && GZCAM=true
# The world starts paused; sim_supervisor unpauses it WARM seconds after launch (RViz is up by then).
ros2 launch "$@" rviz:=true cameras:=$GZCAM start_after:=$WARM duration:=$DUR > "$OUT/$NAME.launch.log" 2>&1 &
LPID=$!
sleep $((WARM - 3))
if [ "$GZCAM" = true ]; then
  rm -f "$OUT/${NAME}_gz.mp4"
  gz service -s /iad/chase_cam/record --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 5000 \
     --req "start: true, format: \"mp4\", save_filename: \"$OUT/${NAME}_gz.mp4\"" > /dev/null 2>&1
fi
ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -framerate "$FPS" -video_size "${CW}x${CH}" -i ":99.0+${CX},${CY}" \
       $FILTER -c:v libx264 -preset veryfast -crf 17 -pix_fmt yuv420p "$OUT/$NAME.mp4" &
FPID=$!
# wall/sim clock log until the sim duration is reached (then stop both recordings)
python3 - "$OUT/${NAME}_clock.txt" "$DUR" <<'PY'
import sys, time, rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
rclpy.init(); n = Node('rec_clock'); f = open(sys.argv[1], 'w'); dur = float(sys.argv[2]); st = {'t': 0.0}
def on(m):
    st['t'] = m.clock.sec + 1e-9 * m.clock.nanosec
n.create_subscription(Clock, '/clock', on, 10)
t_end = time.time() + 4 * dur + 120
while time.time() < t_end and st['t'] < dur - 0.5:
    rclpy.spin_once(n, timeout_sec=0.05)
    w = time.time()
    if w - st.get('w', 0) >= 0.02 and st['t'] >= st.get('s', 0):  # strictly increasing wall, monotonic sim
        f.write('%.3f %.3f\n' % (w, st['t']))
        st['w'], st['s'] = w, st['t']
f.close()
PY
if [ "$GZCAM" = true ]; then
  gz service -s /iad/chase_cam/record --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 5000 --req "stop: true" > /dev/null 2>&1
fi
sleep 1
kill -INT $FPID 2>/dev/null; wait $FPID 2>/dev/null
kill -INT $LPID 2>/dev/null; sleep 4
pkill -9 -f rviz2 2>/dev/null; pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f "lib/iad_" 2>/dev/null; kill $XPID 2>/dev/null
echo "recorded $OUT/$NAME.mp4 $( [ "$GZCAM" = true ] && echo "+ ${NAME}_gz.mp4")"
