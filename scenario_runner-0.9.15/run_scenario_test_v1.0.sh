#!/usr/bin/env zsh
# =========================================================
# run_scenario_test_v1.2.sh
# 依次启动 ScenarioRunner → follow_ego → give_autopilot
# =========================================================

set -euo pipefail
set -m                               # 支持作业控制 (&)

WAIT_BEFORE_AUTOPILOT=7              # 等待秒数，按需调整

#----------------------------------------------------------
# 1. ScenarioRunner（后台）
#----------------------------------------------------------
python scenario_runner.py \
    --sync \
    --openscenario srunner/examples/DEMO001.xosc \
    --reloadWorld \
    > scenario_runner.log 2>&1 &
# python scenario_runner.py \
#     --sync \
#     --openscenario srunner/examples/DEMO001.xosc \
#     > scenario_runner.log 2>&1 &
SCENARIO_PID=$!
echo "[INFO] scenario_runner.py started (pid=${SCENARIO_PID})"

#----------------------------------------------------------
# 2. follow_ego.py（后台）
#----------------------------------------------------------
sleep 1                               # 先给服务器 1 秒缓冲
python follow_ego.py \
    > follow_ego.log 2>&1 &
FOLLOW_PID=$!
echo "[INFO] follow_ego.py started (pid=${FOLLOW_PID})"

#----------------------------------------------------------
# 3. 固定等待 N 秒，再启 autopilot
#----------------------------------------------------------
echo "[INFO] Waiting ${WAIT_BEFORE_AUTOPILOT}s for follow_ego to settle ..."
sleep "${WAIT_BEFORE_AUTOPILOT}"

python give_autopilot.py \
    > give_autopilot.log 2>&1 &
AUTOPILOT_PID=$!
echo "[INFO] give_autopilot.py started (pid=${AUTOPILOT_PID})"

#----------------------------------------------------------
# 4. 清理函数 & 信号捕获
#----------------------------------------------------------
cleanup() {
    echo "\n[INFO] Cleaning up ..."
    kill -INT ${AUTOPILOT_PID}  ${FOLLOW_PID} 2>/dev/null || true
    kill -INT ${SCENARIO_PID}   2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup INT TERM

#----------------------------------------------------------
# 5. 等 scenario_runner 结束
#----------------------------------------------------------
wait ${SCENARIO_PID} || true
echo "[INFO] scenario_runner.py finished."

# 结束辅助脚本（若还在跑）
kill -INT ${AUTOPILOT_PID}  ${FOLLOW_PID} 2>/dev/null || true

#----------------------------------------------------------
# 6. ffmpeg 转码 & no-render
#----------------------------------------------------------
ffmpeg -y -i scenario_video.mp4 -vcodec libx264 -acodec aac out_test_v1.mp4
python ../CARLA/1_set_no_render.py

echo "[INFO] All tasks done!"