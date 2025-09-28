#!/usr/bin/zsh

# srunner/examples/overtake_concrete.osc
python scenario_runner_debug.py \
    --sync  \
    --openscenario2 ai_gen/out.osc \
    --reloadWorld 

ffmpeg -i scenario_video.mp4 -vcodec libx264 -acodec aac out_test.mp4

python ../CARLA/1_set_no_render.py