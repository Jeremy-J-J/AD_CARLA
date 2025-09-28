#!/usr/bin/zsh

# srunner/examples/overtake_concrete.osc
python scenario_runner.py \
    --sync  \
    --openscenario2 ai_gen/cut_out_fully_blocking.osc \
    --reloadWorld 

ffmpeg -y -i scenario_video.mp4 -vcodec libx264 -acodec aac cut_out_fully_blocking.mp4

python ../CARLA/1_set_no_render.py