#!/usr/bin/zsh

# srunner/examples/overtake_concrete.osc
python scenario_runner.py \
    --sync  \
    --openscenario2 srunner/examples/Volkswagen_Demo/DEMO001/DEMO001.osc \
    --reloadWorld 

ffmpeg -i scenario_video.mp4 -vcodec libx264 -acodec aac out_test_v2.mp4

python ../CARLA/1_set_no_render.py