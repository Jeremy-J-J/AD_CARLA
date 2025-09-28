#!/usr/bin/env python3
import carla, time

client = carla.Client('localhost', 2000)
client.set_timeout(2.0)
world = client.get_world()
tm = client.get_trafficmanager()   # Traffic Manager 端口默认 8000

# 等待 ScenarioRunner 把 ego spawn 出来
ego = None
while ego is None:
    actors = world.get_actors().filter('vehicle.*')
    ego = next((a for a in actors if a.attributes.get('role_name') == 'hero'), None)
    time.sleep(0.05)

ego.set_autopilot(True, tm.get_port())
print('Ego set to autopilot')