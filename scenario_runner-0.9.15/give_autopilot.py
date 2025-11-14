#!/usr/bin/env python3
import carla, time

client = carla.Client('localhost', 2000)
client.set_timeout(2.0)
world = client.get_world()
tm = client.get_trafficmanager()   # Traffic Manager 端口默认 8000

# 等待 ScenarioRunner 把 ego spawn 出来
# 注意：scenario_runner中ego车辆的role_name可能是 'Ego' 或 'hero'
ego = None
while ego is None:
    actors = world.get_actors().filter('vehicle.*')
    # 先尝试找 'hero'，然后尝试 'Ego'，最后找带有 'ego' 的
    ego = next((a for a in actors if a.attributes.get('role_name') == 'hero'), None)
    if ego is None:
        ego = next((a for a in actors if a.attributes.get('role_name') == 'Ego'), None)
    if ego is None:
        ego = next((a for a in actors if 'ego' in a.attributes.get('role_name', '').lower()), None)
    time.sleep(0.05)

print(f'Found ego vehicle with role_name: {ego.attributes.get("role_name")}')

# 警告：使用autopilot会覆盖场景中定义的FollowTrajectoryAction
# 如果场景已经定义了轨迹跟随，不应该启用autopilot
print('WARNING: Enabling autopilot will override any FollowTrajectoryAction in the scenario!')
ego.set_autopilot(True, tm.get_port())
print('Ego set to autopilot')