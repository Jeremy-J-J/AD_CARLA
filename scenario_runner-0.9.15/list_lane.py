# file: inspect_lanes_town05.py
import random, carla, math, time

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world  = client.load_world('Town05')
town_map = world.get_map()

print("=== 所有 spawn_point 的 road/lane/s 信息 ===")
for idx, sp in enumerate(town_map.get_spawn_points()):
    wp = town_map.get_waypoint(sp.location,
                               project_to_road=True,
                               lane_type=carla.LaneType.Any)
    print(f"[{idx:02d}] road={wp.road_id:3d}, lane={wp.lane_id:3d}, "
          f"s={wp.s:6.1f},  x={sp.location.x:7.2f}, y={sp.location.y:7.2f}")

print("\n=== 随机采样 200 个位置 ===")
for _ in range(200):
    loc = carla.Location(random.uniform(-150, 150),
                         random.uniform(-150, 150),
                         random.uniform(0, 5))
    wp = town_map.get_waypoint(loc, project_to_road=True,
                               lane_type=carla.LaneType.Any)
    if wp:
        print(f"road={wp.road_id:3d}, lane={wp.lane_id:3d}, "
              f"s={wp.s:6.1f}")