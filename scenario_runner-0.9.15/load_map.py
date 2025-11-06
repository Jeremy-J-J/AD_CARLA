import carla

# CARLA仿真器的默认端口就是 http://localhost:2000，所以这里这样配置
client = carla.Client('localhost', 2000)

# 这里就是加载你的地图了，输入绝对路径
# 需要注意地图给777权限(777没问题，其他权限没试，可能可以小一点)，不然权限问题加载不了
with open('/workspace/pro/carla/AD_CARLA/scenario_runner-0.9.15/srunner/examples/road_networks/alks_road_straight.xodr') as f:
    world = client.generate_opendrive_world(f.read())
print('Map:', world.get_map().name)