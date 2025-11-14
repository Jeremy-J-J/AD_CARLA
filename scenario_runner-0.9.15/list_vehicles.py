import carla
client = carla.Client('localhost', 2000)
world  = client.get_world()
blueprints = world.get_blueprint_library().filter('vehicle.*')
for bp in blueprints:
    print(bp.id)