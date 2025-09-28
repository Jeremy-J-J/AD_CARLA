#!/usr/bin/env python3
import carla, time, math

client = carla.Client('localhost', 2000)
client.set_timeout(2.0)
world = client.get_world()

ego = None
while ego is None:
    actors = world.get_actors().filter('*vehicle*')
    ego = next((a for a in actors if a.attributes.get('role_name') == 'hero'), None)
    time.sleep(0.1)

spectator = world.get_spectator()

def follow():
    transform = ego.get_transform()
    loc = transform.location + carla.Location(x=-8*math.cos(math.radians(transform.rotation.yaw)),
                                              y=-8*math.sin(math.radians(transform.rotation.yaw)),
                                              z=6)
    rot = carla.Rotation(pitch=-15, yaw=transform.rotation.yaw)
    spectator.set_transform(carla.Transform(loc, rot))

while True:
    world.wait_for_tick()
    follow()