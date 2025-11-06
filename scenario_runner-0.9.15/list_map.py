import carla

client = carla.Client('localhost', 2000)
client.set_timeout(5)

print("Available maps:")
for name in client.get_available_maps():
    print(" -", name)