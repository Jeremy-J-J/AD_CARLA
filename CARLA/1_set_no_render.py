import carla, time
client = carla.Client("localhost", 2000)
client.set_timeout(5.0)
world = client.get_world()

settings = world.get_settings()
settings.no_rendering_mode = True
world.apply_settings(settings)

print(">>> no-rendering ENABLED")
time.sleep(1)