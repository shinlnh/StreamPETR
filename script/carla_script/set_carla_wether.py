import carla
import time

# Dictionary mapping WeatherId integers to CARLA weather parameter presets
WEATHER_PRESETS = {
    0: carla.WeatherParameters.Default,
    1: carla.WeatherParameters.ClearNoon,
    2: carla.WeatherParameters.CloudyNoon,
    3: carla.WeatherParameters.WetNoon,
    4: carla.WeatherParameters.WetCloudyNoon,
    5: carla.WeatherParameters.MidRainyNoon,
    6: carla.WeatherParameters.HardRainNoon,
    7: carla.WeatherParameters.SoftRainNoon,
    8: carla.WeatherParameters.ClearSunset,
    9: carla.WeatherParameters.CloudySunset,
    10: carla.WeatherParameters.WetSunset,
    11: carla.WeatherParameters.WetCloudySunset,
    12: carla.WeatherParameters.MidRainSunset,
    13: carla.WeatherParameters.HardRainSunset,
    14: carla.WeatherParameters.SoftRainSunset,
}

def set_weather(client: carla.Client, weather_id: int):
    """
    Set the weather in the CARLA simulation based on the provided weather_id.

    Args:
        client (carla.Client): The CARLA client connected to the server.
        weather_id (int): Integer ID representing a specific weather preset.
    """
    try:
        # Get the current world from the CARLA server
        world = client.get_world()

        # Check if the given weather ID exists in the mapping
        if weather_id in WEATHER_PRESETS:
            world.set_weather(WEATHER_PRESETS[weather_id])
            print(f"Weather successfully set to ID {weather_id}")
        else:
            print("Invalid WeatherId. Please choose a value between 0 and 14.")

    except Exception as e:
        print(f"Error while setting weather: {e}")

def main():
    """
    Main entry point of the script. Connects to the CARLA server and sets the desired weather.
    """
    # Connect to the CARLA server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)  # Timeout in seconds for commands

    # Set this variable to select the desired weather condition
    desired_weather_id = 6  # Example: SoftRainSunset

    # Set the weather in the simulation
    set_weather(client, desired_weather_id)

if __name__ == '__main__':
    main()
