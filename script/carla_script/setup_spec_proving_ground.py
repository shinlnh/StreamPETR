#!/usr/bin/env python

"""
CARLA Map Layer and Environment Objects Manager
This script removes/restores map layers and disables/enables environment objects in CARLA.
It can be run independently alongside other CARLA clients.
"""

import carla
import argparse
import time
import sys


def configure_carla_environment(host='127.0.0.1', port=2000, timeout=10.0, restore=False):
    """
    Configure CARLA environment by unloading/loading map layers and disabling/enabling objects.
    
    Args:
        host: CARLA server host address
        port: CARLA server port
        timeout: Connection timeout in seconds
        restore: If True, restore all layers and objects; if False, remove them
    """
    try:
        # Connect to CARLA server
        print(f"Connecting to CARLA server at {host}:{port}...")
        client = carla.Client(host, port)
        client.set_timeout(timeout)
        
        # Test connection and get server info
        try:
            server_version = client.get_server_version()
            client_version = client.get_client_version()
            print(f"✓ Successfully connected!")
            print(f"  Server version: {server_version}")
            print(f"  Client version: {client_version}")
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            print(f"  Make sure CARLA server is running at {host}:{port}")
            return False
        
        # Get the world
        world = client.get_world()
        map_name = world.get_map().name
        print(f"  Current world: {map_name}")
        
        # Get world settings to verify we can modify it
        settings = world.get_settings()
        print(f"  Synchronous mode: {settings.synchronous_mode}")
        print(f"  Fixed delta seconds: {settings.fixed_delta_seconds}")
        
        # Wait for a tick to ensure world is ready
        print("\nWaiting for world tick...")
        world.wait_for_tick()
        print("✓ World is ready")
        
        # Define map layers
        all_layers = [
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.Ground
        ]
        
        if restore:
            # RESTORE MODE
            print("\n" + "=" * 60)
            print("RESTORING CARLA ENVIRONMENT TO DEFAULT STATE")
            print("=" * 60)
            
            # Load map layers
            print("\n--- Loading Map Layers ---")
            for layer in all_layers:
                try:
                    print(f"  Loading {layer}...", end=" ")
                    world.load_map_layer(layer)
                    print("✓")
                except Exception as e:
                    print(f"✗ Error: {e}")
            
            # Wait for changes to apply
            print("\nWaiting for changes to apply...", end=" ")
            world.wait_for_tick()
            time.sleep(0.5)  # Extra wait to ensure rendering updates
            print("✓")
            
            # Enable traffic lights
            print("\n--- Enabling Traffic Lights ---")
            try:
                print("  Getting traffic light objects...", end=" ")
                traffic_light_objs = world.get_environment_objects(carla.CityObjectLabel.TrafficLight)
                print(f"✓ Found {len(traffic_light_objs)}")
                
                if traffic_light_objs:
                    traffic_light_ids = [tl_obj.id for tl_obj in traffic_light_objs]
                    print(f"  Enabling {len(traffic_light_ids)} traffic lights...", end=" ")
                    world.enable_environment_objects(traffic_light_ids, True)
                    print("✓")
                else:
                    print("  No traffic lights to enable")
            except Exception as e:
                print(f"\n  ✗ Failed to enable traffic lights: {e}")
            
            # Enable traffic signs
            print("\n--- Enabling Traffic Signs ---")
            try:
                print("  Getting traffic sign objects...", end=" ")
                traffic_sign_objs = world.get_environment_objects(carla.CityObjectLabel.TrafficSigns)
                print(f"✓ Found {len(traffic_sign_objs)}")
                
                if traffic_sign_objs:
                    traffic_sign_ids = [ts_obj.id for ts_obj in traffic_sign_objs]
                    print(f"  Enabling {len(traffic_sign_ids)} traffic signs...", end=" ")
                    world.enable_environment_objects(traffic_sign_ids, True)
                    print("✓")
                else:
                    print("  No traffic signs to enable")
            except Exception as e:
                print(f"\n  ✗ Failed to enable traffic signs: {e}")
            
            print("\n✓ Environment restoration completed successfully!")
            
        else:
            # REMOVE MODE (default)
            print("\n" + "=" * 60)
            print("REMOVING MAP LAYERS AND DISABLING OBJECTS")
            print("=" * 60)
            
            # Unload map layers
            print("\n--- Unloading Map Layers ---")
            for layer in all_layers:
                try:
                    print(f"  Unloading {layer}...", end=" ")
                    world.unload_map_layer(layer)
                    print("✓")
                except Exception as e:
                    print(f"✗ Error: {e}")
            
            # Wait for changes to apply
            print("\nWaiting for changes to apply...", end=" ")
            world.wait_for_tick()
            time.sleep(0.5)  # Extra wait to ensure rendering updates
            print("✓")
            
            # Disable traffic lights
            print("\n--- Disabling Traffic Lights ---")
            try:
                print("  Getting traffic light objects...", end=" ")
                traffic_light_objs = world.get_environment_objects(carla.CityObjectLabel.TrafficLight)
                print(f"✓ Found {len(traffic_light_objs)}")
                
                if traffic_light_objs:
                    traffic_light_ids = [tl_obj.id for tl_obj in traffic_light_objs]
                    print(f"  Disabling {len(traffic_light_ids)} traffic lights...", end=" ")
                    world.enable_environment_objects(traffic_light_ids, False)
                    print("✓")
                else:
                    print("  No traffic lights to disable")
            except Exception as e:
                print(f"\n  ✗ Failed to disable traffic lights: {e}")
            
            # Disable traffic signs
            print("\n--- Disabling Traffic Signs ---")
            try:
                print("  Getting traffic sign objects...", end=" ")
                traffic_sign_objs = world.get_environment_objects(carla.CityObjectLabel.TrafficSigns)
                print(f"✓ Found {len(traffic_sign_objs)}")
                
                if traffic_sign_objs:
                    traffic_sign_ids = [ts_obj.id for ts_obj in traffic_sign_objs]
                    print(f"  Disabling {len(traffic_sign_ids)} traffic signs...", end=" ")
                    world.enable_environment_objects(traffic_sign_ids, False)
                    print("✓")
                else:
                    print("  No traffic signs to disable")
            except Exception as e:
                print(f"\n  ✗ Failed to disable traffic signs: {e}")
            
            print("\n✓ Environment configuration completed successfully!")
        
        # Wait for final changes to apply
        print("\nFinalizing changes...", end=" ")
        world.wait_for_tick()
        time.sleep(0.5)
        print("✓")
        
        print("\n" + "=" * 60)
        print("The changes will persist for all clients in this CARLA session.")
        print("If you're viewing in a spectator or other client, you may need to")
        print("reconnect or refresh to see the changes.")
        print("=" * 60)
        return True
        
    except RuntimeError as e:
        print(f"\n✗ Runtime Error: {e}")
        return False
    except Exception as e:
        print(f"\n✗ Unexpected Error: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main function to parse arguments and run the configuration."""
    parser = argparse.ArgumentParser(
        description='Configure CARLA environment by removing/restoring map layers and disabling/enabling objects',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Remove layers and disable objects (default)
  python configure_carla_env.py
  
  # Restore everything back to normal
  python configure_carla_env.py --restore
  
  # Connect to remote CARLA server
  python configure_carla_env.py --host 192.168.1.100 --port 2000
  
  # With increased timeout for slow connections
  python configure_carla_env.py --host remote-server --timeout 30.0
        """)
    
    parser.add_argument(
        '--host',
        default='127.0.0.1',
        help='IP of the CARLA host server (default: 127.0.0.1)')
    
    parser.add_argument(
        '-p', '--port',
        type=int,
        default=2000,
        help='TCP port to listen to (default: 2000)')
    
    parser.add_argument(
        '--timeout',
        type=float,
        default=10.0,
        help='Connection timeout in seconds (default: 10.0)')
    
    parser.add_argument(
        '--restore',
        action='store_true',
        help='Restore all map layers and enable all objects (default: remove/disable)')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("CARLA Environment Configuration Script")
    print("=" * 60)
    print(f"Target server: {args.host}:{args.port}")
    print(f"Mode: {'RESTORE' if args.restore else 'REMOVE'}")
    print(f"Timeout: {args.timeout}s")
    print("=" * 60 + "\n")
    
    success = configure_carla_environment(args.host, args.port, args.timeout, args.restore)
    
    if success:
        print("\n✓ Script completed successfully")
        sys.exit(0)
    else:
        print("\n✗ Script failed")
        sys.exit(1)


if __name__ == '__main__':
    main()