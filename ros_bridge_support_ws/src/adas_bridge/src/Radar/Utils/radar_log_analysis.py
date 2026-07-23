import numpy as np
import sys
from datetime import datetime

def parse_radar_log(filename, mode='final'):
    """
    Parse radar log file with different formats based on mode.
    
    FINAL mode format:
    <frame_num>=========================[<timestamp_ns>]
    <cluster_index>____________________________
    <x> <y> <z> <closest_x> <closest_y> <closest_z> <point_velocity>
    
    SEGMENTED mode format:
    <frame_num>=========================[<timestamp_ns>]
    <cluster_id>____________________________
    <x> <y> <z> <closest_x> <closest_y> <closest_z> <point_velocity> <cluster_id>
    
    Args:
        filename: Path to log file
        mode: 'final' or 'segmented'
    
    Returns:
        List of frames with parsed data
    """
    frames = []
    current_frame = None
    current_cluster = None

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            
            if not line:  # Skip empty lines
                continue

            # Frame header: "<frame_num>=========================[<timestamp_ns>]"
            if '=========================' in line:
                # Save previous frame if exists
                if current_frame is not None:
                    frames.append(current_frame)

                # Parse frame number and timestamp
                parts = line.split('=========================')
                try:
                    frame_num = int(parts[0])
                except (ValueError, IndexError):
                    frame_num = None
                
                # Extract timestamp from brackets
                try:
                    timestamp_str = parts[1].strip()
                    if timestamp_str.startswith('[') and timestamp_str.endswith(']'):
                        timestamp = int(timestamp_str[1:-1])
                    else:
                        timestamp = None
                except (ValueError, IndexError):
                    timestamp = None

                current_frame = {
                    'frame_num': frame_num,
                    'timestamp_ns': timestamp,
                    'clusters': []
                }
                current_cluster = None

            # Cluster header: "<cluster_id>____________________________"
            elif '____________________________' in line:
                parts = line.split('____________________________')
                try:
                    cluster_id = int(parts[0])
                except (ValueError, IndexError):
                    cluster_id = None

                current_cluster = {
                    'cluster_id': cluster_id,
                    'points': []
                }
                
                if current_frame is not None:
                    current_frame['clusters'].append(current_cluster)

            # Point data
            else:
                try:
                    data = list(map(float, line.split()))
                    
                    if mode == 'final':
                        # Format: x y z closest_x closest_y closest_z point_velocity
                        if len(data) == 7:
                            point = {
                                'x': data[0],
                                'y': data[1],
                                'z': data[2],
                                'closest_x': data[3],
                                'closest_y': data[4],
                                'closest_z': data[5],
                                'velocity': data[6]
                            }
                            if current_cluster is not None:
                                current_cluster['points'].append(point)
                    
                    elif mode == 'segmented':
                        # Format: x y z closest_x closest_y closest_z point_velocity cluster_id
                        if len(data) == 8:
                            point = {
                                'x': data[0],
                                'y': data[1],
                                'z': data[2],
                                'closest_x': data[3],
                                'closest_y': data[4],
                                'closest_z': data[5],
                                'velocity': data[6],
                                'point_cluster_id': int(data[7])
                            }
                            if current_cluster is not None:
                                current_cluster['points'].append(point)
                    
                except (ValueError, IndexError):
                    # Skip malformed lines
                    pass

    # Don't forget the last frame
    if current_frame is not None:
        frames.append(current_frame)

    return frames


def print_frame_summary(frames, mode='final'):
    """Print summary statistics of parsed frames"""
    print(f"\n{'='*60}")
    print(f"Mode: {mode.upper()}")
    print(f"Total frames parsed: {len(frames)}")
    print(f"{'='*60}")
    
    if len(frames) == 0:
        print("No frames found in log file!")
        return
    
    # First frame info
    first_frame = frames[0]
    print(f"\nFirst frame (#{first_frame['frame_num']}):")
    print(f"  Timestamp: {first_frame['timestamp_ns']} ns")
    if first_frame['timestamp_ns']:
        dt = datetime.fromtimestamp(first_frame['timestamp_ns'] / 1e9)
        print(f"  Date/Time: {dt.strftime('%Y-%m-%d %H:%M:%S.%f')}")
    print(f"  Clusters: {len(first_frame['clusters'])}")
    
    # Last frame info
    last_frame = frames[-1]
    print(f"\nLast frame (#{last_frame['frame_num']}):")
    print(f"  Timestamp: {last_frame['timestamp_ns']} ns")
    if last_frame['timestamp_ns']:
        dt = datetime.fromtimestamp(last_frame['timestamp_ns'] / 1e9)
        print(f"  Date/Time: {dt.strftime('%Y-%m-%d %H:%M:%S.%f')}")
    print(f"  Clusters: {len(last_frame['clusters'])}")
    
    # Overall statistics
    total_clusters = sum(len(frame['clusters']) for frame in frames)
    total_points = sum(
        len(cluster['points']) 
        for frame in frames 
        for cluster in frame['clusters']
    )
    
    print(f"\nOverall statistics:")
    print(f"  Total clusters: {total_clusters}")
    print(f"  Total points: {total_points}")
    print(f"  Avg clusters/frame: {total_clusters/len(frames):.2f}")
    print(f"  Avg points/cluster: {total_points/total_clusters:.2f}" if total_clusters > 0 else "  Avg points/cluster: N/A")
    
    # Example cluster detail (first cluster of first frame)
    if len(first_frame['clusters']) > 0:
        first_cluster = first_frame['clusters'][0]
        print(f"\nFirst cluster (ID={first_cluster['cluster_id']}) details:")
        print(f"  Number of points: {len(first_cluster['points'])}")
        
        if len(first_cluster['points']) > 0:
            first_point = first_cluster['points'][0]
            print(f"  First point:")
            print(f"    Position: ({first_point['x']:.3f}, {first_point['y']:.3f}, {first_point['z']:.3f})")
            print(f"    Velocity: {first_point['velocity']:.3f} m/s")
            print(f"    Closest:  ({first_point['closest_x']:.3f}, {first_point['closest_y']:.3f}, {first_point['closest_z']:.3f})")
            
            if mode == 'segmented' and 'point_cluster_id' in first_point:
                print(f"    Cluster ID (from point): {first_point['point_cluster_id']}")
            
            # Calculate point cloud statistics for this cluster
            positions = np.array([[p['x'], p['y'], p['z']] for p in first_cluster['points']])
            velocities = np.array([p['velocity'] for p in first_cluster['points']])
            
            centroid = positions.mean(axis=0)
            print(f"  Cluster centroid: ({centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f})")
            print(f"  Cluster extent:")
            print(f"    X: [{positions[:,0].min():.3f}, {positions[:,0].max():.3f}]")
            print(f"    Y: [{positions[:,1].min():.3f}, {positions[:,1].max():.3f}]")
            print(f"    Z: [{positions[:,2].min():.3f}, {positions[:,2].max():.3f}]")
            print(f"  Velocity statistics:")
            print(f"    Mean: {velocities.mean():.3f} m/s")
            print(f"    Std:  {velocities.std():.3f} m/s")
            print(f"    Range: [{velocities.min():.3f}, {velocities.max():.3f}] m/s")
    
    print(f"{'='*60}\n")


def export_to_numpy(frames, mode='final', output_prefix="radar_data"):
    """
    Export parsed data to numpy arrays for further processing
    
    For FINAL mode:
    Saves: <output_prefix>_final_points.npy
    Columns: [frame_num, timestamp_ns, cluster_index, x, y, z, closest_x, closest_y, closest_z, velocity]
    
    For SEGMENTED mode:
    Saves: <output_prefix>_segmented_points.npy
    Columns: [frame_num, timestamp_ns, cluster_id, x, y, z, closest_x, closest_y, closest_z, velocity, point_cluster_id]
    """
    # Flatten all points with metadata
    all_points = []
    
    for frame in frames:
        for cluster in frame['clusters']:
            for point in cluster['points']:
                row = [
                    frame['frame_num'],
                    frame['timestamp_ns'],
                    cluster['cluster_id'],
                    point['x'], point['y'], point['z'],
                    point['closest_x'], point['closest_y'], point['closest_z'],
                    point['velocity']
                ]
                
                # Add point_cluster_id for segmented mode
                if mode == 'segmented' and 'point_cluster_id' in point:
                    row.append(point['point_cluster_id'])
                
                all_points.append(row)
    
    points_array = np.array(all_points)
    
    output_file = f"{output_prefix}_{mode}_points.npy"
    np.save(output_file, points_array)
    
    print(f"\nExported {len(all_points)} points to {output_file}")
    print(f"Array shape: {points_array.shape}")
    
    if mode == 'final':
        print(f"Columns: [frame_num, timestamp_ns, cluster_index, x, y, z, closest_x, closest_y, closest_z, velocity]")
    elif mode == 'segmented':
        print(f"Columns: [frame_num, timestamp_ns, cluster_id, x, y, z, closest_x, closest_y, closest_z, velocity, point_cluster_id]")
    
    return points_array


def validate_data_format(frames, mode='final'):
    """
    Validate that data matches expected format
    """
    if len(frames) == 0:
        print("\nWarning: No frames to validate!")
        return False
    
    print(f"\n{'='*60}")
    print(f"Validating data format for mode: {mode.upper()}")
    print(f"{'='*60}")
    
    total_points = 0
    format_errors = 0
    
    for frame in frames:
        for cluster in frame['clusters']:
            for point in cluster['points']:
                total_points += 1
                
                # Check required fields
                required_fields = ['x', 'y', 'z', 'closest_x', 'closest_y', 'closest_z', 'velocity']
                if mode == 'segmented':
                    required_fields.append('point_cluster_id')
                
                missing_fields = [field for field in required_fields if field not in point]
                if missing_fields:
                    format_errors += 1
                    if format_errors <= 5:  # Only print first 5 errors
                        print(f"  Error: Point missing fields: {missing_fields}")
    
    if format_errors == 0:
        print(f"✓ All {total_points} points have correct format")
        return True
    else:
        print(f"✗ Found {format_errors} format errors in {total_points} points")
        return False


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python parse_radar.py <input_file> [--mode final|segmented] [--export]")
        print("\nOptions:")
        print("  --mode final      Parse as FINAL objects data (default)")
        print("  --mode segmented  Parse as SEGMENTED clusters data")
        print("  --export          Export parsed data to numpy arrays")
        print("\nExamples:")
        print("  python parse_radar.py data.txt")
        print("  python parse_radar.py data.txt --mode segmented --export")
        sys.exit(1)

    input_file = sys.argv[1]
    
    # Parse arguments
    mode = 'final'  # default
    export_data = '--export' in sys.argv
    
    for i, arg in enumerate(sys.argv):
        if arg == '--mode' and i + 1 < len(sys.argv):
            mode = sys.argv[i + 1].lower()
            if mode not in ['final', 'segmented']:
                print(f"Error: Invalid mode '{mode}'. Must be 'final' or 'segmented'")
                sys.exit(1)
    
    print(f"Parsing radar log file: {input_file}")
    print(f"Mode: {mode.upper()}")
    print(f"{'='*60}")
    
    try:
        frames = parse_radar_log(input_file, mode=mode)
        print_frame_summary(frames, mode=mode)
        
        # Validate data format
        validate_data_format(frames, mode=mode)
        
        if export_data:
            export_to_numpy(frames, mode=mode)
        
    except FileNotFoundError:
        print(f"Error: File '{input_file}' not found!")
        sys.exit(1)
    except Exception as e:
        print(f"Error parsing file: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)