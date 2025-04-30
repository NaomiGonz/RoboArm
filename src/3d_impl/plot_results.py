import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import csv
from collections import defaultdict
import argparse
import os # Added for os.path.basename

def parse_3d_rrt_data(filepath):
    """
    Parses the 3D RRT data (Obstacles, Cartesian Path) from the specified CSV file.
    """
    data = {
        'obstacles': [],
        # Dictionary to store path nodes: {path_node_index: [[x0,y0,z0], ..., [x4,y4,z4]]}
        'path_nodes_all_joints': defaultdict(lambda: [[0.0,0.0,0.0] for _ in range(5)])
        # Removed unused sections from the example style
    }
    current_section = None
    header = None # Keep track of header row within sections

    try:
        print(f"Reading data from: {filepath}")
        with open(filepath, 'r', newline='') as f:
            reader = csv.reader(f)

            for row in reader:
                # Skip empty row
                if not row:
                    continue

                # In new data type section
                if row[0].startswith('# Section:'):
                    current_section = row[0].split(':')[-1].strip()
                    header = None # Reset header for new section
                    # print(f"Entering Section: {current_section}") # Optional debug
                    continue

                # Read the header in the new section
                if current_section and not header:
                    header = [h.strip() for h in row]
                    # print(f"Header for {current_section}: {header}") # Optional debug
                    continue

                # Skip lines before the first section or without header
                if not current_section or not header:
                    continue

                # Process data rows based on section
                if current_section == 'Obstacles':
                    if len(row) >= 4: # center_x, center_y, center_z, radius
                        try:
                             center_x, center_y, center_z, radius = map(float, row[:4])
                             data['obstacles'].append({'center': [center_x, center_y, center_z], 'radius': radius})
                        except ValueError:
                            print(f"Skipping invalid obstacle row: {row}")
                    else:
                        print(f"Skipping malformed obstacle row: {row}")


                elif current_section == 'Goal_Path_Cartesian_All_Joints':
                     if len(row) >= 5: # path_node_index, joint_frame_index, x, y, z
                        try:
                            path_node_idx, joint_frame_idx, x, y, z = map(float, row[:5])
                            path_node_idx = int(path_node_idx)
                            joint_frame_idx = int(joint_frame_idx)
                            if 0 <= joint_frame_idx < 5:
                                # The defaultdict initializes the list, just assign the coordinate
                                data['path_nodes_all_joints'][path_node_idx][joint_frame_idx] = [x, y, z]
                            else:
                                print(f"Skipping row with invalid joint_frame_index: {row}")
                        except ValueError:
                            print(f"Skipping invalid path data row: {row}")
                     else:
                         print(f"Skipping malformed path data row: {row}")


    except FileNotFoundError:
        print(f"Error: File not found at {filepath}")
        return None
    except Exception as e:
        print(f"Error parsing file {filepath}: {e}")
        return None

    # Filter out incomplete path nodes before returning
    # (Nodes where not all 5 joint positions were read successfully)
    keys_to_delete = []
    for key, joints in data['path_nodes_all_joints'].items():
         # Check if any default [0,0,0] remains or if structure is wrong
        if any(coord == [0.0, 0.0, 0.0] and key != 0 for coord in joints) or len(joints) != 5: # Allow [0,0,0] for path_idx 0 if that's valid start
            # A more robust check might be needed if [0,0,0] is a valid coordinate
            # Maybe check against the initial defaultdict value if that's more unique
             if all(coord == [0.0, 0.0, 0.0] for coord in joints): # Check if it's still the default init value
                print(f"Warning: Incomplete data detected for path_node_index {key}. Removing entry.")
                keys_to_delete.append(key)


    for key in keys_to_delete:
        del data['path_nodes_all_joints'][key]


    return data

def plot_3d_rrt(data, filename):
    """Plots the 3D RRT data using Matplotlib."""
    if not data:
        print("No data provided for plotting.")
        return

    path_nodes_all_joints = data.get('path_nodes_all_joints', {})
    obstacles = data.get('obstacles', [])

    # --- Plotting Setup ---
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # --- Plot Obstacles ---
    print(f"Plotting {len(obstacles)} obstacles...")
    if obstacles:
        for obs in obstacles:
            center = obs['center']
            radius = obs['radius']

            # Create sphere surface points
            u = np.linspace(0, 2 * np.pi, 50) # Reduced points for performance
            v = np.linspace(0, np.pi, 50)     # Reduced points for performance
            x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
            y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
            z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]
            ax.plot_surface(x, y, z, color='r', alpha=0.3, rstride=5, cstride=5, linewidth=0)
    else:
        print("No obstacle data found to plot.")


    # --- Plot Robot Path Configurations ---
    if not path_nodes_all_joints:
        print("No path data found in 'path_nodes_all_joints'.")
        last_path_node_index = -1
        path_indices = []
    else:
        path_indices = sorted(path_nodes_all_joints.keys())
        if not path_indices:
             last_path_node_index = -1
             print("Path data structure exists but contains no valid path indices.")
        else:
            last_path_node_index = max(path_indices)

    print(f"Plotting path nodes from index {path_indices[0] if path_indices else 'N/A'} to {last_path_node_index}...")

    if not path_indices:
        print("No valid path indices found to plot.")
    else:
        for path_idx in path_indices:
            joint_coords_list = path_nodes_all_joints[path_idx]
            # Data validation happened during parsing, assume structure is correct here
            joint_coords = np.array(joint_coords_list) # Convert list of lists to numpy array (5x3)

            # Determine color
            if path_idx == 0 or path_idx == last_path_node_index:
                color = 'blue'
                marker_size = 6
                line_width = 2.0
            else:
                color = 'black'
                marker_size = 4
                line_width = 1.0

            # Plot joint points
            ax.scatter(joint_coords[:, 0], joint_coords[:, 1], joint_coords[:, 2], color=color, marker='o', s=marker_size**2)

            # Plot connecting lines (arm segments)
            for j in range(len(joint_coords) - 1):
                p1 = joint_coords[j]
                p2 = joint_coords[j+1]
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color=color, linestyle='-', linewidth=line_width)


    # --- Plot Configuration ---
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'RRT* Path and Obstacles\n(File: {os.path.basename(filename)})')

    # Set axis limits based on data, giving some padding
    all_coords = [p for node in path_nodes_all_joints.values() for p in node]
    all_obs_coords = [o['center'] for o in obstacles]
    all_points = all_coords + all_obs_coords

    if all_points: # Only set limits if there is data
        all_points = np.array(all_points)
        min_coords = all_points.min(axis=0)
        max_coords = all_points.max(axis=0)

        if obstacles:
            radii = np.array([o['radius'] for o in obstacles])
            min_coords = np.min(np.vstack((min_coords, np.array([o['center'] for o in obstacles]) - radii[:, np.newaxis])), axis=0)
            max_coords = np.max(np.vstack((max_coords, np.array([o['center'] for o in obstacles]) + radii[:, np.newaxis])), axis=0)

        center = (min_coords + max_coords) / 2
        ranges = max_coords - min_coords
        max_range = max(ranges) * 0.6
        min_display_range = 0.1
        max_range = max(max_range, min_display_range / 2.0)

        ax.set_xlim(center[0] - max_range, center[0] + max_range)
        ax.set_ylim(center[1] - max_range, center[1] + max_range)
        ax.set_zlim(center[2] - max_range, center[2] + max_range)
    else:
        print("Warning: No plot data (path or obstacles) found. Using default axis limits.")
        ax.set_xlim(-0.5, 0.5)
        ax.set_ylim(-0.5, 0.5)
        ax.set_zlim(0, 0.6)

    # Make axes equal scale attempt (using limits)
    # ax.set_aspect('equal', adjustable='box') # Can cause issues, rely on calculated limits

    plt.tight_layout()
    plt.show()
    print("Plotting complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot 3D RRT* results from a CSV data file.")
    parser.add_argument("filepath", help="Path to the CSV data file generated by the C++ RRT program.")
    args = parser.parse_args()

    rrt_data = parse_3d_rrt_data(args.filepath)
    if rrt_data:
        plot_3d_rrt(rrt_data, args.filepath)
