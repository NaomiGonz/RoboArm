import csv
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import argparse
import os
import numpy as np 

def parse_rrt_data(filepath):
    """Parses the RRT data from the specified CSV file."""
    data = {
        'params': {},
        'start': None,
        'goal': None,
        'obstacles': [],
        'all_edges': [],
        'goal_path': [],
        'simplified_path': []
    }
    current_section = None

    try:
        with open(filepath, 'r') as f:
            reader = csv.reader(f)
            header = None 

            # Parse each row in csv file
            for row in reader:

                # Skip empty row
                if not row: 
                    continue

                # In new data type section 
                if row[0].startswith('# Section:'):
                    current_section = row[0].split(':')[-1].strip()
                    header = None # Reset header for new section
                    continue

                # Read all the new headers in the new data type section
                if current_section and not header:
                     header = [h.strip() for h in row] 
                     continue 

                # Skip lines before the first section or without header
                if not current_section or not header:
                    continue

                # Process data rows based on section
                if current_section == 'Parameters':
                    param, value = row[0].strip(), row[1].strip()
                    try:
                        # Attempt to convert numerical values
                        if '.' in value or 'e' in value.lower():
                           data['params'][param] = float(value)
                        elif value.lower() == 'inf':
                            data['params'][param] = np.inf
                        else:
                           data['params'][param] = int(value)
                    except ValueError:
                           # Keep the data as a string if conversion fails
                           data['params'][param] = value
                elif current_section == 'Start':
                     data['start'] = (float(row[0]), float(row[1]))
                elif current_section == 'Goal':
                     data['goal'] = (float(row[0]), float(row[1]))
                elif current_section == 'Obstacles':
                     # center_x, center_y, radius
                     data['obstacles'].append((float(row[0]), float(row[1]), float(row[2])))
                elif current_section == 'All_Edges':
                     # x1, y1, x2, y2
                     p1 = (float(row[0]), float(row[1]))
                     p2 = (float(row[2]), float(row[3]))
                     data['all_edges'].append((p1, p2))
                elif current_section == 'Goal_Path':
                     p1 = (float(row[0]), float(row[1]))
                     p2 = (float(row[2]), float(row[3]))
                     data['goal_path'].append((p1, p2))
                elif current_section == 'Simplified_Path':
                     p1 = (float(row[0]), float(row[1]))
                     p2 = (float(row[2]), float(row[3]))
                     data['simplified_path'].append((p1, p2))

    except FileNotFoundError:
        print(f"Error: File not found at {filepath}")
        return None
    except Exception as e:
        print(f"Error parsing file {filepath}: {e}")
        return None

    return data

def plot_rrt(data, filename):
    """Plots the RRT data using Matplotlib."""
    if not data:
        return

    fig, ax = plt.subplots()

    # Plot obstacles
    for obs in data['obstacles']:
        ax.add_patch(plt.Circle((obs[0], obs[1]), radius=obs[2], fc='y', ec='k', alpha=0.6))

    # Plot edges
    if data['all_edges']:
        lc_edges = LineCollection(data['all_edges'], colors=[(0, 0, 1, 0.3)], linewidths=0.5, label='RRT Edges')
        ax.add_collection(lc_edges)
    else:
        print("Warning: No 'All_Edges' data found.")


    # Plot goal path
    if data['goal_path']:
        lc_goal = LineCollection(data['goal_path'], colors=[(1, 0, 0, 0.8)], linewidths=1.5, label='Goal Path')
        ax.add_collection(lc_goal)
    else:
        print("Info: No 'Goal_Path' data found (goal might not be reachable).")

    # Plot simplified path
    if data['simplified_path']:
        lc_simple = LineCollection(data['simplified_path'], colors=[(0, 1, 0, 0.8)], linewidths=2.0, linestyle='--', label='Simplified Path')
        ax.add_collection(lc_simple)
    else:
        print("Info: No 'Simplified_Path' data found (goal might not be reachable).")


    # Plot start and goal points
    if data['start']:
        ax.scatter(data['start'][0], data['start'][1], c='r', s=50, label='Start', zorder=5) 
    if data['goal']:
        ax.scatter(data['goal'][0], data['goal'][1], c='lime', s=50, label='Goal', zorder=5)

    # Set plot limits and aspect ratio
    xy_min = data['params'].get('xy_min', -2.0) # Default if not in file
    xy_max = data['params'].get('xy_max', 2.0) # Default if not in file
    ax.set_xlim(xy_min, xy_max)
    ax.set_ylim(xy_min, xy_max)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    # Create title from parameters
    params_str = ", ".join([f"{k}={v}" for k, v in data['params'].items() if k not in ['xy_min', 'xy_max','goal_cost']])
    goal_cost = data['params'].get('goal_cost', np.inf)
    cost_str = f"{goal_cost:.2f}" if not np.isinf(goal_cost) else "inf"
    plot_title = f"RRT* Results ({os.path.basename(filename)})\nParams: {params_str}, Cost: {cost_str}"
    ax.set_title(plot_title, wrap=True)

    # Add legend
    ax.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot RRT* results from a CSV data file.")
    parser.add_argument("filepath", help="Path to the CSV data file generated by the C++ RRT program.")
    args = parser.parse_args()

    rrt_data = parse_rrt_data(args.filepath)
    if rrt_data:
        plot_rrt(rrt_data, args.filepath)