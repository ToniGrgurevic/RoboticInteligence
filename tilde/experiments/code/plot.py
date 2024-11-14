import os
import csv
import matplotlib.pyplot as plt


def read_csv(file_path):
    data = []
    with open(file_path, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            data.append({
                'timestamp': float(row['timestamp']),
                'x': float(row['x']),
                'y': float(row['y'])
            })
    return data


def plot_paths(experiment_data):
    plt.figure(figsize=(10, 8))

    for experiment_name, paths in experiment_data.items():
        for robot_name, path in paths.items():
            x_coords = [point['x'] for point in path]
            y_coords = [point['y'] for point in path]
            plt.plot(x_coords, y_coords, label=f"{experiment_name} - {robot_name}")

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Robot Paths Around Tilda')
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_individual_experiments(experiment_data):
    for experiment_name, paths in experiment_data.items():
        plt.figure(figsize=(10, 8))
        for robot_name, path in paths.items():
            x_coords = [point['x'] for point in path]
            y_coords = [point['y'] for point in path]
            plt.plot(x_coords, y_coords, label=f"{experiment_name} - {robot_name}")

        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title(f'Robot Paths for {experiment_name}')
        plt.legend()
        plt.grid(True)
        plt.savefig(f'{experiment_name}_paths.png')
        plt.close()


def plot_experiments_grid(experiment_data):

    filtered_experiments = {k: v for k, v in experiment_data.items() if k in ['experiment1', 'experiment2',
                                                                              'experiment3', 'experiment4']}


    # Create a 2x2 grid of subplots
    fig, axes = plt.subplots(2, 2, figsize=(15, 15))

    # Define consistent colors for each robot
    robot_colors = {
        'robot1': 'blue',
        'robot2': 'red'
    }

    # Sort experiment names to ensure consistent order
    sorted_experiments = sorted(filtered_experiments.keys())

    # Plot each experiment in its respective subplot
    for idx, experiment_name in enumerate(sorted_experiments):
        # Calculate row and column for 2x2 grid
        row = idx // 2
        col = idx % 2

        # Get the current subplot
        ax = axes[row, col]

        for robot_name, path in filtered_experiments[experiment_name].items():
            x_coords = [point['x'] for point in path]
            y_coords = [point['y'] for point in path]
            ax.plot(x_coords, y_coords,
                    color=robot_colors[robot_name],
                    label=robot_name,
                    linewidth=2)

        # Configure subplot
        ax.set_xlabel('X Coordinate')
        ax.set_ylabel('Y Coordinate')
        ax.set_title(f'Experiment {idx + 1}')
        ax.grid(True)
        ax.legend()

        # Set consistent axis limits across all plots
        ax.set_xlim(-24, 12)
        ax.set_ylim(-10, 8)

    # Adjust layout to prevent overlap
    plt.tight_layout()

    # Save the figure
    plt.savefig('all_experiments_grid.png', dpi=300, bbox_inches='tight')
    plt.close()

def plot_combined_robot2(filtered_experiments):
    plt.figure(figsize=(10, 8))


    for experiment_name, paths in filtered_experiments.items():
        if 'robot2' in paths:
            x_coords = [point['x'] for point in paths['robot2']]
            y_coords = [point['y'] for point in paths['robot2']]
            plt.plot(x_coords, y_coords, label=f"{experiment_name} - robot2")

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    #plt.title('Diference on robot pathing based on ')
    plt.legend()
    plt.grid(True)
    plt.savefig('combined_robot2_paths.png')
    plt.close()

def plot_compare(filtered_experiments):
    plt.figure(figsize=(10, 8))

    for experiment_name, paths in filtered_experiments.items():
        if 'robot2' in paths:
            x_coords = [point['x'] for point in paths['robot2']]
            y_coords = [point['y'] for point in paths['robot2']]
            plt.plot(x_coords, y_coords, label=f"{experiment_name} - robot2")

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Fast robot Paths Across Experiments')
    plt.legend()
    plt.grid(True)
    plt.savefig('combined_robot2_paths.png')
    plt.close()


def main():
    experiment_data = {}
    base_dir = 'C:/Users/Toni/Projects/Porto/Inteligent_robotics/ri/tilde/experiments'

    for experiment_dir in os.listdir(base_dir):
        experiment_path = os.path.join(base_dir, experiment_dir)
        if os.path.isdir(experiment_path):
            experiment_name = experiment_dir.split('_')[0]
            experiment_data[experiment_name] = {}

            for file_name in os.listdir(experiment_path):
                if file_name.endswith('.csv'):
                    robot_name = file_name[:-4]
                    file_path = os.path.join(experiment_path, file_name)
                    experiment_data[experiment_name][robot_name] = read_csv(file_path)

    #plot_paths(experiment_data)
    #plot_individual_experiments(experiment_data)

   # filtered_experiments = {"fast robot with sensor rate 10Hz": experiment_data['experiment3'],
     #                       "fast robot with sensor rate 100Hz": experiment_data['experiment5']}

    filtered_experiments = {k: v for k, v in experiment_data.items() if k in ['experiment1', 'experiment2',
                                                                              'experiment3', 'experiment4']}
    plot_combined_robot2(filtered_experiments)


    plot_experiments_grid(experiment_data)
if __name__ == "__main__":
    main()