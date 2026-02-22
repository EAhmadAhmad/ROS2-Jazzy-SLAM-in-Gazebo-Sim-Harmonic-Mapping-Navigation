# ROS2 SLAM Project: TurtleBot3 House Mapping

## Project Overview
This project implements Simultaneous Localization and Mapping (SLAM) using TurtleBot3 in a custom house world with ROS2 Jazzy and Gazebo Harmonic. The robot explores the environment, builds a map using Cartographer SLAM, and enables navigation capabilities.

**Author:** Ahmad Ahmad  
**ID:** 475774  
**Group:** R4137c  
**Course:** Robot Programming (Fall 2025)  
**Instructor:** Kirilll Artemov

## Features
- TurtleBot3 robot model (waffle_pi)
- Custom house world in Gazebo
- SLAM implementation using Google Cartographer
- Map generation and saving
- Navigation stack integration
- Visualization in RViz

## Technologies Used
- ROS2 Jazzy
- Gazebo Harmonic
- Google Cartographer SLAM
- Navigation2 (Nav2)
- TurtleBot3 packages
- robot_localization

## Project Structure
.
├── build/ # Build files (ignored by git)
├── install/ # Installation files (ignored by git)
├── log/ # Log files (ignored by git)
├── maps/ # Saved maps
│ ├── my_map.pgm # Generated occupancy grid
│ ├── my_map.yaml # Map metadata
│ └── my_house.rviz # RViz configuration
├── src/ # Source code
│ ├── turtlebot3_fake_node/
│ ├── turtlebot3_gazebo/
│ └── turtlebot3_simulations/
├── Dockerfile # Container configuration
├── docker-entrypoint.sh # Docker entrypoint script
└── README.md # This file


## Prerequisites
- Docker (recommended) or
- Ubuntu 24.04 with ROS2 Jazzy and Gazebo Harmonic

## Installation & Launch

### Using Docker (Recommended)
1. **Build the Docker image:**
   \`\`\`bash
   docker build -t turtlebot3-slam .
   \`\`\`

2. **Run the container with GUI support:**
   \`\`\`bash
   xhost +local:docker
   docker run -it --rm \\
     --env="DISPLAY" \\
     --env="QT_X11_NO_MITSHM=1" \\
     --env="TURTLEBOT3_MODEL=waffle_pi" \\
     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \\
     --volume="$(pwd)/maps:/workspace/maps" \\
     --name turtlebot3_slam \\
     turtlebot3-slam
   \`\`\`

3. **In the container, launch the house world:**
   \`\`\`bash
   # Launch Gazebo with house world
   ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
   
   # In another terminal (attach to container)
   docker exec -it turtlebot3_slam bash
   
   # Launch Cartographer SLAM
   ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
   
   # Control the robot
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   \`\`\`

## Saving and Using Maps

### Save Cartographer Map
\`\`\`bash
# Save the pose graph
ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05 -map_filename my_map
ros2 run nav2_map_server map_saver_cli -f my_map
\`\`\`

### Load Existing Map for Navigation
\`\`\`bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=my_map.yaml
\`\`\`

## Results

### Generated Map
![Generated Map](maps/my_map.png)
*Occupancy grid map generated using Cartographer SLAM*

### House World
![House World](maps/my_house.png)  
*Custom house environment in Gazebo*

## Code Quality

### Principles Applied
- **SOLID:** Modular nodes with single responsibilities
- **DRY:** Reusable launch files and configurations
- **KISS:** Simple, maintainable structure
- **YAGNI:** Only implemented required features

### Design Patterns
- **Factory Pattern:** Robot model configuration
- **Observer Pattern:** Sensor data subscription
- **Strategy Pattern:** Pluggable SLAM algorithms

## Personal Contribution
- Configured Cartographer SLAM for TurtleBot3
- Created custom house world in Gazebo
- Integrated multiple TurtleBot3 packages
- Set up Docker containerization
- Managed build configurations and debugging
- Generated and saved maps successfully
- Documented the entire process

## License
MIT License

## Acknowledgments
- Instructor: Kirilll Artemov
- ITMO University
- TurtleBot3 Community
- ROS2 and Cartographer teams
