# MR-APF: Multi-Robot Artificial Potential Fields Path Planning

A ROS-based implementation of distributed multi-robot path planning using Artificial Potential Fields (APF) for real-time navigation and obstacle avoidance.

## Overview

This project implements a multi-robot artificial potential fields path planning system that enables multiple robots to navigate autonomously while avoiding obstacles and each other. The system supports both real-time Gazebo simulation and simplified point-robot simulation with RViz visualization.

## Features

- **Distributed Planning**: Each robot runs its own planner while sharing fleet data
- **Real-time Visualization**: Live monitoring through RViz
- **Dual Simulation Modes**: Gazebo for realistic physics or simplified point-robot simulation
- **Obstacle Avoidance**: Dynamic obstacle and inter-robot collision avoidance
- **Fleet Coordination**: Centralized fleet data management with distributed execution
- **Result Analysis**: Automatic path recording and visualization

## System Architecture

![Design Overview](design/design.png)

The system consists of:
- **Central MRAPF Service**: Coordinates fleet data and robot initialization
- **Robot Planners**: Individual APF-based planners for each robot
- **Fleet Data Handler**: Manages robot positions and states
- **Visualization System**: RViz-based real-time monitoring

## Simulation Modes

### 1. Real-time Gazebo Simulation
- Full physics simulation with TurtleBot3 models
- Realistic sensor data and dynamics
- Complete ROS navigation stack integration

### 2. Simple Point-Robot Simulation
- Lightweight simulation for algorithm testing
- Robots represented as circular agents
- Faster execution for large fleet testing

Both modes provide:
- Real-time visualization in RViz
- Path recording and analysis
- Performance metrics collection

## Sample Environment

![Sample Map](design/sample_map.png)

## Installation

1. Clone the repository into your ROS workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/MortezaHagh/mr_apf.git
```

2. Build the package:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### Quick Start
```bash
# Launch the main APF simulation
roslaunch mr_apf main_apf.launch

# For Gazebo simulation with TurtleBot3
roslaunch mr_apf turtlebot3_walls2.launch
```

### Configuration
- Edit `scripts/parameters.py` for algorithm parameters
- Modify launch files for different robot configurations
- Adjust map files in model_inputs.py

## Project Structure

```
mr_apf/
├── action/          # ROS action definitions
├── design/          # Architecture diagrams and documentation
├── launch/          # ROS launch files
├── maps/            # Environment maps (JSON, PNG, SVG)
├── models/          # Robot and obstacle models
├── msg/             # ROS message definitions
├── results/         # Simulation results and analysis
├── rviz/            # RViz configuration files
├── scripts/         # Core Python implementation
│   ├── apf_planner_*.py    # APF algorithm implementations
│   ├── mrapf_main.py       # Main simulation entry point
│   ├── visualization.py    # RViz visualization
│   └── ...
└── srv/             # ROS service definitions
```

## Key Components

- **APF Planners**: Core artificial potential field algorithms
- **Fleet Management**: Centralized coordination system
- **Visualization**: Real-time RViz display system
- **Model Creation**: Environment and robot model generation
- **Results Analysis**: Path analysis and performance metrics

## Results

Results are automatically saved to the `results/` directory, including:
- Robot trajectories and paths
- Performance metrics
- Visualization plots
- JSON data for further analysis

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Citation

If you use this work in your research, please cite:
```bibtex
@misc{mr_apf,
  title={Multi-Robot Artificial Potential Fields Path Planning},
  author={MortezaHagh},
  year={2025},
  url={https://github.com/MortezaHagh/mr_apf}
}
```
