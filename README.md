# usv_visualizations

A ROS 2‑based visualization framework for the autonomous USV project.  
Provides tools for visualizing satellite/RViz overlays, robot localization, and related visualization assets to support development, debugging, and demonstration.

---

## Table of Contents

1. [Features](#features)  
2. [Repository Structure](#repository-structure)  
3. [Prerequisites](#prerequisites)  
4. [Installation / Setup](#installation--setup)  
5. [Usage](#usage)  
6. [Development](#development)  
7. [Contributing](#contributing)  
8. [License](#license)  

---

## Features

- Satellite imagery display in RViz (for context / map)  
- Robot localization support via `robot_localization` package  
- Scripts/tools for visualization (in `usviz/scripts`)  
- Modular architecture: separate ROS nodes & packages (e.g. `robot_localization_pkg`, etc.)  

---

## Repository Structure

```text
usv_visualizations/
├── robot/                        ← code related to robot-specific visualization nodes
├── robot_localization_pkg/      ← localization_pkg for ROS 2 / robot localization tools
├── rviz_satellite/               ← RViz plugin or configuration for satellite imagery
├── usviz/scripts/                ← helper scripts (launch, data prep, etc.)
├── .gitmodules                   ← sub‑module(s), if any
└── README.md                     ← this file
```

---

## Prerequisites

These should be installed / set up before using:

- ROS 2 (Humble)  
- `robot_localization` package for ROS 2  
- RViz2  

---

## Installation / Setup

Here’s how to get things running:

1. **Clone the repository**

   ```bash
   git clone https://github.com/Anaskherro/usv_visualizations.git
   ```

2. **Initialize submodules**

   ```bash
   cd usv_visualizations
   git submodule update --init --recursive
   ```

3. **Install ROS 2 workspace**

   ```bash
   # Assuming you have a ROS‑2 workspace
   cd ~/ros2_ws/src
   ln -s /path/to/usv_visualizations usv_visualizations
   cd ~/ros2_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   colcon build
   ```

## Usage

Run the scripts in `usviz/scripts` in order to visualize the current usv path and odometry, and use the robot package to visualize the usv model in rviz.

---

## Development

If you want to extend this project:

- Add new visualization nodes under `robot/`  
- Update localization package parameters  
- Create new scripts / tools under `usviz/scripts`  
- Maintain consistent code quality: use linting, formatting  

---

## Contributing

Contributions are very welcome! To contribute:

1. Fork the repo  
2. Create a feature branch (`git checkout -b feature/YourFeature`)  
3. Make changes & commit with meaningful messages  
4. Ensure existing tests (if any) pass, and add tests for new functionality  
5. Submit a pull request 

---

## License

MIT Licence.

---

## Contact

For questions or support, contact me at *anas.kherro@um6p.ma*, or open an issue in this repo.
