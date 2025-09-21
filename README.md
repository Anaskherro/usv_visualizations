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

- ROS 2 (specify the version: e.g. Humble, Iron, etc.)  
- `robot_localization` package for ROS 2  
- RViz2  
- Dependencies for satellite imagery (e.g. geospatial libraries, image providers)  
- Python (version), plus any Python packages used in `usviz/scripts`  

---

## Installation / Setup

Here’s how to get things running:

1. **Clone the repository**

   ```bash
   git clone https://github.com/Anaskherro/usv_visualizations.git
   ```

2. **Initialize submodules (if any)**

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

4. **Configure parameters**

   - Edit parameter files for localization, satellite imagery paths, etc.  
   - Set up RViz configuration files (if included in `rviz_satellite/`)  

---

## Usage

Examples of how to run or test the visualization framework:

- Launch the visualization for robot localization and satellite overlay:

  ```bash
  ros2 launch usv_visualizations rviz_satellite_launch.launch.py
  ```

  *(Adjust launch file name/path as needed.)*

- Run helper script:

  ```bash
  python3 usviz/scripts/some_script.py --arg1 value
  ```

- Open RViz with the provided configuration:

  ```bash
  rviz2 -d usv_visualizations/rviz_satellite/config_file.rviz
  ```

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

*(Specify the license here: MIT, GPL, Apache, etc.)*

---

## Contact

For questions or support, contact **Your Name** (or the project maintainer) at *email@yourdomain.com*, or open an issue in this repo.
