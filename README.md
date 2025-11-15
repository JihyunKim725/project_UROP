# project_UROP
UROP project about rrt algorithm

A ROS Catkin workspace for the RRT planner (package: `rrt`) used in the F1/10 project.

Key files
- Package CMake entry: [src/CMakeLists.txt](src/CMakeLists.txt)
- Package specific CMake: [src/f110-rrt/CMakeLists.txt](src/f110-rrt/CMakeLists.txt)
- Node source: [src/f110-rrt/node/rrt_node.cpp](src/f110-rrt/node/rrt_node.cpp)
- Workspace setup helpers:
  - [devel/setup.sh](devel/setup.sh)
  - [devel/local_setup.fish](devel/local_setup.fish)
  - [build/catkin_generated/generate_cached_setup.py](build/catkin_generated/generate_cached_setup.py)
- CMake/build internals: [build/CMakeFiles/Makefile.cmake](build/CMakeFiles/Makefile.cmake), [build/CMakeCache.txt](build/CMakeCache.txt)

Prerequisites
- ROS Noetic (or compatible ROS distribution)
- catkin build tools (catkin_make / catkin_tools)
- Standard Linux build toolchain (gcc / g++ / make / cmake)

Quick build (catkin_make)
1. From workspace root:
   - mkdir -p build && cd build
   - cmake ..   (optional; catkin_make will configure automatically)
   - catkin_make
2. Source the workspace:
   - Bash: source devel/setup.bash
   - Fish: source devel/setup.fish
   See: [devel/setup.sh](devel/setup.sh) and [devel/local_setup.fish](devel/local_setup.fish)

Run the node
- After sourcing the workspace, run:
  - rosrun rrt rrt_node
  The executable target is declared in [src/f110-rrt/CMakeLists.txt](src/f110-rrt/CMakeLists.txt) (target name `rrt_node`) and built into devel/lib/rrt/rrt_node.

Notes and troubleshooting
- If catkin cannot be found during CMake configure, ensure a ROS setup file has been sourced before configuring. See the top-level toplevel CMake logic in [src/CMakeLists.txt](src/CMakeLists.txt).
- Generated setup utilities and environment-hook logic are created under `build/catkin_generated` and `atomic_configure` (e.g. [build/catkin_generated/generate_cached_setup.py](build/catkin_generated/generate_cached_setup.py)). If environment hooks are not applied, inspect the generated scripts in `build/catkin_generated/installspace/`.
- For build failures inspect generated CMake logs: [build/CMakeFiles/CMakeConfigureLog.yaml](build/CMakeFiles/CMakeConfigureLog.yaml) and the makefiles under [build/CMakeFiles](build/CMakeFiles/).

Repository layout
- build/         — CMake and catkin-generated build artifacts
- devel/         — catkin devel-space (sourced after build)
- src/           — packages and source code (main package under `src/f110-rrt`)
- xtl/           — third-party CMake-managed component used by the project

License
- See repository LICENSE file (if present). The workspace is organized using standard ROS/catkin templates.

If you need a more detailed developer guide (unit tests, CI, or containerized build), indicate which area to expand.
