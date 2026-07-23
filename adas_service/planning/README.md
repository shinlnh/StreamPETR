# Path-planning module for BV ADAS app

**Description**: This module contains source code for path-planning to create a smooth path to tracking with lines and avoid obstacles.


## Architecture for Path Planning Module
Path Planning is design with MVC architecture.
- The M(Model): EnvironmentBuilder class
- The V(View): PathPlanningVisuallizer class.
- The C(Controller): TrajectoryGenerator class.

## Folder structure

```
planning/
├─ environment_builder/
├─ lane_environment/
├─ trajectory_generator/
```

- `/environment_builder`: contains code for class builder to handle path-planning environment. This class will generate/supply information about lane structure, cars, our vehicle ...
- `/lane_environment`: contains blueprint for the builder to build lane struct, cars...
- `/trajectory_generator`: the class perform the planning algorithm.
- `/tcp_ip_streaming`: contain a darf tcp/ip socket for handling connection with the ROS Bridge for BV_ADAS.
- 2 file `./decisionmakingmodule.cpp` and `./decisionmakingmodule.h` contain the class for setting the policy to choose a path from path planner.

## TODO
- [ ] Move 2 file `./decisionmakingmodule.cpp` & `./decisionmakingmodule.h` into a seperated folder.
- [ ] Consider move `tcp_ip_streaming` outside for the `planning` folder.