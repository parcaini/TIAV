# TIAV
Simulate multiple instances of the CommonRoad Reactive Planner algorithm in a single scenario to observe AV interactions.

### Installation
The software is developed and tested on WSL2 (therefore it should also work on OS X and Linux).

For the Python installation an Anaconda environment at Python version 3.7 is used.

Dependencies:
Some of the `CommonRoad` need to be installed manually, e.g. the `commonroad-drivability-checker`, therefore please refer to the installation instructions [here](https://gitlab.lrz.de/tum-cps/commonroad-drivability-checker).
Besides that the installation of dependencies is quite similar to the ones used in the `ReactivePlanner`([README](planners/reactive_planner_zipped_latest/README.md)).

Other dependencies can be installed with `pip install -r requirements.txt`

#### Reactive Planner

The system is designed for the **Reactive Planner** in `./planners`.
On the tested setup the planner had to be **manually added to the python site-packages**.

### Configuration
First, configure simulation details in `config.py` or use the default configuration.
Then set a directory with CommonRoad scenarios and the base scenario that should be used for simulations in

`run_multi_motion_planner_system.py`
```
base_dir = "./scenarios"
filename = "showcase-scenario.xml"
```

### Run TIAV

Generates 10 simulations using the criticality fitness function `danger`:

`python run_multi_motion_planner_system.py --simulation_retries 10 --criticality_mode max_danger`

Result GIFs for each simulation and a CSV are written to `/results`,
it also includes a XML CommonRoad scenario with the most critical ego vehicle as `PlanninProblem` and the other simulated ego vehicles as `DynamicObstacles`, this can be used to simulate the planner again in the generated scenario.