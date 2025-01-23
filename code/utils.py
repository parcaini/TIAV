import operator
from copy import deepcopy
from enum import Enum
import os
import math
import csv
import numpy as np
from typing import Dict, Tuple, List
from dataclasses import dataclass

import matplotlib.pyplot as plt

from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType

from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory, State
from commonroad.geometry.shape import Rectangle
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile

from commonroad_dc.boundary.boundary import create_road_boundary_obstacle
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker

from commonroad_rp.parameter import VehModelParameters

from config import ScenarioGeneratorConfig
from planning_generator import PlanningGenerator
from scenario_generator.scenario_generator import ScenarioGenerator
from plotter.plotter import result_path


class PlanningStatus(Enum):
    COULD_NOT_SOLVE = 1
    FINISHED = 2


class InteractionCategory(Enum):
    TAILGATING = 1
    LANE_SWITCH = 2
    NO_INTERACTION = 3


@dataclass
class SimulationResult:
    images: List
    scenario: Scenario
    problem_set: PlanningProblemSet
    simulation_state_map: Dict
    criticality_ratings_map: Dict
    simulation_trajectory_map: Dict
    interaction_counter: int = 0
    tailgating_interaction_counter: int = 0
    lane_switch_interaction_counter: int = 0
    max_danger: float = 0
    avg_danger: float = 0
    goals_not_reached: int = 0
    collisions: int = 0
    emergency_maneuvers: int = 0
    overall_steps: int = 0
    number_egos: int = 0
    simulation_number: int = 0
    algorithm: str = 'max_danger'
    emergency_stops: int = 0


def create_cc_scenario(scenario: Scenario):
    road_boundary_obstacle, road_boundary_sg_triangles = create_road_boundary_obstacle(scenario)
    collision_checker_scenario = create_collision_checker(scenario)
    collision_checker_scenario.add_collision_object(road_boundary_sg_triangles)
    # cc_without_road = create_collision_checker(scenario)
    return collision_checker_scenario


# get n unique colors
def get_cmap(n, name='hsv'):
    """Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
    RGB color; the keyword argument name must be a standard mpl colormap name."""
    return plt.cm.get_cmap(name, n)


def prepare_scenario(crfr, validator, scenario: Scenario = None, problem_set: PlanningProblemSet = None, random=False):
    retry_generation = validator.config.retry_generation
    ignore_validation = validator.config.ignore_validation
    for i in range(retry_generation):
        if crfr is not None and (scenario is None or problem_set is None):
            # TODO deepcopy didn't work due to lanelet_network needing its own deepcopy
            #  therefore solving this with reloading the file
            new_scenario, new_problem_set = crfr.open()
        else:
            new_scenario, new_problem_set = scenario, problem_set
        scenario_generator = ScenarioGenerator(new_scenario)
        if random:
            new_scenario, new_problem_set = scenario_generator.get_random_scenario(new_scenario, new_problem_set)
        else:
            new_scenario, new_problem_set = scenario_generator.generate(
                new_scenario, new_problem_set)
            scenario_generator.mutate_scenario(new_scenario, new_problem_set)

        # validate scenario
        is_scenario_valid = validator.is_scenario_valid(new_scenario, new_problem_set)
        if ignore_validation or is_scenario_valid:
            print(f"Found valid initial Scenario after {i} tries")
            return new_scenario, new_problem_set
    return None, None


def write_scenario(filename, state_map, scenario: Scenario, problem_set, critical_key, sim_name):
    critical_pp = PlanningProblemSet()
    vehicle_constraints = VehModelParameters()
    length = vehicle_constraints.veh_length
    width = vehicle_constraints.veh_width

    # write simulated scenario as XML file with dynamic cars for replication purposes
    for key, state_list in state_map.items():
        if key == critical_key:
            pp = problem_set.find_planning_problem_by_id(key)
            critical_pp.add_planning_problem(pp)
            continue
        # transform trajectory to dynamic obstacle and add it to the scenario
        shape = Rectangle(length=length, width=width)
        trajectory = Trajectory(state_list[0].time_step, state_list)
        prediction = TrajectoryPrediction(trajectory, shape)
        init_state = problem_set.find_planning_problem_by_id(key).initial_state
        unused_id = scenario.generate_object_id()
        dyn_obstacle = DynamicObstacle(unused_id,
                                       ObstacleType.CAR,
                                       shape,
                                       init_state,
                                       prediction)
        scenario.add_objects(dyn_obstacle)

    # write scenario with dynamic cars replicating the steps taken by planners
    path = result_path + sim_name
    os.makedirs(path, exist_ok=True)
    fw = CommonRoadFileWriter(scenario, critical_pp, 'MultiPlanner')
    output_filename = filename.split('.')[0] + f"-multi-planner-simulated-{critical_key}.xml"
    fw.write_to_file(path + '/' + output_filename, OverwriteExistingFile.ALWAYS)


def get_avg_velocity_sim(simulation_state_map):
    velocities = []
    for state_list in simulation_state_map.values():
        velocities += (state.velocity for state in state_list)
    if not velocities:
        return 0
    return np.mean(velocities)


def get_interaction_category(planner_a: PlanningGenerator, planner_b: PlanningGenerator,
                             lanelet_network: LaneletNetwork):
    # if planner_a.new_state_list is None or planner_b.new_state_list is None:
    #     return InteractionCategory.NO_INTERACTION
    # trajectory_a = prev_trajectory_a.state_list
    # trajectory_b = prev_trajectory_b.state_list
    # if not trajectory_a or not trajectory_b:
    #     return InteractionCategory.NO_INTERACTION
    # check if the trajectories from the previous step have been intersecting
    #  if not: do not consider it as an interaction => this neglected a lot of interactions(=false negatives)
    # line_a = LineString([state.position for state in trajectory_a])
    # line_b = LineString([state.position for state in trajectory_b])
    # if not line_a.intersects(line_b):
    #     return InteractionCategory.NO_INTERACTION

    position_a = planner_a.x_0.position
    position_b = planner_b.x_0.position
    lanelets_a = lanelet_network.find_lanelet_by_position([position_a])
    lanelets_b = lanelet_network.find_lanelet_by_position([position_b])
    distance = compute_euclidean_distance(position_a, position_b)
    config = ScenarioGeneratorConfig()
    # when an ego car is within the configured safety distance in the same lanelet,
    #  the interaction is considered TAILGATING
    if set(*lanelets_a) == set(*lanelets_b) and distance < config.car_distance_formula(planner_a.x_0.velocity):
        return InteractionCategory.TAILGATING
    # elif distance < config.lane_switch_distance:
    elif distance < config.car_distance_formula(planner_a.x_0.velocity):
        # this category is tracked very unreliably
        # to improve this: run planners without information about other egos and compare the output with the simulation
        #  where planners have full information of other egos
        return InteractionCategory.LANE_SWITCH
    else:
        return InteractionCategory.NO_INTERACTION


def save_as_csv(simulation_results: List[SimulationResult], filename: str, sim_name: str):
    path = result_path + sim_name
    os.makedirs(path, exist_ok=True)
    output_filename = filename.split('.')[0] + '-multi-planner-simulation-result.csv'
    full_path = path + '/' + output_filename
    with open(full_path, 'a', newline='') as f:
        writer = csv.writer(f, delimiter=",")
        headers = ['sim_num', 'max_danger', 'avg_danger', 'interactions', 'lane_switch_interactions',
                   'tailgating_interactions', 'goals_not_reached', 'collisions',
                   'emergency_maneuvers', 'emergency_stops',
                   'avg_velocity', 'overall_steps', 'number_egos', 'algorithm']
        writer.writerow(headers)
        for sim_result in simulation_results:
            row = [sim_result.simulation_number, sim_result.max_danger, sim_result.avg_danger,
                   sim_result.interaction_counter,
                   sim_result.lane_switch_interaction_counter, sim_result.tailgating_interaction_counter,
                   sim_result.goals_not_reached,
                   sim_result.collisions, sim_result.emergency_maneuvers,
                   sim_result.emergency_stops,
                   get_avg_velocity_sim(sim_result.simulation_state_map), sim_result.overall_steps,
                   sim_result.number_egos, sim_result.algorithm]
            writer.writerow(row)


def rate_criticality_for_states(state_a: State, state_b: State):
    # compute relative velocity of A to B
    # this is probably overcomplicated by splitting both velocities into x,y direction and then doing the computation
    sin_a, cos_a = compute_sin_cos_from_orientation(state_a.orientation)
    sin_b, cos_b = compute_sin_cos_from_orientation(state_b.orientation)
    velocity_x_a, velocity_y_a = compute_2D_velocity_vectors(state_a.velocity, sin_a, cos_a)
    velocity_x_b, velocity_y_b = compute_2D_velocity_vectors(state_b.velocity, sin_b, cos_b)
    diff_x = velocity_x_a - velocity_x_b
    diff_y = velocity_y_a - velocity_y_b
    relative_velocity = math.sqrt(diff_x ** 2 + diff_y ** 2)

    # compute euclidean distance
    euclidean_distance = compute_euclidean_distance(state_a.position, state_b.position)

    # get criticality metric
    result = relative_velocity / euclidean_distance
    return result


def compute_euclidean_distance(pos_a: Tuple[float, float], pos_b: Tuple[float, float]):
    position_diff = (pos_b[0] - pos_a[0]) ** 2 + (pos_b[1] - pos_a[1]) ** 2
    return math.sqrt(position_diff)


def compute_criticality_metrics(simulation_result: SimulationResult):
    pp_id: int
    value: Dict[int, float]
    criticality_map = simulation_result.criticality_ratings_map
    max_avg_danger, max_danger = 0, 0
    max_avg_pp_id, max_danger_pp_id = 0, 0
    for pp_id, value in criticality_map.items():
        current_max_avg_danger = np.mean([val[1] for val in value.items()])
        current_max_time_step, current_max_danger = max(value.items(), key=operator.itemgetter(1))
        if max_avg_danger < current_max_avg_danger:
            max_avg_pp_id = pp_id
            max_avg_danger = current_max_avg_danger
        if max_danger < current_max_danger:
            max_danger_pp_id = pp_id
            max_danger = current_max_danger
    simulation_result.max_danger = max_danger
    simulation_result.avg_danger = max_avg_danger
    return max_avg_danger, max_danger, simulation_result.interaction_counter, max_avg_pp_id, max_danger_pp_id


def get_criticality_metric(avg_danger, max_danger, max_avg_pp_id, max_danger_pp_id,
                           mode='max_danger'):
    if mode == 'avg_danger':
        return max_avg_pp_id, avg_danger
    else:
        return max_danger_pp_id, max_danger


# compare trajectories (with same length) with given tolerance deviation
def is_trajectory_similar(trajectory_a: Trajectory, trajectory_b: Trajectory, tolerance_distance: int = 1):
    if len(trajectory_a.state_list) != len(trajectory_b.state_list):
        print('Trajectory comparison: Not the same length!')
        return False

    for state_a, state_b in zip(trajectory_a.state_list, trajectory_b.state_list):
        is_pos_equal = is_position_similar(state_a.position, state_b.position, tolerance_distance)
        if not is_pos_equal:
            return False
    return True


def is_position_similar(pos_a: Tuple[float, float], pos_b: Tuple[float, float], tolerance_distance: int = 1) -> bool:
    euclidean_distance = compute_euclidean_distance(pos_a, pos_b)
    if euclidean_distance < tolerance_distance:
        return True
    else:
        return False


# crop the second trajectory to the first one, so both have the same length and initial time step
def crop_trajectories(trajectory_a: Trajectory, trajectory_b: Trajectory) -> Tuple[Trajectory, Trajectory]:
    if trajectory_a.initial_time_step < trajectory_b.initial_time_step:
        raise Exception('trajectory_b has a larger time_step and therefore can not be cropped to trajectory_a')
    time_step_diff = trajectory_a.initial_time_step - trajectory_b.initial_time_step
    cut_trajectory_a = get_cut_trajectory_end(trajectory_a, time_step_diff)
    cut_trajectory_b = get_cut_trajectory_start(trajectory_b, time_step_diff)
    return cut_trajectory_a, cut_trajectory_b


def get_cut_trajectory_end(trajectory: Trajectory, amount: int) -> Trajectory:
    states = deepcopy(trajectory.state_list)
    cut_off = len(states) - amount
    states = states[:cut_off]
    return Trajectory(states[0].time_step, states)


def get_cut_trajectory_start(trajectory: Trajectory, amount: int) -> Trajectory:
    states = deepcopy(trajectory.state_list)
    states = states[amount:]
    return Trajectory(states[0].time_step, states)


def compute_2D_velocity_vectors(velocity: float, sin, cos):
    vector_x = velocity * cos
    vector_y = velocity * sin
    return vector_x, vector_y


def compute_sin_cos_from_orientation(orientation: float):
    cos = math.cos(orientation)
    sin = math.sin(orientation)
    return sin, cos
