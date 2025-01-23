import argparse
import glob
import os
import subprocess
from copy import deepcopy
from typing import List, Any, Tuple

import matplotlib.colors as colors
from commonroad.common.file_reader import CommonRoadFileReader
# commonroad
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory, State
# commonroad_dc
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_object
# commonroad_rp
# /planners/reactive_planner/commonroad_rp was added manually to site-packages because I couldn't link it
from commonroad_rp.parameter import VehModelParameters

import utils
from config import ScenarioGeneratorConfig
from planning_generator import PlanningGenerator
from plotter.plotter import save_images_as_gif, scenario_to_image
from scenario_generator.scenario_generator import ScenarioGenerator
from utils import SimulationResult, PlanningStatus, create_cc_scenario, get_cmap, prepare_scenario, write_scenario
from validation.validator import ScenarioValidator


def save_simulation_value(result_map, pp_id, result):
    if pp_id not in result_map:
        result_map[pp_id] = []
    result_map[pp_id].append(result)


def save_criticality(crit_map, pp_id, step, rating):
    if pp_id not in crit_map:
        crit_map[pp_id] = dict()
    if step not in crit_map[pp_id]:
        crit_map[pp_id][step] = 0
    crit_map[pp_id][step] += rating


def evaluate_trajectories(trajectory_map, pp_id, index):
    # subtract -1 and -2 because list starts at 0 but simulation step starts at 1
    trajectory_new = trajectory_map[pp_id][index - 1]
    trajectory_old = trajectory_map[pp_id][index - 2]
    cropped_traj_a, cropped_traj_b = utils.crop_trajectories(trajectory_new, trajectory_old)
    return not utils.is_trajectory_similar(cropped_traj_a, cropped_traj_b)


def get_prev_trajectory(trajectory_map, pp_id, step):
    # subtract -2 because the comparison is based on the previous trajectories
    return trajectory_map[pp_id][step - 2]


base_dir = "./scenarios"

filename = "showcase-scenario.xml"
# filename = "CHN_Cho-2_2_I-1-1.cr.xml"
# filename = "USA_US101-15_2_T-1.xml"
# filename = "DEU_Cologne-79_2_I-1.cr.xml"
# filename = "USA_US101-23_1_T-1.xml"

validator = ScenarioValidator()


def get_init_scenario():
    scenario_path = os.path.join(base_dir, filename)
    files = sorted(glob.glob(scenario_path))

    # load scenario
    crfr = CommonRoadFileReader(files[0])
    scenario: Scenario
    problem_set: PlanningProblemSet
    scenario, problem_set = crfr.open()
    # do not mutate init scenario when it already has the minimum amount of planning problems
    if len(problem_set.planning_problem_dict.values()) >= ScenarioGeneratorConfig().min_amount_pp:
        return scenario, problem_set

    # generate new scenario until it is valid
    scenario, problem_set = prepare_scenario(crfr, validator)
    no_scenario_found = scenario is None or problem_set is None
    if no_scenario_found or (
            not validator.config.ignore_validation and not validator.is_scenario_valid(scenario, problem_set)):
        print("No valid Scenario found!")
        quit()
    return scenario, problem_set


def get_random_scenario():
    scenario_path = os.path.join(base_dir, filename)
    files = sorted(glob.glob(scenario_path))

    # load scenario
    crfr = CommonRoadFileReader(files[0])
    scenario: Scenario
    problem_set: PlanningProblemSet

    # generate new scenario until it is valid
    scenario, problem_set = prepare_scenario(crfr, validator, random=True)
    no_scenario_found = scenario is None or problem_set is None
    if no_scenario_found or (
            not validator.config.ignore_validation and not validator.is_scenario_valid(scenario, problem_set)):
        print("No valid Scenario found!")
    return scenario, problem_set


def get_mutated_scenario(scenario: Scenario, problem_set: PlanningProblemSet):
    scenario_generator = ScenarioGenerator(scenario)
    for i in range(validator.config.retry_generation):
        new_scenario, new_problem_set = deepcopy(scenario), deepcopy(problem_set)
        scenario_generator.mutate_scenario(new_scenario, new_problem_set)
        is_valid = validator.is_scenario_valid(new_scenario, new_problem_set)
        if is_valid or validator.config.ignore_validation:
            print(f"Mutation to valid Scenario after {i} tries")
            return new_scenario, new_problem_set
    raise Exception('No valid scenario mutation found!')


def get_planner_by_key(planners, key):
    return next((other_gen_object for (other_gen_object, other_gen) in planners if
                 other_gen_object.planning_problem.planning_problem_id == key), None)


def simulate(scenario: Scenario, problem_set: PlanningProblemSet, max_steps=200):
    # vehicle constraints same as reactive planner uses
    vehicle_constraints = VehModelParameters()
    length = vehicle_constraints.veh_length
    width = vehicle_constraints.veh_width

    # init planning generators, each with the same collision checker
    planning_generators: List[Tuple[PlanningGenerator, Any]] = list()
    planning_problems = list(problem_set.planning_problem_dict.values())
    unique_colors = get_cmap(len(planning_problems), 'rainbow')
    for i, problem in enumerate(planning_problems):
        # create collision checker
        # each generator has its own collision_checker but all the other planning problems are replaced with
        # car objects that have the planned trajectory computed by other planners
        collision_checker_scenario = create_cc_scenario(scenario)
        # add objects of all other planning problems to each planning generator
        for prob in planning_problems:
            if prob.planning_problem_id == problem.planning_problem_id:
                continue
            shape = Rectangle(length=length, width=width)
            state_list = list()
            state_list.append(
                State(**{'time_step': 0, 'position': prob.initial_state.position,
                         'orientation': problem.initial_state.orientation}))
            trajectory = Trajectory(0, state_list)
            traj_pred = TrajectoryPrediction(trajectory=trajectory, shape=shape)
            ego = create_collision_object(traj_pred)
            collision_checker_scenario.add_collision_object(ego)

        color = colors.to_hex(unique_colors(i))
        generator = PlanningGenerator(
            scenario, problem, collision_checker_scenario, color)
        generator.planner._DEBUG = False
        # for implementation of parallel execution: probably do not init generator.plan() here
        planning_generators.append((generator, generator.plan()))

    some_generator_running = True
    # start plotting at step 1 because step 0 would be the initial states without trajectories
    step = 1
    # store states of each planner to later save the simulated scenario
    simulation_state_map = dict()
    images = list()
    criticality_ratings_map = dict()
    simulation_trajectory_map = dict()
    crash_counter = 0
    interaction_counter = 0
    tailgating_interaction_counter = 0
    lane_switch_interaction_counter = 0

    while some_generator_running and step < max_steps:
        some_generator_running = False
        # first compute results for each planner and then update the car states for other planners
        # this way planners have the same knowledge of each other when planning their next step
        last_solution_map = dict()
        for generator_object, generator in planning_generators:
            x_0, x_cl, new_state_list, *rest = next(generator, (PlanningStatus.FINISHED, None, None))

            if x_0 != PlanningStatus.FINISHED:
                assert len(new_state_list.state_list) == generator_object.planner.N, \
                    'Simulation: Invalid length of trajectory'
                assert new_state_list.state_list[0].time_step == step
                assert x_0.time_step == step, 'Simulation: planning time step does not match simulation time step'
                last_solution_map[generator_object.planning_problem.planning_problem_id] = (
                    deepcopy(x_0), x_cl, new_state_list, *rest)

            if x_0 != PlanningStatus.FINISHED and not generator_object.could_not_solve \
                    and not generator_object.is_crashed:
                some_generator_running = True

        if not some_generator_running:
            break

        plotting_planners = list()
        crashed_egos_keys = set()
        for key, (x_0, x_cl, new_state_list, *rest) in last_solution_map.items():
            current_planner = get_planner_by_key(planning_generators, key)
            if current_planner.is_crashed:
                continue
            shape = Rectangle(length=length, width=width)
            traj_pred = TrajectoryPrediction(
                trajectory=new_state_list, shape=shape)
            ego = create_collision_object(traj_pred)
            # add all other egos to a new cc to later check if the solution of the planner has a collision
            for other_key, (other_x_0, other_x_cl, other_new_state_list, *other_rest) in last_solution_map.items():
                if key == other_key:
                    continue
                other_traj_pred = TrajectoryPrediction(
                    trajectory=other_new_state_list, shape=shape)
                other_ego = create_collision_object(other_traj_pred)
                has_collision = ego.obstacle_at_time(step).collide(other_ego.obstacle_at_time(step))
                if has_collision:
                    crashed_egos_keys.add(key)

        # update egos that crashed
        for key in crashed_egos_keys:
            planner = next((p for (p, gen) in planning_generators if not p.is_crashed and p.id == key), None)
            if planner:
                planner.set_crashed()
                crash_counter += 1

        previous_ego, previous_key = None, None
        # update car states for other planners with latest computed state
        for i, (gen_object, gen) in enumerate(planning_generators):
            # check if planner has a solution
            if gen_object.planning_problem.planning_problem_id not in last_solution_map:
                continue

            # create new collision checker where ego cars of other planners are added with their new planned trajectory
            # this is necessary because it is currently not possible to update objects that are already added to the cc
            updated_cc = create_cc_scenario(scenario)
            gen_object.update_collision_checker(updated_cc)

            # for plotting: add each ego to planners once
            rest = last_solution_map.get(
                gen_object.planning_problem.planning_problem_id)[2:]
            plotting_planners.append((gen_object, *rest))

            # for saving the simulation
            save_simulation_value(simulation_state_map, gen_object.planning_problem.planning_problem_id,
                                  last_solution_map.get(gen_object.planning_problem.planning_problem_id)[0])

            # save trajectories
            save_simulation_value(simulation_trajectory_map, gen_object.planning_problem.planning_problem_id,
                                  last_solution_map.get(gen_object.planning_problem.planning_problem_id)[2])

            # check if there was an interaction with other planners (=change in trajectory)
            is_interaction = False
            if step > 1:
                is_interaction = evaluate_trajectories(simulation_trajectory_map,
                                                       gen_object.planning_problem.planning_problem_id, step)

            current_ego, current_key = None, None
            # add other ego cars to collision checker
            for j, (key, (x_0, x_cl, new_state_list, *rest)) in enumerate(last_solution_map.items()):
                is_last_iteration = j == len(last_solution_map.items()) - 1
                # ego car is removed from scenario when it reaches its goal
                # it will still be added where it stopped when the planner could not solve the planning problem
                if new_state_list is None and x_0 == PlanningStatus.FINISHED:
                    # reached goal => car is no longer part of the scenario
                    continue

                shape = Rectangle(length=length, width=width)
                traj_pred = TrajectoryPrediction(
                    trajectory=new_state_list, shape=shape)

                ego = create_collision_object(traj_pred)
                # only add objects for other planners, not the object of the own planner
                # adding its own object would break the planner
                if gen_object.planning_problem.planning_problem_id == key:
                    current_ego = ego
                    current_key = key
                    if is_last_iteration:
                        previous_ego = current_ego
                        previous_key = current_key
                    continue
                planner_a: PlanningGenerator = get_planner_by_key(planning_generators, key)
                planner_b: PlanningGenerator = get_planner_by_key(planning_generators, previous_key)

                # categorize interaction
                if i > 0 and is_interaction and previous_ego is not None and key != previous_key \
                        and planner_a is not None and planner_b is not None:
                    # prev_trajectory_a = get_prev_trajectory(simulation_trajectory_map, planner_a.id, step)
                    # prev_trajectory_b = get_prev_trajectory(simulation_trajectory_map, planner_b.id, step)
                    interaction_category: utils.InteractionCategory = \
                        utils.get_interaction_category(planner_a, planner_b, scenario.lanelet_network)
                    if interaction_category == utils.InteractionCategory.LANE_SWITCH:
                        lane_switch_interaction_counter += 1
                        interaction_counter += 1
                        is_interaction = False
                    elif interaction_category == utils.InteractionCategory.TAILGATING:
                        tailgating_interaction_counter += 1
                        interaction_counter += 1
                        is_interaction = False

                if is_last_iteration:
                    previous_ego = current_ego
                    previous_key = current_key
                # rnd = MPRenderer()
                # gen_object.collision_checker_scenario.time_slice(step).draw(rnd, draw_params={})
                # ego.obstacle_at_time(step).draw(rnd, draw_params={})
                # rnd.render(show=False)
                # plt.show()
                gen_object.collision_checker_scenario.add_collision_object(ego)
                criticality = utils.rate_criticality_for_states(gen_object.x_0, x_0)
                save_criticality(criticality_ratings_map, gen_object.planning_problem.planning_problem_id, step,
                                 criticality)

        # plot scenario after each step
        image = scenario_to_image(scenario, plotting_planners, step, show_reference_path=False)
        images.append(image)

        step += 1

    # overall steps is always 2 more than the planner with the most time_steps
    # this is due to the check if all planners are finished
    print('Overall steps:', step)

    goals_not_reached = sum(
        0 if not gen_object.is_crashed and gen_object.is_goal_reached() else 1 for gen_object, gen in
        planning_generators)
    emergency_maneuvers = sum(gen_object.no_solution_counter for gen_object, gen in planning_generators)
    emergency_stops = sum(1 for gen_object, gen in planning_generators if gen_object.could_not_solve)

    return SimulationResult(images=images, scenario=scenario, problem_set=problem_set,
                            simulation_state_map=simulation_state_map, criticality_ratings_map=criticality_ratings_map,
                            simulation_trajectory_map=simulation_trajectory_map,
                            interaction_counter=interaction_counter, goals_not_reached=goals_not_reached,
                            emergency_maneuvers=emergency_maneuvers, collisions=crash_counter, overall_steps=step,
                            number_egos=len(planning_problems),
                            tailgating_interaction_counter=tailgating_interaction_counter,
                            lane_switch_interaction_counter=lane_switch_interaction_counter,
                            algorithm=args.criticality_mode, emergency_stops=emergency_stops)


if __name__ == "__main__":
    # command line arguments
    parser = argparse.ArgumentParser("run_multi_motion_planner_system")
    # parser.add_argument("--trajectory_length", type=int, nargs='?', const=20, default=20,
    #                     help="Adjust the knowledge of other planners planned trajectory.")
    parser.add_argument("--run_result", type=bool, nargs='?', const=False, default=False,
                        help="Run the generated most critical scenario with the reference planner implementation")
    parser.add_argument("--simulation_retries", type=int, nargs='?', const=10, default=10,
                        help="Amount of simulation cycles with applied mutations in between each cycle")
    parser.add_argument("--criticality_mode", type=str,
                        choices=["max_danger", "avg_danger", "random"],
                        default="max_danger", help="Select a rating mode for simulations")
    args = parser.parse_args()

    best_scenario, best_problem_set = get_init_scenario()
    scenario_generator = ScenarioGenerator(best_scenario)
    max_steps = scenario_generator.config.max_time_step
    result_name = f"{filename.split('.')[0]}_{args.criticality_mode}"
    # simulate base scenario to find starting values
    best_result = simulate(best_scenario, best_problem_set, max_steps=max_steps)
    max_avg_danger, max_danger, max_avg_pp_id, max_danger_pp_id = utils.compute_criticality_metrics(
        best_result)
    best_criticality_pp_id, best_criticality = utils.get_criticality_metric(avg_danger=max_avg_danger,
                                                                            max_danger=max_danger,
                                                                            max_avg_pp_id=max_avg_pp_id,
                                                                            max_danger_pp_id=max_danger_pp_id,
                                                                            mode=args.criticality_mode)
    # save gif of initial simulation
    sim_filename = filename.split('.')[0] + "_sim_0"
    save_images_as_gif(best_result.images, result_name, sim_filename)
    result_iteration = 0
    # initial simulation is stored at index 0
    results = [best_result]
    for simulation_number in range(1, args.simulation_retries + 1):
        try:
            # TODO often a scenario is not mutated => same criticality over and over again
            #  might be due to validation
            current_scenario, current_problem_set = best_scenario, best_problem_set
            if args.criticality_mode == "random":
                current_scenario, current_problem_set = get_random_scenario()
            else:
                current_scenario, current_problem_set = get_mutated_scenario(best_scenario, best_problem_set)
        except Exception as e:
            print(e)
            print(f"Error during scenario generation, skipping this simulation {simulation_number}!")
            continue
        try:
            current_result = simulate(current_scenario, current_problem_set, max_steps=max_steps)
        except Exception as e:
            print(e)
            print(f"Exception during simulation, skipping this simulation {simulation_number}!")
            continue
        current_avg_danger, current_danger, current_interactions, current_avg_pp_id, current_danger_pp_id = \
            utils.compute_criticality_metrics(current_result)
        current_criticality_pp_id, current_criticality = \
            utils.get_criticality_metric(avg_danger=current_avg_danger,
                                         max_danger=current_danger,
                                         max_avg_pp_id=current_avg_pp_id,
                                         max_danger_pp_id=current_danger_pp_id,
                                         mode=args.criticality_mode)
        current_result.overall_criticality = current_criticality
        current_result.simulation_number = simulation_number
        print(
            f"Simulation cycle: {simulation_number}, interactions: {current_result.interaction_counter} "
            f"criticality: {current_criticality}")

        # save gif of simulation
        sim_filename = filename.split('.')[0] + "_sim_" + str(simulation_number)
        save_images_as_gif(current_result.images, result_name, sim_filename)
        if current_criticality > best_criticality:
            best_criticality = current_criticality
            best_scenario, best_problem_set = current_scenario, current_problem_set
            best_result = current_result
            result_iteration = simulation_number

        # store results (without images to save up RAM during execution) for saving them as csv
        current_result.images = []
        results.append(current_result)

    utils.save_as_csv(results, filename, result_name)

    # get most critical planning problem that remains in the final test scenario
    critical_key = best_criticality_pp_id
    print(f"Most critical key - criticality: {critical_key} - {best_criticality}, sim_nr: {result_iteration}")

    write_scenario(filename, best_result.simulation_state_map, best_result.scenario, best_result.problem_set,
                   critical_key, result_name)

    if args.run_result:
        pure_filename = filename.split('.')[0]
        planner_path = os.path.abspath(os.getcwd() + "/planners/reactive_planner_zipped_latest/run_combined_planner.py")
        subprocess.run(
            " ".join(
                ["python",
                 planner_path,
                 "--base_dir",
                 f"./plots/{pure_filename}", "--filename", pure_filename + "-multi-planner-simulated.xml"]), shell=True)
