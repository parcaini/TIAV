import os
import time
from copy import deepcopy

from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
# reactive planner
from commonroad.scenario.trajectory import Trajectory
# commonroad_dc
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_rp.reactive_planner import ReactivePlanner

import config
from predictor import Predictor

# *************************
# Planner Configurations
# *************************
DT = 0  # planning time step
T_H = 2  # planning horizon
replanning_frequency = 3  # re-plan every i-th time step
plot = False  # plot results


class PlanningGenerator:
    def __init__(self, scenario: Scenario, planning_problem: PlanningProblem, collision_checker_scenario, color='blue'):
        self.planning_problem = planning_problem
        self.initial_state = planning_problem.initial_state

        # goal state configuration
        self.goal = planning_problem.goal
        if hasattr(planning_problem.goal.state_list[0], 'velocity'):
            if planning_problem.goal.state_list[0].velocity.start != 0:
                self.desired_velocity = (
                                                planning_problem.goal.state_list[0].velocity.start +
                                                planning_problem.goal.state_list[0].velocity.end) / 2
            else:
                self.desired_velocity = (planning_problem.goal.state_list[0].velocity.start
                                         + planning_problem.goal.state_list[0].velocity.end) / 2
        else:
            self.desired_velocity = planning_problem.initial_state.velocity

        self.scenario = scenario
        self.color = color
        self.could_not_solve = False
        self.no_solution_counter = 0
        self.is_in_emergency_mode = False
        self.is_crashed = False
        self.new_state_list = None
        self.id = planning_problem.planning_problem_id
        # print debug outputs
        self.DEBUG = False

        # create plot directory
        if plot:
            filename = "path_for_planning_problem_id_" + \
                       str(planning_problem.planning_problem_id)
            self.plot_path = "./plots/" + filename
            os.makedirs(self.plot_path, exist_ok=True)

        # initialize collision checker and road boundary
        self.collision_checker_scenario = collision_checker_scenario

        # initialize planner
        DT = scenario.dt
        self.planner = ReactivePlanner(dt=DT, t_h=T_H, N=int(
            T_H / DT), v_desired=self.desired_velocity)
        self.planner.set_d_sampling_parameters(-3, 3)
        self.planner.set_t_sampling_parameters(
            0.4, self.planner.dT, self.planner.horizon)

        # initialize route planner get reference path
        route_planner = RoutePlanner(scenario, self.planning_problem)
        # TODO sometimes list index out fo range: IndexError
        self.ref_path = route_planner.plan_routes().retrieve_first_route().reference_path

        self.planner.set_reference_path(self.ref_path)

    def update_collision_checker(self, updated_cc):
        self.collision_checker_scenario = updated_cc

    def is_goal_reached(self):
        return self.goal.is_reached(self.x_0)

    def set_crashed(self):
        self.is_crashed = True

    def plan(self):
        # initialize state list
        self.record_state_list = list()
        # init state config
        if not hasattr(self.initial_state, 'acceleration'):
            self.initial_state.acceleration = 0.
        self.x_0 = deepcopy(self.initial_state)
        self.record_state_list.append(self.x_0)
        self.x_cl = None
        self.current_count = 0
        self.planning_times = list()
        # planning
        while not self.goal.is_reached(self.x_0):
            if plot:
                self.rnd = MPRenderer()
            self.current_count = len(self.record_state_list) - 1
            if self.no_solution_counter > config.ScenarioGeneratorConfig.max_emergency_steps \
                    or self.could_not_solve or self.is_crashed:
                # print(f"No more solution - id: {self.planning_problem.planning_problem_id}")
                # once a planner has no solution the ego remains in its last state
                # for plotting and adding it to other planners the last state needs to be extended to a trajectory
                self.new_state_list = Predictor.extend_state_to_trajectory(self.x_0, self.current_count + 1,
                                                                           self.planner.N)
                self.record_state_list.append(self.new_state_list.state_list[0])
                self.x_0 = self.record_state_list[-1]
                self.print_debug("NO_MORE_SOLUTION")
                yield self.x_0, self.x_cl, self.new_state_list, self.ref_path
                continue
            if self.current_count % replanning_frequency == 0 or self.is_in_emergency_mode:
                # new planning cycle -> new optimal trajectory
                # START TIMER
                comp_time_start = time.time()
                self.planner.set_desired_velocity(self.desired_velocity)
                optimal = None
                try:
                    optimal = self.planner.plan(
                        self.x_0, self.collision_checker_scenario, cl_states=self.x_cl, draw_traj_set=plot)
                except Exception as err:
                    if str(err) == 'Initial state or reference incorrect! Curvilinear velocity is negative which ' \
                                   'indicates that the ego vehicle is not driving in the same direction as specified ' \
                                   'by the reference':
                        print(f"Planner {self.planning_problem.planning_problem_id} too far off of reference path")
                        self.could_not_solve = True
                    else:
                        raise err

                comp_time_end = time.time()
                # END TIMER
                if not optimal:
                    # track when recovery was triggered => multiple recoveries needed = good critical scenarios
                    # print(f"Emergency maneuver, continue on planned trajectory - id:"
                    #       f"{self.planning_problem.planning_problem_id}")
                    self.no_solution_counter += 1
                    if self.no_solution_counter > config.ScenarioGeneratorConfig.max_emergency_steps:
                        self.could_not_solve = True
                    self.is_in_emergency_mode = True
                    temp_traj = None
                    if self.new_state_list is None:
                        temp_traj = Trajectory(self.x_0.time_step, [self.x_0])
                        Predictor.enlarge_trajectory(temp_traj, self.planner.N, self.planner.dT, slow_down=True)
                    else:
                        # continue on previous optimal trajectory by extending it
                        temp_traj = Predictor.get_slowed_down_trajectory(self.new_state_list, self.planner.dT)
                    self.new_state_list = Predictor.compute_trajectory(self.current_count + 1, temp_traj,
                                                                       self.planner.dT, self.planner.N,
                                                                       slow_down=False)

                    new_state = deepcopy(self.new_state_list.state_list[0])
                    self.record_state_list.append(new_state)

                    # update init state and curvilinear state
                    self.x_0 = deepcopy(self.record_state_list[-1])
                    self.x_cl = None
                    self.print_debug("NOT_OPTIMAL")
                    # self.x_cl = (optimal[2][1 + temp], optimal[3][1 + temp])

                    yield self.x_0, self.x_cl, self.new_state_list, self.ref_path
                    continue

                self.is_in_emergency_mode = False
                self.new_state_list = self.planner.shift_orientation(
                    optimal[0])

                # TODO add a disclaimer about the asynchronity of generator time_step and multi_motion_system time_step
                # new_state_list after optimal planning always has the previous x0 at state_list[0] and has actually
                #  a length of planning_horizon+1, so we need to shift the list and
                #  set the start state of new_state_list to the next new_state
                self.new_state_list.initial_time_step = self.current_count + 1
                self.new_state_list.state_list = self.new_state_list.state_list[1:]
                # this next line won't change new_state_list in most cases, but the emergency maneuver could
                #  slow down the planner to standing still, and it would return a too short trajectory
                self.new_state_list = Predictor.compute_trajectory(self.current_count + 1, self.new_state_list,
                                                                   self.planner.dT, self.planner.N)
                new_state = self.new_state_list.state_list[0]

                self.record_state_list.append(new_state)

                # update init state and curvilinear state
                self.x_0 = deepcopy(self.record_state_list[-1])
                self.x_cl = (optimal[2][1], optimal[3][1])
                self.print_debug("OPTIMAL")
            else:
                # continue on optimal trajectory
                # enlarge trajectory to planning horizon length before getting next state
                self.new_state_list = Predictor.compute_trajectory(self.current_count + 1, self.new_state_list,
                                                                   self.planner.dT, self.planner.N)
                temp = self.current_count % replanning_frequency
                new_state = deepcopy(self.new_state_list.state_list[0])
                # new_state.time_step = self.current_count + 1
                self.record_state_list.append(new_state)

                # update init state and curvilinear state
                self.x_0 = deepcopy(self.record_state_list[-1])
                # self.x_cl = (optimal[2][1 + temp], optimal[3][1 + temp]) if optimal is not None else None
                self.x_cl = None
                self.print_debug("FROM_PREVIOUS")

            if self.goal.is_reached(self.x_0):
                print(f"goal reached - id: {self.planning_problem.planning_problem_id}, steps: {self.current_count}")
            yield self.x_0, self.x_cl, self.new_state_list, self.ref_path

    def print_debug(self, current_mode):
        if self.DEBUG:
            print(f"ID: {self.id}, Mode: {current_mode}, x0.pos: {self.x_0.position}, vel: {self.x_0.velocity},"
                  f"acc: {self.x_0.acceleration}, step: {self.current_count}")
