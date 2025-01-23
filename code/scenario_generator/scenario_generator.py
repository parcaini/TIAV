import math
import random
from typing import Tuple

import networkx as nx
import numpy as np
from commonroad.common.util import Interval
from commonroad.geometry.shape import Rectangle
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from commonroad_route_planner.route_planner import RoutePlanner
from config import ScenarioGeneratorConfig
from shapely.geometry import LineString
from validation.validator import cut, get_successor_lanelets


class ScenarioGenerator:
    def __init__(self, scenario: Scenario):
        self.config: ScenarioGeneratorConfig = ScenarioGeneratorConfig()
        self.scenario = scenario
        self.lanelet_network = scenario.lanelet_network

    def generate(self, scenario: Scenario, problem_set: PlanningProblemSet):
        for i in range(self.config.amount_pp):
            lanelet = random.choice(scenario.lanelet_network.lanelets)
            full_lanelets = get_successor_lanelets(
                lanelet, scenario.lanelet_network)
            # for ll in full_lanelets:
            # plt.plot(*ll.polygon.shapely_object.exterior.xy)
            # plt.show()
            # plt.clf()
            random_ll = random.choice(full_lanelets)

            goal_area = self.get_possible_goal_for_lanelet(random_ll)
            goal = self.generate_goal(goal_area)
            id = 999 + i
            init_state = self.get_random_init_state(random_ll)
            new_pp = PlanningProblem(id, init_state, goal)
            problem_set.add_planning_problem(new_pp)

        return scenario, problem_set

    def get_random_scenario(self, scenario: Scenario, problem_set: PlanningProblemSet):
        pp_to_add = random.randint(self.config.min_amount_pp, self.config.max_amount_pp)
        for i in range(0, pp_to_add):
            self.add_random_planning_problem(scenario, problem_set)
        self.mutate_scenario(scenario, problem_set)
        return scenario, problem_set

    def mutate_scenario(self, scenario: Scenario, problem_set: PlanningProblemSet):
        if not self.config.do_mutations:
            return
        while self.config.min_amount_pp - len(problem_set.planning_problem_dict.values()) > 0:
            self.add_random_planning_problem(scenario, problem_set)
        number_pp = len(problem_set.planning_problem_dict.values())
        is_mutation_applied = False
        # TODO could also mutate all the problems instead of one at a time (on average)
        for pp in list(problem_set.planning_problem_dict.values()):
            random_number = random.uniform(0, 1)
            if random_number < 1 / number_pp:
                other_random = random.uniform(0, 1)
                # due to 5 applied mutation operators
                amount_mutation_operators = 5
                if other_random < 1 / amount_mutation_operators:
                    is_mutation_applied = True
                    self.mutate_planning_problem_goal(pp)
                other_random = random.uniform(0, 1)
                if other_random < 1 / amount_mutation_operators:
                    is_mutation_applied = self.mutate_planning_problem_velocity(pp)
                other_random = random.uniform(0, 1)
                if other_random < 1 / amount_mutation_operators:
                    is_mutation_applied = True
                    self.mutate_planning_problem_position(pp, scenario)
                other_random = random.uniform(0, 1)
                if other_random < 1 / amount_mutation_operators and number_pp < self.config.max_amount_pp:
                    is_mutation_applied = True
                    self.add_random_planning_problem(scenario, problem_set)
                other_random = random.uniform(0, 1)
                if other_random < 1 / amount_mutation_operators and number_pp > self.config.min_amount_pp:
                    is_mutation_applied = True
                    ScenarioGenerator.remove_random_planning_problem(problem_set)
        if not is_mutation_applied:
            self.mutate_scenario(scenario, problem_set)

    def mutate_planning_problem_position(self, planning_problem: PlanningProblem, scenario: Scenario):
        current_position = planning_problem.initial_state.position
        ll_network = scenario.lanelet_network
        lanelets = ll_network.find_lanelet_by_position([current_position])
        # planning_problem is initially offroad
        if len(lanelets) == 0 or len(lanelets[0]) == 0:
            return
        lanelet: Lanelet = ll_network.find_lanelet_by_id(lanelets[0][0])
        center_line = LineString(lanelet.center_vertices)
        position_modifier = random.uniform(
            -self.config.max_position_modifier, self.config.max_position_modifier)
        new_position = center_line.interpolate(position_modifier)
        planning_problem.initial_state.position = np.array([
            new_position.x, new_position.y])

    def mutate_planning_problem_velocity(self, planning_problem: PlanningProblem):
        current_velocity = planning_problem.initial_state.velocity
        velocity_modifier = random.uniform(
            -self.config.max_init_velocity, self.config.max_init_velocity)
        new_velocity = current_velocity + velocity_modifier
        # TODO velocity sometimes is negative; might come from here
        if self.config.min_velocity <= new_velocity <= self.config.max_velocity:
            planning_problem.initial_state.velocity = new_velocity
            return True
        else:
            return False

    def mutate_planning_problem_goal(self, planning_problem: PlanningProblem):
        # might have problems with circular road networks due to the check for lanelet.successor == 0
        exit_points = set(
            [lanelet.lanelet_id for lanelet in self.lanelet_network.lanelets if len(lanelet.successor) == 0])
        route_planner = RoutePlanner(
            self.scenario, planning_problem)
        # force graph generation (for survival scnearios without goal area the route_planner does not create a graph per default)
        graph = route_planner._create_graph_from_lanelet_network()
        initial_lanelets = self.lanelet_network.find_lanelet_by_position(
            [planning_problem.initial_state.position])
        if len(initial_lanelets) == 0 or len(initial_lanelets[0]) == 0:
            # could not find lanelet for init position
            return
        initial_lanelet_id = initial_lanelets[0][0]
        # make sure to only choose from reachable lanelets
        reachable_lanelets = nx.descendants(graph, initial_lanelet_id)
        if len(reachable_lanelets) == 0:
            reachable_lanelets.add(initial_lanelet_id)
        leaf_nodes = list(exit_points.intersection(reachable_lanelets))
        # TODO improvement: try to choose a different goal area than previously
        #  or use assertion that it could not be mutated to a reachable goal area => maybe validation
        ll_id = random.choice(leaf_nodes)
        lanelet = self.lanelet_network.find_lanelet_by_id(ll_id)
        goal_area = self.get_possible_goal_for_lanelet(lanelet)
        goal = self.generate_goal(goal_area)

        planning_problem.goal = goal

    def add_random_planning_problem(self, scenario: Scenario, problem_set: PlanningProblemSet):
        lanelet = random.choice(scenario.lanelet_network.lanelets)
        full_lanelets = get_successor_lanelets(
            lanelet, scenario.lanelet_network)

        goal_area = self.get_possible_goal_for_lanelet(random.choice(full_lanelets))
        goal = self.generate_goal(goal_area)
        if problem_set.planning_problem_dict:
            last_key, pp = max(list(problem_set.planning_problem_dict.items()))
        else:
            last_key = 0
        new_id = last_key + 1
        init_state = self.get_random_init_state(random.choice(full_lanelets))
        new_pp = PlanningProblem(new_id, init_state, goal)
        problem_set.add_planning_problem(new_pp)

    @staticmethod
    def remove_random_planning_problem(problem_set: PlanningProblemSet):
        pp = random.choice(list(problem_set.planning_problem_dict.values()))
        pp_id_to_remove = problem_set.find_planning_problem_by_id(pp.planning_problem_id)
        problem_set.planning_problem_dict.pop(pp_id_to_remove.planning_problem_id)

    def get_random_init_state(self, lanelet: Lanelet):
        # mandatory fields for init State: [position, velocity, orientation, yaw_rate, slip_angle, time_step]
        velocity = random.uniform(
            self.config.min_init_velocity, self.config.max_init_velocity)

        random_index = random.choice(range(len(lanelet.center_vertices) - 1))
        position = lanelet.center_vertices[random_index]

        # TODO might be out of bounds
        next_point = lanelet.center_vertices[random_index + 1]
        orientation = get_orientation_by_coords(
            (position[0], position[1]), (next_point[0], next_point[1]))

        yaw_rate = 0.0
        slip_angle = 0.0

        return State(velocity=velocity, orientation=orientation, time_step=0, position=position, yaw_rate=yaw_rate,
                     slip_angle=slip_angle)

    # return a rectangle positioned at the end of a lanelet
    def get_possible_goal_for_lanelet(self, lanelet: Lanelet) -> Rectangle:
        # reverse center vertices to get the distance from lanelet end with cut()
        reversed_center_vertices = LineString(lanelet.center_vertices[::-1])
        goal_center_vertices: LineString = cut(
            reversed_center_vertices, self.config.dist_to_end)[0]
        goal_center = goal_center_vertices.centroid

        # last_point is at index 0 because vertices were reversed previously
        last_point = goal_center_vertices.coords[0]
        orientation = get_orientation_by_coords(
            (goal_center.x, goal_center.y), (last_point[0], last_point[1]))
        return Rectangle(self.config.length, self.config.width, np.array([goal_center.x, goal_center.y]), orientation)

    def get_reachable_lanelets(self, id: int, lanelet_network: LaneletNetwork, planning_problem: PlanningProblem):
        route_planner = RoutePlanner(self.scenario, planning_problem)
        paths = route_planner.find_all_shortest_paths()
        # this will be good to check if init+goal are valid

    def generate_goal(self, goal_area: Rectangle):
        goal_state_list = [
            State(position=goal_area, time_step=Interval(0, self.config.max_time_step))]
        return GoalRegion(goal_state_list)


def get_orientation_by_coords(first_point: Tuple[float, float], next_point: Tuple[float, float]):
    a_x, a_y = next_point
    b_x, b_y = first_point
    # compute orientation: https://stackoverflow.com/questions/42258637/how-to-know-the-angle-between-two-vectors
    return math.atan2(a_y - b_y, a_x - b_x)
