from commonroad.scenario.lanelet import LaneletType


class ValidationConfig:

    # safety_time in seconds will be used to calculate safe distances to other ego cars
    def __init__(self, safety_time=2) -> None:
        # time in seconds for a safe distance between two cars
        self.safety_time = safety_time

    lanelet_types = {LaneletType.HIGHWAY, LaneletType.ACCESS_RAMP,
                     LaneletType.EXIT_RAMP, LaneletType.URBAN}
    # this validation is not needed when taking valid scenarios from the online dataset
    validate_lanelet_type_and_direction = False
    lanelet_same_direction_only = False

    def car_distance_formula(self, speed): return speed * self.safety_time

    min_dist_goal = 50
    retry_generation = 10000

    ignore_validation = False  # for debugging purposes


class ScenarioGeneratorConfig:
    dist_to_end = 10.0  # length of goal area at the end of a lanelet
    length = 10.0  # ego car dimensions
    width = 5.0
    amount_pp = 1  # initially added planning problems
    max_amount_pp = 6  # max amount of planning problems in a scenario
    min_amount_pp = 2
    do_mutations = True  # for debugging
    max_position_modifier = 50  # max amount for position mutations; min amount is -max_position_modifier
    max_velocity = 25  # max velocity of an ego by mutation
    min_velocity = 5  # min velocity of an ego by mutation
    max_init_velocity = 10  # max init velocity for new egos; also max velocity modifier for velocity mutations
    min_init_velocity = 5  # see max_init_velocity
    car_distance_formula = ValidationConfig().car_distance_formula
    max_time_step = 200  # maximum number of steps that each planner can take
    emergency_slow_down_amount = 5  # how much a car is slowing down for each step it spends in emergency mode
    max_emergency_steps = 4
