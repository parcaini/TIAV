from copy import deepcopy
from typing import List

import numpy as np
from commonroad.scenario.trajectory import State, Trajectory

from config import ScenarioGeneratorConfig


class Predictor:

    @staticmethod
    def compute_trajectory(initial_time_step: int, new_state_list: Trajectory, dt, horizon=20,
                           slow_down=False) -> Trajectory:
        """
        Extend each trajectory from its current time step up to max_length states.
        This trajectory can then be shared between planners to also consider other ego cars during their planning.
        """
        required_length = horizon
        states: List[State] = [state for state in new_state_list.state_list if state.time_step >= initial_time_step]
        trajectory = Trajectory(initial_time_step, states)

        if len(states) < required_length:
            diff = required_length - len(states)
            Predictor.enlarge_trajectory(trajectory, steps=diff, dt=dt, slow_down=slow_down)

        return trajectory

    @staticmethod
    def extend_state_to_trajectory(state: State, current_step: int, required_length: int = 20) -> Trajectory:
        states = []
        for i in range(required_length):
            updated_state = deepcopy(state)
            updated_state.time_step = current_step + i
            states.append(updated_state)
        return Trajectory(states[0].time_step, states)

    # similar to trajectories.CartesianSample.enlarge()
    # slow_down will only slow down future states, the previously computed states are not being modified
    @staticmethod
    def enlarge_trajectory(trajectory: Trajectory, steps: int, dt: float, slow_down=False):
        # omit unwanted properties from previous states
        old_states = []
        for old_state in trajectory.state_list:
            state_props = dict()
            state_props['acceleration'] = old_state.acceleration
            state_props['velocity'] = old_state.velocity
            state_props['orientation'] = old_state.orientation
            state_props['yaw_rate'] = old_state.yaw_rate
            state_props['position'] = old_state.position
            state_props['time_step'] = old_state.time_step
            old_states.append(State(**state_props))

        # get enlarged states
        last_state = trajectory.state_list[-1]
        new_states = Predictor.get_next_states(last_state, steps, dt, slow_down)

        trajectory.state_list = old_states + new_states

    # slow down a full trajectory by modifying each state
    @staticmethod
    def get_slowed_down_trajectory(trajectory: Trajectory, dt: float):
        states = []
        emergency_slow_down_amount = ScenarioGeneratorConfig().emergency_slow_down_amount
        for i, previous_state in enumerate(trajectory.state_list):
            state_props = dict()
            # enlarge acceleration values
            state_props['acceleration'] = -emergency_slow_down_amount

            # enlarge velocities by considering acceleration
            v_temp = previous_state.velocity + dt * state_props['acceleration']
            # remove negative velocities
            v_temp = v_temp * np.greater_equal(v_temp, 0)
            state_props['velocity'] = v_temp
            state_props['acceleration'] = state_props['acceleration'] if v_temp != 0 else 0

            # enlarge orientations
            state_props['orientation'] = previous_state.orientation
            # enlarge curvatures
            state_props['yaw_rate'] = previous_state.yaw_rate

            # enlarge positions
            x = previous_state.position[0] + np.cumsum(dt * v_temp * np.cos(previous_state.orientation))
            y = previous_state.position[1] + np.cumsum(dt * v_temp * np.sin(previous_state.orientation))
            state_props['position'] = np.array([x[0], y[0]])

            # increase time_step because previous states have their own values applied to get the new state
            # so state[0] becomes the new state[1] and so on
            state_props['time_step'] = previous_state.time_step + 1
            states.append(State(**state_props))
        return Trajectory(trajectory.initial_time_step + 1, states)

    # compute future states by using the last_states' values
    @staticmethod
    def get_next_states(last_state: State, steps: int, dt: float, slow_down=False) -> List[State]:
        states = []
        state_to_extend = deepcopy(last_state)
        # create time index
        t = np.arange(1, steps + 1, 1) * dt
        emergency_slow_down_amount = ScenarioGeneratorConfig().emergency_slow_down_amount
        for i in range(steps):
            state_props = dict()
            # enlarge acceleration values
            state_props['acceleration'] = -emergency_slow_down_amount if slow_down else state_to_extend.acceleration

            # enlarge velocities by considering acceleration
            v_temp = state_to_extend.velocity + t[i] * state_props['acceleration']
            # remove negative velocities
            v_temp = v_temp * np.greater_equal(v_temp, 0)
            state_props['velocity'] = v_temp
            state_props['acceleration'] = state_props['acceleration'] if v_temp != 0 else 0

            # enlarge orientations
            state_props['orientation'] = state_to_extend.orientation
            # enlarge curvatures
            state_props['yaw_rate'] = state_to_extend.yaw_rate

            # enlarge positions
            x = state_to_extend.position[0] + np.cumsum(dt * v_temp * np.cos(state_to_extend.orientation))
            y = state_to_extend.position[1] + np.cumsum(dt * v_temp * np.sin(state_to_extend.orientation))
            state_props['position'] = np.array([x[0], y[0]])

            # new time_step
            state_props['time_step'] = state_to_extend.time_step + 1
            state_to_extend = State(**state_props)
            states.append(State(**state_props))
        return states
