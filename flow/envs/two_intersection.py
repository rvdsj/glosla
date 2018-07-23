import numpy as np
from gym.spaces.box import Box
from gym.spaces.tuple_space import Tuple

from flow.core import rewards
from flow.envs.intersection_env import IntersectionEnvironment


class TwoIntersectionEnvironment(IntersectionEnvironment):
    """
    Fully functional environment. Takes in an *acceleration* as an action. Reward function is negative norm of the
    difference between the velocities of each vehicle, and the target velocity. State function is a vector of the
    velocities for each vehicle.
    """

    @property
    def action_space(self):
        """
        Actions are a set of accelerations from 0 to 15m/s
        :return:
        """

        max_deacc = self.env_params.max_deacc
        max_acc = self.env_params.max_acc

        lb = [-abs(max_deacc), -1] * self.vehicles.num_rl_vehicles
        ub = [max_acc, 1] * self.vehicles.num_rl_vehicles

        return Box(np.array(lb), np.array(ub))

    @property
    def observation_space(self):
        """
        See parent class
        An observation is an array the velocities for each vehicle
        """

        speed = Box(low=0, high=np.inf, shape=(self.vehicles.num_vehicles,))
        absolute_pos = Box(low=0., high=np.inf, shape=(self.vehicles.num_vehicles,))
        lane = Box(low=0, high=(2 - 1), shape=(self.vehicles.num_vehicles,))
        return Tuple((speed, absolute_pos, lane))

    def apply_rl_actions(self, actions):
        """
        See parent class
        """
        acceleration = actions[::2]
        direction = np.round(actions[1::2])

        # re-arrange actions according to mapping in observation space
        sorted_rl_ids = [veh_id for veh_id in self.sorted_ids if veh_id in self.rl_ids]
        # sorted_rl_ids = self.rl_ids

        # represents vehicles that are allowed to change lanes


        self.apply_acceleration(sorted_rl_ids, acceleration)
        self.apply_lane_change(sorted_rl_ids, direction)


    def compute_reward(self, state, rl_actions, **kwargs):
        """
        See parent class
        """
        return rewards.desired_velocity(self, fail=kwargs["fail"])

    def get_state(self, **kwargs):
        """
        See parent class
        The state is an array the velocities for each vehicle
        :return: a matrix of velocities and absolute positions for each vehicle
        """
        length = self.scenario.net_params.additional_params["length"]
        enter_speed = self.scenario.initial_config.additional_params["enter_speed"]
        return np.array([[self.vehicles.get_speed(veh_id)/enter_speed,
                          self.vehicles.get_absolute_position(veh_id)/length,
                          self.vehicles.get_lane(veh_id)]
                         for veh_id in self.sorted_ids])
