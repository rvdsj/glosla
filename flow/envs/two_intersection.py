import numpy as np
from gym.spaces.box import Box
from gym.spaces.tuple_space import Tuple
import  traci
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

        lb = [-abs(max_deacc), -1]
        ub = [max_acc, 1]

        print (lb)
        print (ub)
        return Box(np.array(lb), np.array(ub))



    @property
    def observation_space(self):
        """
        See parent class
        An observation is an array the velocities for each vehicle
        """

        speed = Box(low=0, high=np.inf, shape=(1,))
        absolute_pos = Box(low=0., high=np.inf, shape=(1,))
        lane = Box(low=0, high=(2 - 1), shape=(1,))

        return Tuple((speed, absolute_pos,lane))

    def apply_rl_actions(self, rl_actions):
        """
        See parent class
        """

        acceleration = rl_actions[::2]
        direction = np.round(rl_actions[1::2])

        # re-arrange actions according to mapping in observation space
        sorted_rl_ids = [veh_id for veh_id in self.sorted_ids if veh_id in self.rl_ids]
        self.apply_acceleration(sorted_rl_ids, acc=acceleration)
        self.apply_lane_change(sorted_rl_ids, direction=direction)


    def compute_reward(self, state, rl_actions, **kwargs):
        """
        See parent class
        """

        sorted_rl_ids = [veh_id for veh_id in self.sorted_ids if veh_id in self.rl_ids]

        return rewards.desired_velocity(self, fail=kwargs["fail"])

    def get_state(self, **kwargs):
        """
        See parent class
        The state is an array the velocities for each vehicle
        :return: a matrix of velocities and absolute positions for each vehicle
        """
        length = self.scenario.net_params.additional_params["length"]
        enter_speed = self.scenario.initial_config.additional_params["enter_speed"]

        distanceToJunction = {}
        #all lenths are equal
        laneLenth = self.traci_connection.lane.getLength("altleft1_0")
        for veh_id in self.sorted_ids:
            xPosition = self.traci_connection.vehicle.getLanePosition(veh_id)
            distanceToJunction[veh_id] = laneLenth - xPosition

        return np.array([[self.vehicles.get_speed(veh_id)/enter_speed,
                          self.vehicles.get_absolute_position(veh_id)/length,self.vehicles.get_lane(veh_id)]
                         for veh_id in self.sorted_ids])
