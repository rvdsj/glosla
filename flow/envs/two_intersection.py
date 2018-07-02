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
        # return Box(low=-np.abs(self.env_params.max_deacc),
        #            high=self.env_params.max_acc,
        #            shape=(self.vehicles.num_rl_vehicles, ))

        #return Box(low=-np.abs(self.env_params.max_deacc),
        #           high=self.env_params.max_acc,
        #           shape=(1, ))


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
        # speed = Box(low=0, high=np.inf, shape=(self.vehicles.num_vehicles,))
        # absolute_pos = Box(low=0., high=np.inf, shape=(self.vehicles.num_vehicles,))
        # remaining_green_light = Box(low=0., high=90, shape=(self.vehicles.num_vehicles,))
        # next_green_light = Box(low=0., high=np.inf, shape=(self.vehicles.num_vehicles,))
        # cycle = Box(low=0., high=np.inf, shape=(self.vehicles.num_vehicles,))
        # lane = Box(low=0, high=(2 - 1), shape=(self.vehicles.num_vehicles,))
        # distance_to_junction = Box(low=0, high=np.inf, shape=(self.vehicles.num_vehicles,))
        # noOfFollowers = Box(low=0, high=np.inf, shape=(self.vehicles.num_vehicles,))
        # noOfLeaders = Box(low=0, high=np.inf, shape=(self.vehicles.num_vehicles,))
        # # return Tuple((speed, absolute_pos,remeaining_green_light, next_green_light,cycle, lane,distance_to_junction,noOfFollowers,noOfLeaders))
        # return Tuple((speed, absolute_pos))speed = Box(low=0, high=np.inf, shape=(self.vehicles.num_vehicles,))

        speed = Box(low=0, high=np.inf, shape=(1,))
        absolute_pos = Box(low=0., high=np.inf, shape=(1,))
        remaining_green_light = Box(low=0., high=90, shape=(1,))
        next_green_light = Box(low=0., high=np.inf, shape=(1,))
        cycle = Box(low=0., high=np.inf, shape=(1,))
        lane = Box(low=0, high=(2 - 1), shape=(1,))
        distance_to_junction = Box(low=0, high=np.inf, shape=(1,))
        noOfFollowers = Box(low=0, high=np.inf, shape=(1,))
        noOfLeaders = Box(low=0, high=np.inf, shape=(1,))
        return Tuple((speed, absolute_pos, distance_to_junction,lane,noOfFollowers,noOfLeaders))

    def apply_rl_actions(self, rl_actions):
        """
        See parent class
        """

        acceleration = rl_actions[::2]
        direction = np.round(rl_actions[1::2])

        #print("direction")
        #print(direction)

        # re-arrange actions according to mapping in observation space
        sorted_rl_ids = [veh_id for veh_id in self.sorted_ids if veh_id in self.rl_ids]
        self.apply_acceleration(sorted_rl_ids, acc=acceleration)
        self.apply_lane_change(sorted_rl_ids, direction=direction)


    def compute_reward(self, state, rl_actions, **kwargs):
        """
        See parent class
        """

        sorted_rl_ids = [veh_id for veh_id in self.sorted_ids if veh_id in self.rl_ids]

        #for i, veh_id in enumerate(sorted_rl_ids):
            #self.traci_connection.vehicle

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
                          self.vehicles.get_absolute_position(veh_id)/length, distanceToJunction[veh_id],self.vehicles.get_lane(veh_id),0,0]
                         for veh_id in self.sorted_ids])
