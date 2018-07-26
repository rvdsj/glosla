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
        distance_to_intersection = Box(low=0., high=np.inf, shape=(self.vehicles.num_vehicles,))
        lane = Box(low=0, high=(2 - 1), shape=(self.vehicles.num_vehicles,))
        no_leaders = Box(low=0., high=np.inf, shape=(self.vehicles.num_vehicles,))
        no_follower = Box(low=0., high=np.inf, shape=(self.vehicles.num_vehicles,))
        remaining_green = Box(low=0., high=np.inf, shape=(self.vehicles.num_vehicles,))
        next_green = Box(low=0., high=np.inf, shape=(self.vehicles.num_vehicles,))
        return Tuple((speed, absolute_pos,distance_to_intersection, lane, no_leaders,no_follower,remaining_green,next_green))

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

        # all lenths are equal
        altleft_length = self.traci_connection.lane.getLength("altleft1_0")
        altbottom_length = self.traci_connection.lane.getLength("altbottom1_0")
        altright_length = self.traci_connection.lane.getLength("altright1_0")
        alttop_length = self.traci_connection.lane.getLength("alttop1_0")

        veh_ids_altleft1_0 = self.traci_connection.lane.getLastStepVehicleIDs("altleft1_0")
        veh_ids_altleft1_1 = self.traci_connection.lane.getLastStepVehicleIDs("altleft1_1")
        veh_ids_altright1_0 = self.traci_connection.lane.getLastStepVehicleIDs("altright1_0")
        veh_ids_altright1_1 = self.traci_connection.lane.getLastStepVehicleIDs("altright1_1")
        veh_ids_altbottom1_0 = self.traci_connection.lane.getLastStepVehicleIDs("altbottom1_0")
        veh_ids_altbottom1_1 = self.traci_connection.lane.getLastStepVehicleIDs("altbottom1_1")
        veh_ids_alttop1_0 = self.traci_connection.lane.getLastStepVehicleIDs("alttop1_0")
        veh_ids_alttop1_1 = self.traci_connection.lane.getLastStepVehicleIDs("alttop1_1")

        distanceToJunction = {}
        no_leader= {}
        no_follower = {}
        remaining_green_light = {}
        next_green_light = {}

        for veh_id in self.sorted_ids:
            no_leader[veh_id] = 0
            no_follower[veh_id] = 0
            remaining_green_light[veh_id]=0
            next_green_light[veh_id] =0

        no_leader,no_follower=self.calculateNoOfLeadersAndFollowers(veh_ids_altleft1_0, altleft_length,no_leader,no_follower)
        no_leader,no_follower=self.calculateNoOfLeadersAndFollowers(veh_ids_altleft1_1, altleft_length,no_leader,no_follower)
        no_leader,no_follower=self.calculateNoOfLeadersAndFollowers(veh_ids_altright1_0, altright_length,no_leader,no_follower)
        no_leader,no_follower=self.calculateNoOfLeadersAndFollowers(veh_ids_altright1_1, altright_length,no_leader,no_follower)
        no_leader,no_follower=self.calculateNoOfLeadersAndFollowers(veh_ids_altbottom1_0, altbottom_length,no_leader,no_follower)
        no_leader,no_follower=self.calculateNoOfLeadersAndFollowers(veh_ids_altbottom1_1, altbottom_length,no_leader,no_follower)
        no_leader,no_follower=self.calculateNoOfLeadersAndFollowers(veh_ids_alttop1_0, alttop_length,no_leader,no_follower)
        no_leader,no_follower=self.calculateNoOfLeadersAndFollowers(veh_ids_alttop1_1, alttop_length,no_leader,no_follower)

        remaining_green_light, next_green_light= self.phaseSelector1(veh_ids_altleft1_0, altleft_length,remaining_green_light,next_green_light)
        remaining_green_light, next_green_light= self.phaseSelector1(veh_ids_altleft1_1, altleft_length,remaining_green_light,next_green_light)
        remaining_green_light, next_green_light= self.phaseSelector1(veh_ids_altright1_0, altright_length,remaining_green_light,next_green_light)
        remaining_green_light, next_green_light=self.phaseSelector1(veh_ids_altright1_1, altright_length,remaining_green_light,next_green_light)
        remaining_green_light, next_green_light=self.phaseSelector2(veh_ids_altbottom1_0, altbottom_length,remaining_green_light,next_green_light)
        remaining_green_light, next_green_light=self.phaseSelector2(veh_ids_altbottom1_1, altbottom_length,remaining_green_light,next_green_light)
        remaining_green_light, next_green_light=self.phaseSelector2(veh_ids_alttop1_0, alttop_length,remaining_green_light,next_green_light)
        remaining_green_light, next_green_light=self.phaseSelector2(veh_ids_alttop1_1, alttop_length,remaining_green_light,next_green_light)

        for veh_id in self.sorted_ids:
            xPosition = self.traci_connection.vehicle.getLanePosition(veh_id)
            distanceToJunction[veh_id] = altleft_length - xPosition

        return np.array([[self.vehicles.get_speed(veh_id)/enter_speed,
                          self.vehicles.get_absolute_position(veh_id)/length,
                          distanceToJunction[veh_id],
                          self.vehicles.get_lane(veh_id),
                          no_leader[veh_id],
                          no_follower[veh_id],
                          remaining_green_light[veh_id],
                          next_green_light[veh_id]]
                          for veh_id in self.sorted_ids])


    def phaseSelector1(self, veh_ids, positionJunctionX,remaining_green_light,next_green_light):
        arraySize = len(veh_ids)
        for x in range(0, arraySize):
            print("vehicle Id", veh_ids[x])

            phrase = self.traci_connection.trafficlights.getRedYellowGreenState("center")

            if phrase == "rrrrGGGgrrrrGGGg":
                print("PHRASE 1")
                phraseNo = 1
                remaining_green_light, next_green_light=self.calculateRemainigtimeANDNextGreenLight(veh_ids[x], positionJunctionX, phraseNo,remaining_green_light,next_green_light)

            elif phrase == "rrrryyygrrrryyyg":
                print("PHRASE 2")
                phraseNo = 2
                remaining_green_light, next_green_light=self.calculateRemainigtimeANDNextGreenLight(veh_ids[x], positionJunctionX, phraseNo,remaining_green_light,next_green_light)

            elif phrase == "rrrrrrrGrrrrrrrG":
                print("PHRASE 3")
                phraseNo = 3
                remaining_green_light, next_green_light=self.calculateRemainigtimeANDNextGreenLight(veh_ids[x], positionJunctionX, phraseNo,remaining_green_light,next_green_light)

            elif phrase == "rrrrrrryrrrrrrry":
                print("PHRASE 4")
                phraseNo = 4
                remaining_green_light, next_green_light=self.calculateRemainigtimeANDNextGreenLight(veh_ids[x], positionJunctionX, phraseNo,remaining_green_light,next_green_light)

            elif phrase == "GGGgrrrrGGGgrrrr":
                print("PHRASE 5")
                phraseNo = 5
                remaining_green_light, next_green_light=self.calculateRemainigtimeANDNextGreenLight(veh_ids[x], positionJunctionX, phraseNo,remaining_green_light,next_green_light)

            elif phrase == "yyygrrrryyygrrrr":
                print("PHRASE 6")
                phraseNo = 6
                remaining_green_light, next_green_light=self.calculateRemainigtimeANDNextGreenLight(veh_ids[x], positionJunctionX, phraseNo,remaining_green_light,next_green_light)

            elif phrase == "rrrGrrrrrrrGrrrr":
                print("PHRASE 7")
                phraseNo = 7
                remaining_green_light, next_green_light=self.calculateRemainigtimeANDNextGreenLight(veh_ids[x], positionJunctionX, phraseNo,remaining_green_light,next_green_light)

            elif phrase == "rrryrrrrrrryrrrr":
                print("PHRASE 8")
                phraseNo = 8
                remaining_green_light, next_green_light=self.calculateRemainigtimeANDNextGreenLight(veh_ids[x], positionJunctionX, phraseNo,remaining_green_light,next_green_light)

        print("-------------------")
        return remaining_green_light,next_green_light


    def phaseSelector2(self,veh_ids, positionJunctionX,remaining_green_light,next_green_light):
        arraySize = len(veh_ids)
        for x in range(0, arraySize):
            print("vehicle Id", veh_ids[x])

            phrase = self.traci_connection.trafficlights.getRedYellowGreenState("center")
            if phrase == "GGGgrrrrGGGgrrrr":
                print ("PHRASE 1")
                phraseNo = 1
                remaining_green_light, next_green_light=self.calculateRemainigtimeANDNextGreenLight(veh_ids[x], positionJunctionX, phraseNo,remaining_green_light,next_green_light)

            elif phrase == "yyygrrrryyygrrrr":
                print("PHRASE 2")
                phraseNo = 2
                remaining_green_light, next_green_light=self.calculateRemainigtimeANDNextGreenLight(veh_ids[x], positionJunctionX, phraseNo,remaining_green_light,next_green_light)

            elif phrase == "rrrGrrrrrrrGrrrr":
                print("PHRASE 3")
                phraseNo = 3
                remaining_green_light, next_green_light=self.calculateRemainigtimeANDNextGreenLight(veh_ids[x], positionJunctionX, phraseNo,remaining_green_light,next_green_light)

            elif phrase == "rrryrrrrrrryrrrr":
                print("PHRASE 4")
                phraseNo = 4
                remaining_green_light, next_green_light=self.calculateRemainigtimeANDNextGreenLight(veh_ids[x], positionJunctionX, phraseNo,remaining_green_light,next_green_light)

            elif phrase == "rrrrGGGgrrrrGGGg":
                print("PHRASE 5")
                phraseNo = 5
                remaining_green_light, next_green_light=self.calculateRemainigtimeANDNextGreenLight(veh_ids[x], positionJunctionX, phraseNo,remaining_green_light,next_green_light)

            elif phrase == "rrrryyygrrrryyyg":
                print("PHRASE 6")
                phraseNo = 6
                remaining_green_light, next_green_light=self.calculateRemainigtimeANDNextGreenLight(veh_ids[x], positionJunctionX, phraseNo,remaining_green_light,next_green_light)

            elif phrase == "rrrrrrrGrrrrrrrG":
                print("PHRASE 7")
                phraseNo = 7
                remaining_green_light, next_green_light=self.calculateRemainigtimeANDNextGreenLight(veh_ids[x], positionJunctionX, phraseNo,remaining_green_light,next_green_light)

            elif phrase == "rrrrrrryrrrrrrry":
                print("PHRASE 8")
                phraseNo = 8
                remaining_green_light, next_green_light=self.calculateRemainigtimeANDNextGreenLight(veh_ids[x], positionJunctionX, phraseNo,remaining_green_light,next_green_light)

        print("-------------------")
        return remaining_green_light,next_green_light



    def calculateRemainigtimeANDNextGreenLight(self,veh_id, positionJunctionX, phraseNo,remaining_green_light,next_green_light):

        remaining = (self.traci_connection.trafficlights.getNextSwitch("center") - self.traci_connection.simulation.getCurrentTime()) / 1000

        if phraseNo == 1:
            nextGreenPhrase = 57 + remaining
            remaining_green = (self.traci_connection.trafficlights.getNextSwitch("center") - self.traci_connection.simulation.getCurrentTime()) / 1000

        elif phraseNo == 2:
            nextGreenPhrase = 54 + remaining
            remaining_green = 0

        elif phraseNo == 3:
            nextGreenPhrase = 48 + remaining
            remaining_green =0

        elif phraseNo == 4:
            nextGreenPhrase = 45 + remaining
            remaining_green =0

        elif phraseNo == 5:
            nextGreenPhrase = 12 + remaining
            remaining_green = 0

        elif phraseNo == 6:
            nextGreenPhrase = 9 + remaining
            remaining_green =0

        elif phraseNo == 7:
            nextGreenPhrase = 6 + remaining
            remaining_green =0

        elif phraseNo == 8:
            nextGreenPhrase = (remaining)
            remaining_green = 0

        remaining_green_light[veh_id]=remaining_green
        next_green_light[veh_id]=nextGreenPhrase
        return  remaining_green_light,next_green_light


    def calculateNoOfLeadersAndFollowers(self, vehicleList, laneLength,no_leader, no_follower):

        for veh_id in vehicleList:
            length = len(vehicleList)
            # reference vehicle
            xPositionRef = self.traci_connection.vehicle.getLanePosition(veh_id)
            distanceRef = laneLength - xPositionRef
            leaderCount = 0
            followerCount=0

            for x in range(0, length):
                xPosition = self.traci_connection.vehicle.getLanePosition(vehicleList[x])
                distance = laneLength - xPosition
                if distanceRef > distance:
                    leaderCount = leaderCount + 1
                elif distanceRef < distance:
                    followerCount= followerCount+1

            print("no_leader[veh_id] ",no_leader)
            print ("no_follower[veh_id]",no_follower)
            no_leader[veh_id] = leaderCount
            no_follower[veh_id] = followerCount

        return  no_leader,no_follower