import random

import numpy as np

import traci

from flow.scenarios.base_scenario import Scenario


class MyTwoWayIntersectionScenario(Scenario):

    def __init__(self, name, generator_class, vehicles, net_params,
                 initial_config=None):
        """
        Initializes a two-way intersection scenario. Required net_params: horizontal_length_before,
        horizontal_length_after, horizontal_lanes, vertical_length_before, vertical_length_after, vertical_lanes,
        speed_limit. Required initial_config: positions.
        See Scenario.py for description of params.
        """
        self.left_len = net_params.additional_params["horizontal_length_in"]
        self.right_len = net_params.additional_params["horizontal_length_out"]
        self.bottom_len = net_params.additional_params["vertical_length_in"]
        self.top_len = net_params.additional_params["vertical_length_out"]

        self.horizontal_junction_len = 2.9 + 3.3 * net_params.additional_params["vertical_lanes"]
        self.vertical_junction_len = 2.9 + 3.3 * net_params.additional_params["horizontal_lanes"]
        self.inner_space_len = 0.28

        # instantiate "length" in net params
        net_params.additional_params["length"] = self.left_len + self.right_len + self.horizontal_junction_len + \
            self.bottom_len + self.top_len + self.vertical_junction_len

        if "horizontal_lanes" not in net_params.additional_params:
            raise ValueError("number of horizontal lanes not supplied")

        if "vertical_lanes" not in net_params.additional_params:
            raise ValueError("number of vertical lanes not supplied")

        self.lanes = {"top": net_params.additional_params["vertical_lanes"],
                      "bottom": net_params.additional_params["vertical_lanes"],
                      "left": net_params.additional_params["horizontal_lanes"],
                      "right": net_params.additional_params["horizontal_lanes"]}

        # enter_lane specifies which lane a car enters given a certain direction
        self.enter_lane = {"horizontal": "left", "vertical": "bottom"}

        if "speed_limit" not in net_params.additional_params:
            raise ValueError("speed limit not supplied")

        # if the speed limit is a single number, then all lanes have the same speed limit
        if isinstance(net_params.additional_params["speed_limit"], int) or \
                isinstance(net_params.additional_params["speed_limit"], float):
            self.speed_limit = {"horizontal": net_params.additional_params["speed_limit"],
                                "vertical": net_params.additional_params["speed_limit"]}
        # if the speed limit is a dict with separate values for vertical and horizontal,
        # then they are set as such
        elif "vertical" in net_params.additional_params["speed_limit"] and \
                        "horizontal" in net_params.additional_params["speed_limit"]:
            self.speed_limit = {"horizontal": net_params.additional_params["speed_limit"]["horizontal"],
                                "vertical": net_params.additional_params["speed_limit"]["vertical"]}
        else:
            raise ValueError('speed limit must contain a number or a dict with keys: "vertical" and "horizontal"')

        super().__init__(name, generator_class, vehicles, net_params, initial_config)

    def specify_edge_starts(self):
        edgestarts = \
            [("bottom", 0),
             ("top", self.bottom_len + self.vertical_junction_len),
             ("left", (self.bottom_len + self.vertical_junction_len + self.top_len)),
             ("altleft1", (self.bottom_len + self.vertical_junction_len + self.top_len) + 20),
             ("right", (self.bottom_len + self.vertical_junction_len + self.top_len) +
              self.left_len + self.horizontal_junction_len),
             ("newright", (self.bottom_len + self.vertical_junction_len + self.top_len)),
             ("newleft", (self.bottom_len + self.vertical_junction_len + self.top_len) +
              self.right_len + self.horizontal_junction_len),
             ("newbottom", 0),
             ("altbottom1" , 20),
             ("alttop1", 1500 + 20),
             ("altright1", 1500 + 20),
             ("newtop", self.bottom_len + self.vertical_junction_len)
             ]
        return edgestarts

    def specify_intersection_edge_starts(self):
        intersection_edgestarts = \
            [
                (":center_%s" % (1 + self.lanes["left"]), self.bottom_len),
                (":center_1", (self.bottom_len + self.vertical_junction_len + self.top_len) + self.left_len),
                (":center_12", (self.bottom_len + self.vertical_junction_len + self.top_len) + self.left_len)
             ]
        return intersection_edgestarts

    def specify_internal_edge_starts(self):
        internal_edgestarts = \
            [   (":center_1", (self.bottom_len + self.vertical_junction_len + self.top_len) + self.left_len),
                (":center_8", self.bottom_len + self.vertical_junction_len+ self.top_len),
                (":center_9", (self.bottom_len + self.vertical_junction_len)) ,
                (":center_5", 408),
                (":center_7", 407),
                (":center_20", 397),
                (":center_22", 396),
                (":center_22", 400),
                (":center_23", 392),
                (":center_24", 402),
                (":center_25", 391),
                (":center_27", 399),
                (":center_28", 393),
                (":center_29", 401),
                (":center_34", 398),
                (":center_37", 396),
                (":center_13", (self.left_len + self.horizontal_junction_len + self.right_len )+ self.top_len),
                (":altleft_0", (self.bottom_len + self.vertical_junction_len + self.top_len) + 20),
                (":altright_0", 1500 + 20),
                (":alttop_0", 1500 + 20),
                (":altbottom_0", 20)
            ]
        return internal_edgestarts

    def gen_custom_start_pos(self, initial_config, **kwargs):
        """
        Generate random positions starting from the ends of the track.
        Vehicles are spaced so that no car can arrive at the
        control portion of the track more often than...
        :return: list of start positions [(edge0, pos0), (edge1, pos1), ...]
        """

        rate = initial_config.additional_params["intensity"]
        v_enter = initial_config.additional_params["enter_speed"]

        start_positions = []
        start_lanes = []
        startedge_lengths = {"left": 20, "altleft1": 1480, "bottom": 20, "altbottom1": 1480, "newright" : 20 , "altright1" : 1480, "newtop" : 20 , "alttop1" : 1480}
        #startedge_lengths = {"left": 20, "altleft1": 1480, "bottom": 20, "altbottom1": 1480}
        x = dict([(e, {0: 0, 1: 0}) for e in startedge_lengths.keys()])

        # Fix it so processes in both lanes are poisson with the right
        # intensity, rather than half the intensity
        while len(start_positions) < self.vehicles.num_vehicles:
            lane_ix = np.random.randint(2)
            d_inc = v_enter * random.expovariate(1.0 / rate)
            # FIXME to get length of car that has been placed already
            # This should be the car length, other values are to make problem
            # easier
            if d_inc > 10:
                start_edge_id = random.sample(list(startedge_lengths.keys()), 1)[0]
                start_pos = x[start_edge_id][lane_ix] + d_inc
                edge_length = startedge_lengths[start_edge_id]
                if start_pos < edge_length:
                    x[start_edge_id][lane_ix] = start_pos
                    start_positions.append((start_edge_id, start_pos))
                    start_lanes.append(lane_ix)
        return start_positions, start_lanes
