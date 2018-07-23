from flow.core.generator import Generator

from lxml import etree

E = etree.Element


class MyTwoWayIntersectionGenerator(Generator):
    """
    Generator for two-way intersections. Requires from net_params:
     - horizontal_length_in: length of the horizontal lane before the intersection
     - horizontal_length_out: length of the horizontal lane after the intersection
     - horizontal_lanes: number of lanes in the horizontal lane
     - vertical_length_in: length of the vertical lane before the intersection
     - vertical_length_out: length of the vertical lane after the intersection
     - vertical_lanes: number of lanes in the vertical lane
     - speed_limit: max speed limit of the vehicles on the road network. May be a single
                    value (for both lanes) or a dict separating the two, of the form:
                    speed_limit = {"horizontal": {float}, "vertical": {float}}
     - no-internal-links: set to False to receive queueing at intersections.
    """

    def __init__(self, net_params, base):
        """
        See parent class
        """
        super().__init__(net_params, base)

        horizontal_length_in = net_params.additional_params["horizontal_length_in"]
        horizontal_length_out = net_params.additional_params["horizontal_length_out"]
        horizontal_lanes = net_params.additional_params["horizontal_lanes"]
        vertical_length_in = net_params.additional_params["vertical_length_in"]
        vertical_length_out = net_params.additional_params["vertical_length_out"]
        vertical_lanes = net_params.additional_params["vertical_lanes"]

        self.name = "%s-horizontal-%dm%dl-vertical-%dm%dl" % \
                    (base, horizontal_length_in + horizontal_length_out, horizontal_lanes,
                     vertical_length_in + vertical_length_out, vertical_lanes)

    def specify_nodes(self, net_params):
        """
        See parent class
        """
        horz_length_in = net_params.additional_params["horizontal_length_in"]
        horz_length_out = net_params.additional_params["horizontal_length_out"]
        vert_length_in = net_params.additional_params["vertical_length_in"]
        vert_length_out = net_params.additional_params["vertical_length_out"]


        nodes = [{"id": "center", "x": repr(0),               "y": repr(0),               "type": "traffic_light"},
                 {"id": "bottom", "x": repr(0),          "y": repr(-vert_length_in), "type": "priority"},
                 {"id": "altbottom", "x": repr(0), "y": repr(-vert_length_in+20), "type": "priority"},
                 {"id": "top",    "x": repr(0),               "y": repr(vert_length_out), "type": "priority"},
                 {"id": "alttop", "x": repr(0), "y": repr(vert_length_out-20), "type": "priority"},
                 {"id": "left",   "x": repr(-horz_length_in), "y": repr(0),               "type": "priority"},
                 {"id": "altleft", "x": repr(-horz_length_in+ 20), "y": repr(0), "type": "priority"},
                 {"id": "right",  "x": repr(horz_length_out), "y": repr(0),               "type": "priority"},
                 {"id": "altright", "x": repr(horz_length_out-20), "y": repr(0), "type": "priority"}]


        return nodes

    def specify_edges(self, net_params):
        """
        See parent class
        """
        horz_length_in = net_params.additional_params["horizontal_length_in"]
        horz_length_out = net_params.additional_params["horizontal_length_out"]
        vert_length_in = net_params.additional_params["vertical_length_in"]
        vert_length_out = net_params.additional_params["vertical_length_out"]

        alt_length = 20

        edges = [{"id": "left", "type": "horizontal", "priority": "78",
                  "from": "left", "to": "altleft", "length": repr(alt_length)},
                 {"id": "altleft1", "type": "horizontal", "priority": "78",
                  "from": "altleft", "to": "center", "length": repr(horz_length_in - alt_length)},
                 {"id": "newleft", "type": "horizontal", "priority": "78",
                  "from": "center", "to": "left", "length": repr(horz_length_in)},

                 {"id": "right", "type": "horizontal", "priority": "78",
                  "from": "center", "to": "right", "length": repr(horz_length_out)},
                 {"id": "newright", "type": "horizontal", "priority": "78",
                  "from": "right", "to": "altright", "length": repr(alt_length)},
                 {"id": "altright1", "type": "horizontal", "priority": "78",
                  "from": "altright", "to": "center", "length": repr(horz_length_out- alt_length)},


                 {"id": "bottom", "type": "vertical", "priority": "78",
                  "from": "bottom", "to": "altbottom", "length": repr(alt_length)},
                 {"id": "altbottom1", "type": "vertical", "priority": "78",
                  "from": "altbottom", "to": "center", "length": repr(vert_length_in - alt_length)},
                 {"id": "newbottom", "type": "vertical", "priority": "78",
                  "from": "center", "to": "bottom", "length": repr(vert_length_in)},



                 {"id": "top", "type": "vertical", "priority": "78",
                  "from": "center", "to": "top", "length": repr(vert_length_out)},
                 {"id": "newtop", "type": "vertical", "priority": "78",
                  "from": "top", "to": "alttop", "length": repr(alt_length)},
                 {"id": "alttop1", "type": "vertical", "priority": "78",
                  "from": "alttop", "to": "center", "length": repr(vert_length_out -alt_length)}

                 ]

        return edges

    def specify_types(self, net_params):
        """
        See parent class
        """
        horizontal_lanes = net_params.additional_params["horizontal_lanes"]
        vertical_lanes = net_params.additional_params["vertical_lanes"]
        if isinstance(net_params.additional_params["speed_limit"], int) or \
                isinstance(net_params.additional_params["speed_limit"], float):
            speed_limit = {"horizontal": net_params.additional_params["speed_limit"],
                           "vertical": net_params.additional_params["speed_limit"]}
        else:
            speed_limit = net_params.additional_params["speed_limit"]

        types = [{"id": "horizontal", "numLanes": repr(horizontal_lanes), "speed": repr(speed_limit["horizontal"])},
                 {"id": "vertical", "numLanes": repr(vertical_lanes), "speed": repr(speed_limit["vertical"])}]

        return types

    def specify_routes(self, net_params):
        """
        See parent class
        """
        #rts = {"left": ["left", "right"], "bottom": ["bottom", "top"], "altbottom1": ["altbottom1", "newleft"], "newright": ["newright", "newleft"], "newtop": ["newtop", "newbottom"], "alttop1": ["alttop1", "right"], "altleft1": ["altleft1", "top"],  "altright1": ["altright1", "newbottom"]}
        rts = {"left": ["left","altleft1", "right"], "altleft1": ["altleft1", "top"], "bottom": ["bottom","altbottom1","top"], "altbottom1": ["altbottom1","newleft"], "newright": ["newright","altright1","newleft"], "altright1": ["altright1","newbottom"], "newtop": ["newtop","alttop1","newbottom"], "alttop1": ["alttop1","right"]}
        #rts = {"left": ["left","altleft1", "right"], "altleft1": ["altleft1", "right"], "bottom": ["bottom","altbottom1","top"], "altbottom1": ["altbottom1","top"], "newright": ["newright","altright1","newleft"], "altright1": ["altright1","newleft"], "newtop": ["newtop","alttop1","newbottom"], "alttop1": ["alttop1","newbottom"]}

        return rts
