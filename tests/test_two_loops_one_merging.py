import unittest
import numpy as np
from numpy import pi

from flow.core.experiment import SumoExperiment
from flow.core.params import SumoParams, EnvParams, InitialConfig, NetParams
from flow.core.vehicles import Vehicles
from flow.controllers.car_following_models import *
from flow.controllers.lane_change_controllers import StaticLaneChanger
from flow.scenarios.two_loops_one_merging.gen import TwoLoopOneMergingGenerator
from flow.scenarios.two_loops_one_merging.two_loops_one_merging_scenario \
    import TwoLoopsOneMergingScenario
from flow.envs.two_loops_one_merging import TwoLoopsOneMergingEnvironment


def two_loops_one_merging_exp_setup(vehicles=None):
    sumo_params = SumoParams(time_step=0.1,
                             human_speed_mode="no_collide",
                             sumo_binary="sumo")

    if vehicles is None:
        vehicles = Vehicles()
        vehicles.add_vehicles(veh_id="idm",
                              acceleration_controller=(IDMController, {}),
                              lane_change_controller=(StaticLaneChanger, {}),
                              num_vehicles=5)
        vehicles.add_vehicles(veh_id="merge-idm",
                              acceleration_controller=(IDMController, {}),
                              lane_change_controller=(StaticLaneChanger, {}),
                              num_vehicles=5)

    additional_env_params = {"target_velocity": 8, "max-deacc": -6,
                             "max-acc": 3}
    env_params = EnvParams(additional_params=additional_env_params)

    additional_net_params = {"ring_radius": 230 / (2 * np.pi), "lanes": 1,
                             "speed_limit": 30, "resolution": 40}
    net_params = NetParams(
        no_internal_links=False,
        additional_params=additional_net_params
    )

    initial_config = InitialConfig(spacing="custom",
                                   additional_params={"merge_bunching": 0})

    scenario = TwoLoopsOneMergingScenario(
        "loop-merges", TwoLoopOneMergingGenerator, vehicles, net_params,
        initial_config=initial_config)

    env = TwoLoopsOneMergingEnvironment(env_params, sumo_params, scenario)

    return env, scenario


class TestLoopMerges(unittest.TestCase):
    """
    Tests the loop_merges generator, scenario, and environment.
    """
    def setUp(self):
        # create the environment and scenario classes for a ring road
        self.env, scenario = two_loops_one_merging_exp_setup()

        # instantiate an experiment class
        self.exp = SumoExperiment(self.env, scenario)

    def tearDown(self):
        # terminate the traci instance
        self.env.terminate()

        # free up used memory
        self.env = None
        self.exp = None

    def test_it_runs(self):
        """
        Tests that the loop merges experiment runs, and vehicles do not exit
        the network.
        """
        self.exp.run(1, 10)

    def test_gen_custom_start_pos(self):
        """
        Tests that vehicle with the prefix "merge" are in the merge_in lane, and
        all other vehicles are in the ring road.
        """
        # reset the environment to ensure all vehicles are at their starting
        # positions
        self.env.reset()
        ids = self.env.vehicles.get_ids()

        # collect the starting edges of all vehicles
        merge_starting_edges = []
        other_starting_edges = []
        for veh_id in ids:
            if veh_id[:5] == "merge":
                merge_starting_edges.append(self.env.vehicles.get_edge(veh_id))
            else:
                other_starting_edges.append(self.env.vehicles.get_edge(veh_id))

        # ensure that all vehicles are starting in the edges they should be in
        expected_merge_starting_edges = ["right_bottom", "right_top"]

        self.assertTrue(
            np.all([merge_starting_edges[i] in expected_merge_starting_edges
                    for i in range(len(merge_starting_edges))]))

        self.assertTrue(
            np.all([other_starting_edges[i] not in expected_merge_starting_edges
                    for i in range(len(other_starting_edges))]))


if __name__ == '__main__':
    unittest.main()
