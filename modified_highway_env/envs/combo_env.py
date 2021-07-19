import numpy as np
from gym.envs.registration import register

from modified_highway_env import utils
from modified_highway_env.envs.common.abstract import AbstractEnv
from modified_highway_env.envs.common.action import Action
from modified_highway_env.road.road import Road, RoadNetwork
from modified_highway_env.utils import near_split
from modified_highway_env.vehicle.controller import ControlledVehicle
from modified_highway_env.road.lane import LineType, StraightLane, SineLane
from modified_highway_env.vehicle.objects import Obstacle



class ComboEnv(AbstractEnv):
    """
    A highway driving environment.

    The vehicle is driving on a straight highway with several lanes, and is rewarded for reaching a high speed,
    staying on the rightmost lanes and avoiding collisions.
    """

    @classmethod
    def default_config(cls) -> dict:
        cfg = super().default_config()
        cfg.update({
            "collision_reward": -1,
            "right_lane_reward": 0.1,
            "high_speed_reward": 0.2,
            "merging_speed_reward": -0.5,
            "lane_change_reward": -0.05,
        })
        return cfg
        # config = super().default_config()
        # config.update({
        #     "observation": {
        #         "type": "Kinematics"
        #     },
        #     "action": {
        #         "type": "DiscreteMetaAction",
        #     },
        #     "lanes_count": 4,
        #     "vehicles_count": 50,
        #     "controlled_vehicles": 1,
        #     "initial_lane_id": None,
        #     "duration": 40,  # [s]
        #     "ego_spacing": 2,
        #     "vehicles_density": 1,
        #     "collision_reward": -1,    # The reward received when colliding with a vehicle.
        #     "right_lane_reward": 0.1,  # The reward received when driving on the right-most lanes, linearly mapped to
        #                                # zero for other lanes.
        #     "high_speed_reward": 0.2,  # The reward received when driving at full speed, linearly mapped to zero for
        #                                # lower speeds according to config["reward_speed_range"].
        #     "lane_change_reward": -0.05,   # The reward received at each lane change action.
        #     "merging_speed_reward": -0.5,
        #     "reward_speed_range": [20, 30],
        #     "offroad_terminal": False
        # })
        # return config

    def _reward(self, action: int) -> float:
        """
        The vehicle is rewarded for driving with high speed on lanes to the right and avoiding collisions

        But an additional altruistic penalty is also suffered if any vehicle on the merging lane has a low speed.

        :param action: the action performed
        :return: the reward of the state-action transition
        """
        action_reward = {0: self.config["lane_change_reward"],
                         1: 0,
                         2: self.config["lane_change_reward"],
                         3: 0,
                         4: 0}
        reward = self.config["collision_reward"] * self.vehicle.crashed \
            + self.config["right_lane_reward"] * self.vehicle.lane_index[2] / 1 \
            + self.config["high_speed_reward"] * self.vehicle.speed_index / (self.vehicle.SPEED_COUNT - 1)

        # Altruistic penalty
        for vehicle in self.road.vehicles:
            if vehicle.lane_index == ("b", "c", 2) and isinstance(vehicle, ControlledVehicle):
                reward += self.config["merging_speed_reward"] * \
                          (vehicle.target_speed - vehicle.speed) / vehicle.target_speed

        return utils.lmap(action_reward[action] + reward,
                          [self.config["collision_reward"] + self.config["merging_speed_reward"],
                           self.config["high_speed_reward"] + self.config["right_lane_reward"]],
                          [0, 1])

    def _is_terminal(self) -> bool:
        """The episode is over when a collision occurs or when the access ramp has been passed."""
        return self.vehicle.crashed or self.vehicle.position[0] > 720

    def _reset(self) -> None:
        self._make_road()
        self._make_vehicles()

    def _make_road(self) -> None:
        """
        Make a road composed of a straight highway and a merging lane.

        :return: the road
        """
        net = RoadNetwork()

        # Highway lanes
        ends = [150, 80, 80, 150, 80, 150, 80, 150]  # Before, converging, merge, after, converging, merge, after
        c, s, n = LineType.CONTINUOUS_LINE, LineType.STRIPED, LineType.NONE
        y = [0, StraightLane.DEFAULT_WIDTH, StraightLane.DEFAULT_WIDTH*2]
        line_type = [[c, s], [n, c]]
        line_type_merge_1 = [[c, s], [n, s]]
        line_type_merge_2 = [[c, s], [n, s]]

        for i in range(2):
            net.add_lane("a", "b", StraightLane([0, y[i]], [sum(ends[:2]), y[i]], line_types=line_type[i]))
            net.add_lane("b", "c", StraightLane([sum(ends[:2]), y[i]], [sum(ends[:3]), y[i]], line_types=line_type_merge_1[i]))
            net.add_lane("c", "d", StraightLane([sum(ends[:3]), y[i]], [sum(ends[:5]), y[i]], line_types=line_type[i]))
            net.add_lane("d", "e", StraightLane([sum(ends[:5]), y[i]], [sum(ends[:6]), y[i]], line_types=line_type_merge_1[i]))
            net.add_lane("e", "f", StraightLane([sum(ends[:6]), y[i]], [sum(ends[:7]), y[i]], line_types=line_type[i]))

        # Merging lane
        amplitude = 3.25
        ljk = StraightLane([0, 6.5 + 4 + 4], [ends[0], 6.5 + 4 + 4], line_types=[c, c], forbidden=True)
        lkb = SineLane(ljk.position(ends[0], -amplitude), ljk.position(sum(ends[:2]), -amplitude),
                       amplitude, 2 * np.pi / (2*ends[1]), np.pi / 2, line_types=[c, c], forbidden=True)
        lbc = StraightLane(lkb.position(ends[1], 0), lkb.position(ends[1], 0) + [ends[2], 0],
                           line_types=[n, c], forbidden=True)
        net.add_lane("j", "k", ljk)
        net.add_lane("k", "b", lkb)
        net.add_lane("b", "c", lbc)

        # Merging Lane 2
        amplitude = 3.25
        lmn = StraightLane([sum(ends[:3]), 6.5 + 4 + 4], [sum(ends[:4]), 6.5 + 4 + 4], line_types=[c, c], forbidden=True)
        lnd = SineLane(lmn.position(sum(ends[:3]), -amplitude), lmn.position(sum(ends[:5]), -amplitude),
                       amplitude, 2 * np.pi / (2*ends[1]), np.pi / 2, line_types=[c, c], forbidden=True)
        lde = StraightLane(lnd.position(sum(ends[:4]), 0), lnd.position(sum(ends[:4]), 0) + [sum(ends[:6]), 0],
                           line_types=[n, c], forbidden=True)
        net.add_lane("m", "n", lmn)
        net.add_lane("n", "d", lnd)
        net.add_lane("d", "e", lde)

        road = Road(network=net, np_random=self.np_random, record_history=self.config["show_trajectories"])
        road.objects.append(Obstacle(road, lbc.position(ends[2], 0)))
        self.road = road

    def _make_vehicles(self) -> None:
        """
        Populate a road with several vehicles on the highway and on the merging lane, as well as an ego-vehicle.

        :return: the ego-vehicle
        """
        road = self.road
        ego_vehicle = self.action_type.vehicle_class(road,
                                                     road.network.get_lane(("a", "b", 1)).position(30, 0),
                                                     speed=30)
        road.vehicles.append(ego_vehicle)

        other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
        road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 0)).position(90, 0), speed=29))
        road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 1)).position(70, 0), speed=31))
        road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 0)).position(5, 0), speed=31.5))

        merging_v = other_vehicles_type(road, road.network.get_lane(("j", "k", 0)).position(110, 0), speed=20)
        merging_v.target_speed = 30
        road.vehicles.append(merging_v)
        self.vehicle = ego_vehicle


# register(
#     id='combo-v0',
#     entry_point='highway_env.envs:ComboEnv',
# )
