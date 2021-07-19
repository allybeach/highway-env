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
        config = super().default_config()
        config.update({
            "observation": {
                "type": "Kinematics"
            },
            "action": {
                "type": "DiscreteMetaAction",
            },
            "lanes_count": 4,
            "vehicles_count": 50,
            "controlled_vehicles": 1,
            "initial_lane_id": None,
            "duration": 40,  # [s]
            "ego_spacing": 2,
            "vehicles_density": 1,
            "collision_reward": -1,    # The reward received when colliding with a vehicle.
            "right_lane_reward": 0.1,  # The reward received when driving on the right-most lanes, linearly mapped to
                                       # zero for other lanes.
            "high_speed_reward": 0.2,  # The reward received when driving at full speed, linearly mapped to zero for
                                       # lower speeds according to config["reward_speed_range"].
            "lane_change_reward": -0.05,   # The reward received at each lane change action.
            "merging_speed_reward": -0.5,
            "reward_speed_range": [20, 30],
            "offroad_terminal": False
        })
        return config

    def _reset(self) -> None:
        self._create_road()
        self._create_vehicles()

    # def _create_road(self) -> None:
    #     """Create a road composed of straight adjacent lanes."""
    #     self.road = Road(network=RoadNetwork.straight_road_network(self.config["lanes_count"], speed_limit=30),
    #                      np_random=self.np_random, record_history=self.config["show_trajectories"])
    
    def _create_road(self) -> None:
        # """
        # Make a road composed of a straight highway and a merging lane.

        # :return: the road
        # """
        # net = RoadNetwork()

        # # Highway lanes
        # ends = [150, 80, 80, 150]  # Before, converging, merge, after
        # c, s, n = LineType.CONTINUOUS_LINE, LineType.STRIPED, LineType.NONE
        # y = [0, StraightLane.DEFAULT_WIDTH]
        # line_type = [[c, s], [n, c]]
        # line_type_merge = [[c, s], [n, s]]
        # for i in range(3):
        #     # lane that is merging away 
        #     net.add_lane("a", "b", StraightLane([0, y[i]], [sum(ends[:2]), y[i]], line_types=line_type[i]))
        #     # lane that is getting merged into 
        #     net.add_lane("b", "c", StraightLane([sum(ends[:2]), y[i]], [sum(ends[:3]), y[i]], line_types=line_type_merge[i]))
            
        #     net.add_lane("c", "d", StraightLane([sum(ends[:3]), y[i]], [sum(ends), y[i]], line_types=line_type[i]))
        #     # new merging lane 
        #     net.add_lane("d", "e", StraightLane([sum(ends[:3]), y[0]], [sum(ends), y[i]], line_types=line_type[i]))


        # # Merging lane - first one that merges into highway 
        # amplitude = 3.25
        # ljk = StraightLane([0, 6.5 + 4 + 4], [ends[0], 6.5 + 4 + 4], line_types=[c, c], forbidden=True)
        # lkb = SineLane(ljk.position(ends[0], -amplitude), ljk.position(sum(ends[:2]), -amplitude),
        #                amplitude, 2 * np.pi / (2*ends[1]), np.pi / 2, line_types=[c, c], forbidden=True)
        # lbc = StraightLane(lkb.position(ends[1], 0), lkb.position(ends[1], 0) + [ends[2], 0],
        #                    line_types=[n, c], forbidden=True)
        
        # net.add_lane("j", "k", ljk)
        # net.add_lane("k", "b", lkb)
        # net.add_lane("b", "c", lbc)

        # # Merging lane - second one that merges away from highway 
        # amplitude = 3.25
        # lab = StraightLane([0, 6.5 + 4 + 4], [ends[0], 6.5 + 4 + 4], line_types=[c, c], forbidden=True)
        # lbp = SineLane(ljk.position(ends[0], -amplitude), ljk.position(sum(ends[:2]), -amplitude),
        #                amplitude, 2 * np.pi / (2*ends[1]), np.pi / 2, line_types=[c, c], forbidden=True)
        # lpq = StraightLane(lkb.position(ends[1], 0), lkb.position(ends[1], 0) + [ends[2], 0],
        #                    line_types=[n, c], forbidden=True)

        # net.add_lane("a", "b", ljk)
        # net.add_lane("b", "p", lkb)
        # net.add_lane("p", "q", lbc)

        # road = Road(network=net, np_random=self.np_random, record_history=self.config["show_trajectories"])
        # road.objects.append(Obstacle(road, lbc.position(ends[2], 0)))
        # self.road = road

        """
        Make a road composed of a straight highway and a merging lane.

        :return: the road
        """
        net = RoadNetwork()

        # Highway lanes
        ends = [150, 80, 80, 150]  # Before, converging, merge, after
        c, s, n = LineType.CONTINUOUS_LINE, LineType.STRIPED, LineType.NONE
        y = [0, StraightLane.DEFAULT_WIDTH]
        line_type = [[c, s], [n, c]]
        line_type_merge = [[c, s], [n, s]]
        for i in range(2):
            net.add_lane("a", "b", StraightLane([0, y[i]], [sum(ends[:2]), y[i]], line_types=line_type[i]))
            net.add_lane("b", "c", StraightLane([sum(ends[:2]), y[i]], [sum(ends[:3]), y[i]], line_types=line_type_merge[i]))
            net.add_lane("c", "d", StraightLane([sum(ends[:3]), y[i]], [sum(ends), y[i]], line_types=line_type[i]))

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
        road = Road(network=net, np_random=self.np_random, record_history=self.config["show_trajectories"])
        road.objects.append(Obstacle(road, lbc.position(ends[2], 0)))
        self.road = road
        

    def _create_vehicles(self) -> None:
        """Create some new random vehicles of a given type, and add them on the road."""
        other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
        other_per_controlled = near_split(self.config["vehicles_count"], num_bins=self.config["controlled_vehicles"])

        self.controlled_vehicles = []
        for others in other_per_controlled:
            controlled_vehicle = self.action_type.vehicle_class.create_random(
                self.road,
                speed=25,
                lane_id=self.config["initial_lane_id"],
                spacing=self.config["ego_spacing"]
            )
            self.controlled_vehicles.append(controlled_vehicle)
            self.road.vehicles.append(controlled_vehicle)

            for _ in range(others):
                self.road.vehicles.append(
                    other_vehicles_type.create_random(self.road, spacing=1 / self.config["vehicles_density"])
                )

    def _reward(self, action: Action) -> float:
        """
        The reward is defined to foster driving at high speed, on the rightmost lanes, and to avoid collisions.
        :param action: the last action performed
        :return: the corresponding reward
        """
        neighbours = self.road.network.all_side_lanes(self.vehicle.lane_index)
        lane = self.vehicle.target_lane_index[2] if isinstance(self.vehicle, ControlledVehicle) \
            else self.vehicle.lane_index[2]
        scaled_speed = utils.lmap(self.vehicle.speed, self.config["reward_speed_range"], [0, 1])
        reward = \
            + self.config["collision_reward"] * self.vehicle.crashed \
            + self.config["right_lane_reward"] * lane / max(len(neighbours) - 1, 1) \
            + self.config["high_speed_reward"] * np.clip(scaled_speed, 0, 1)

        # Altruistic penalty
        for vehicle in self.road.vehicles:
            if vehicle.lane_index == ("b", "c", 2) and isinstance(vehicle, ControlledVehicle):
                reward += self.config["merging_speed_reward"] * \
                          (vehicle.target_speed - vehicle.speed) / vehicle.target_speed

        # TODO: watch out for the action_reward for lane changing - took out the action_reward
        reward = utils.lmap(reward,
                          [self.config["collision_reward"],
                           self.config["high_speed_reward"] + self.config["right_lane_reward"]],
                          [0, 1])
        reward = 0 if not self.vehicle.on_road else reward
        return reward


    def _is_terminal(self) -> bool:
        """The episode is over if the ego vehicle crashed or the time is out."""
        return self.vehicle.crashed or \
            self.steps >= self.config["duration"] or \
            (self.config["offroad_terminal"] and not self.vehicle.on_road)

    def _cost(self, action: int) -> float:
        """The cost signal is the occurrence of collision."""
        return float(self.vehicle.crashed)


# register(
#     id='combo-v0',
#     entry_point='highway_env.envs:ComboEnv',
# )
