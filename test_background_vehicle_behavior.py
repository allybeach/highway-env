import gym
import pprint
from matplotlib import pyplot as plt
import modified_highway_env

env = gym.make("highway-v0")

# polite background vehicles 
# iterates over different vehicle densities configurations 
# for i in [float(j) / 10 for j in range(1, 26, 5)]:
#     env.config["vehicles_density"] = i
#     pprint.pprint(env.config)
#     env.reset()
#     for _ in range(5):
#         action = env.action_type.actions_indexes["IDLE"]
#         obs, reward, done, info = env.step(action)
#         env.render()

#     plt.imshow(env.render(mode="rgb_array"))
#     plt.show(block=False)
#     plt.pause(1)
#     figure_name = "density_" + str(i) + ".png"
#     # plt.savefig(figure_name)
#     plt.close()

# env.config["other_vehicles_type"] = "modified_highway_env.vehicle.behavior.IDMVehicle"
# env.config["vehicles_density"] = 1
# env.reset()
# for _ in range(5):
#     action = env.action_type.actions_indexes["IDLE"]
#     obs, reward, done, info = env.step(action)
#     env.render()

# plt.imshow(env.render(mode="rgb_array"))
# plt.show(block=False)
# plt.pause(1)
# figure_name = "aggressive_vehicle.png"
# # plt.savefig(figure_name)
# plt.close()


# env.config["other_vehicles_type"] = "modified_highway_env.vehicle.behavior.LinearVehicle"
# env.config["vehicles_density"] = 1
# env.reset()
# for _ in range(5):
#     action = env.action_type.actions_indexes["IDLE"]
#     obs, reward, done, info = env.step(action)
#     env.render()

# plt.imshow(env.render(mode="rgb_array"))
# plt.show(block=False)
# plt.pause(1)
# figure_name = "aggressive_vehicle.png"
# # plt.savefig(figure_name)
# plt.close()

# env.config["other_vehicles_type"] = "modified_highway_env.vehicle.behavior.AggressiveVehicle"
# env.config["vehicles_density"] = 1
# env.reset()
# for _ in range(5):
#     action = env.action_type.actions_indexes["IDLE"]
#     obs, reward, done, info = env.step(action)
#     env.render()

# plt.imshow(env.render(mode="rgb_array"))
# plt.show(block=False)
# plt.pause(1)
# figure_name = "aggressive_vehicle.png"
# # plt.savefig(figure_name)
# plt.close()
    
# env.config["other_vehicles_type"] = "modified_highway_env.vehicle.behavior.DefensiveVehicle"
# env.config["vehicles_density"] = 1
# env.reset()
# for _ in range(5):
#     action = env.action_type.actions_indexes["IDLE"]
#     obs, reward, done, info = env.step(action)
#     env.render()

# plt.imshow(env.render(mode="rgb_array"))
# plt.show(block=False)
# plt.pause(1)
# figure_name = "aggressive_vehicle.png"
# # plt.savefig(figure_name)
# plt.close()

env.config["other_vehicles_type"] = "modified_highway_env.vehicle.behavior.DevAggressiveVehicle"
env.config["vehicles_density"] = 1
pprint.pprint(env.config)
env.reset()
for _ in range(5):
    action = env.action_type.actions_indexes["IDLE"]
    obs, reward, done, info = env.step(action)
    env.render()

plt.imshow(env.render(mode="rgb_array"))
plt.show(block=False)
plt.pause(1)
figure_name = "aggressive_vehicle.png"
# plt.savefig(figure_name)
plt.close()

env.config["other_vehicles_type"] = "modified_highway_env.vehicle.behavior.DevPoliteVehicle"
env.config["vehicles_density"] = 1
pprint.pprint(env.config)
env.reset()
for _ in range(5):
    action = env.action_type.actions_indexes["IDLE"]
    obs, reward, done, info = env.step(action)
    env.render()

plt.imshow(env.render(mode="rgb_array"))
plt.show(block=False)
plt.pause(1)
figure_name = "aggressive_vehicle.png"
# plt.savefig(figure_name)
plt.close()
