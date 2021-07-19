import gym
from matplotlib import pyplot as plt
import modified_highway_env

env = gym.make('modified_highway_env:combo-v0')
env.reset()

for _ in range(5):
    action = env.action_type.actions_indexes["IDLE"]
    obs, reward, done, info = env.step(action)
    env.render()

plt.imshow(env.render(mode="rgb_array"))
plt.show(block=False)
plt.pause(1)
plt.close()
