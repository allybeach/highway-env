import gym
import modified_highway_env

env = gym.make('modified_highway_env:combo-v0')
obs = env.reset()
obs, reward, done, info = env.step(env.action_space.sample())
print(info)