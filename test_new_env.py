import gym
import highway_env.envs

env = gym.make('highway-v1')
obs = env.reset()
obs, reward, done, info = env.step(env.action_space.sample())
print(info)