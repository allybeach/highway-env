import gym
import highway_env

env = gym.make('combo-v0')
obs = env.reset()
obs, reward, done, info = env.step(env.action_space.sample())
