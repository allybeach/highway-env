import gym
import modified_highway_env
#import highway_env

env = gym.make('modified_highway_env:combo-v0')
#env = gym.make("highway-v0")

done = False
while not done:
    #action = ... # Your agent code here
    obs, reward, done, info = env.step(env.action_space.sample())
    #obs, reward, done, info = env.step(env.action_space.sample())
    print(info)
    #obs, reward, done, info = env.step(action)
    env.render()

#obs = env.reset()

