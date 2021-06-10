# Hide pygame support prompt
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
# Import the envs module so that envs register themselves
# import highway_env.envs
from gym.envs.registration import register

register(
    id='combo-v0',
    entry_point='modified_highway_env.envs:ComboEnv',
)
register(
    id='exit-v0',
    entry_point='modified_highway_env.envs:ExitEnv',
)
register(
    id='highway-v0',
    entry_point='modified_highway_env.envs:HighwayEnv',
)
register(
    id='intersection-v0',
    entry_point='modified_highway_env.envs:IntersectionEnv',
)
register(
    id='intersection-multi-agent-v0',
    entry_point='modified_highway_env.envs:MultiAgentIntersectionEnv',
)
register(
    id='intersection-multi-agent-v1',
    entry_point='modified_highway_env.envs:TupleMultiAgentIntersectionEnv',
)
register(
    id='lane-keeping-v0',
    entry_point='modified_highway_env.envs:LaneKeepingEnv',
    max_episode_steps=200
)
register(
    id='merge-v0',
    entry_point='modified_highway_env.envs:MergeEnv',
)
register(
    id='parking-v0',
    entry_point='modified_highway_env.envs:ParkingEnv',
)
register(
    id='parking-ActionRepeat-v0',
    entry_point='modified_highway_env.envs:ParkingEnvActionRepeat'
)
register(
    id='roundabout-v0',
    entry_point='modified_highway_env.envs:RoundaboutEnv',
)
register(
    id='summon-v0',
    entry_point='modified_highway_env.envs:SummonEnv',
    max_episode_steps=100
)
register(
    id='summon-ActionRepeat-v0',
    entry_point='modified_highway_env.envs:SummonEnvActionRepeat',
    max_episode_steps=20
)
register(
    id='two-way-v0',
    entry_point='modified_highway_env.envs:TwoWayEnv',
    max_episode_steps=15
)
register(
    id='u-turn-v0',
    entry_point='modified_highway_env.envs:UTurnEnv'
)

