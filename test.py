from mlagents_envs.environment import UnityEnvironment
env = UnityEnvironment(file_name="BuildTestEnv")

env.reset()
behavior_name = list(env.behavior_specs)[0]
spec = env.behavior_specs[behavior_name]
decision_steps, terminal_steps = env.get_steps(behavior_name)
env.close()


