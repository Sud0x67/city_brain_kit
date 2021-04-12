from gym.envs.registration import register

register(
    id = 'CBEngine-v0',
    entry_point = 'CBEngine.envs:CBEngine',
)