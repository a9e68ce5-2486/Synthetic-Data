# agents/ped_agent.py
import config
from agents.base_agent import BaseAgent


class PedAgent(BaseAgent):
    def __init__(self, aid, start, env):
        super().__init__(aid, start, env, mode="walk")

    def step(self, goal):
        self.move_along_path(goal, config.EVAC_SPEED_WALK)
