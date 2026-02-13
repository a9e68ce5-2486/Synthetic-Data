# agents/car_agent.py
import config
from agents.base_agent import BaseAgent


class CarAgent(BaseAgent):
    def __init__(self, aid, start, env):
        super().__init__(aid, start, env, mode="drive")

    def step(self, goal):
        self.move_along_path(goal, config.EVAC_SPEED_CAR)
