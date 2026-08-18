from sim.map import Map
from sim.simulator import Simulator
from .planner import PathPlanner


class AStar(PathPlanner):
    def __init__(self, robo_map: Map, sim: Simulator) -> None:
        super().__init__(robo_map, sim)
