from planner.planner import PathPlanner
from sim.simulator import Simulator


# 这是一个使用nav_cbk的示例
def nav_cbk(sim: Simulator) -> None:
    """导航回调：自己读取目标点，并把计算出的路径交给 Simulator。"""
    goal = sim.get_goal()
    if goal is None:
        sim.set_display_path([])
        sim.set_control(0.0, 0.0)
        return

    planner = PathPlanner(sim.map, sim)
    planner.set_goal(goal)
    path = planner.plan()
    sim.set_display_path(path)

    # 这里根据 path、机器人位姿和反馈计算下一周期控制量。
    speed, omega = follow_path(sim, path)
    sim.set_control(speed, omega)


def follow_path(sim: Simulator, path: list[tuple[float, float]]) -> tuple[float, float]:
    """根据路径计算下一周期的线速度和角速度。"""
    raise NotImplementedError("implement path following here")
