# PyRobo 上层导航接口

本文档只说明导航、规划和控制算法需要使用的接口。地图加载、pygame 窗口、噪声和机器人运动由 `Simulator` 内部负责。

## 基本约定

- 位置单位：米。
- 线速度单位：`m/s`。
- 角速度单位：`rad/s`。
- 位姿格式：`(x, y, yaw)`。
- 路径格式：`[(x1, y1), (x2, y2), ...]`。
- 路径和目标点都使用世界坐标，不使用图片像素坐标。

## 导航回调

```python
def nav_cbk(sim: Simulator) -> None:
    ...


sim.run(callback=nav_cbk)
```

每个周期的执行顺序是：

```text
机器人执行上一周期控制量
        ↓
Simulator 更新反馈和位姿
        ↓
调用 nav_cbk(sim)
        ↓
nav_cbk 设置下一周期控制量
```

因此，回调中读取到的是当前周期反馈，设置的控制量在下一周期生效。

## 控制量

```python
sim.set_control(
    speed: float,
omega: float,
robot_id: str = "robot",
) -> None
```

设置期望线速度和角速度。Simulator 内部会加入控制噪声，实际执行速度通过 `get_feedback()` 获取。

```python
sim.set_control(speed=0.2, omega=0.0)
```

## 反馈和位姿

```python
sim.get_feedback(robot_id: str = "robot") -> tuple[float, float]
sim.get_pose(robot_id: str = "robot") -> tuple[float, float, float]
```

```python
feedback_speed, feedback_omega = sim.get_feedback()
x, y, yaw = sim.get_pose()
```

`get_feedback()` 返回最近一个周期实际执行的线速度和角速度；`get_pose()` 返回当前世界位姿。

## 目标点

```python
sim.get_goal() -> tuple[float, float, float] | None
```

用户可以在 pygame 地图中左键点击，或按 `G` 配合鼠标位置设置目标点。目标点只保存在 Simulator 中，不会自动调用规划器。

```python
goal = sim.get_goal()
if goal is None:
    sim.set_control(0.0, 0.0)
    return

goal_x, goal_y, goal_yaw = goal
```

测试或上层程序也可以主动设置、清除目标点：

```python
sim.set_goal((x, y))
sim.clear_goal()
```

## 路径显示

```python
sim.set_display_path(path: Iterable[tuple[float, float]]) -> None
```

向 Simulator 提供要显示的世界坐标路径。路径可以来自任意算法，不要求使用 `PathPlanner`。

```python
path = planner.plan(current_pose=sim.get_pose(), goal=sim.get_goal())
sim.set_display_path(path)
```

清空路径显示：

```python
sim.set_display_path([])
```

## 二维雷达

```python
sim.get_lidar(
    robot_id: str = "robot",
count: int = 360,
max_range: float = 3.0,
fov: float = 2 * math.pi,
) -> list[float]
```

返回距离数组，距离单位为米。第 `i` 项对应的相对角度为：

```python
angle = -fov / 2 + i * fov / count
```

```python
ranges = sim.get_lidar(count=360, max_range=3.0)
front_distance = ranges[len(ranges) // 2]
```

雷达适合用于未知地图、局部避障和在线重规划。当前雷达只读取静态地图，不包含其他机器人。

## 多机器人

```python
robot_id = sim.add_robot(
    name="robot_2",
    pose=(1.0, 0.5, 0.0),
    radius=0.05,
)
```

之后通过 `robot_id` 分别控制和读取：

```python
sim.set_control(0.2, 0.0, robot_id="robot_2")
pose = sim.get_pose(robot_id="robot_2")
feedback = sim.get_feedback(robot_id="robot_2")
```

所有机器人会在同一个 `sim.step()` 中推进，Simulator 会检查机器人之间的圆形碰撞。

## 推荐导航回调

```python
def nav_cbk(sim: Simulator) -> None:
    goal = sim.get_goal()
    pose = sim.get_pose()
    feedback = sim.get_feedback()

    if goal is None:
        sim.set_display_path([])
        sim.set_control(0.0, 0.0)
        return

    # 1. 使用 pose、goal 和地图或雷达数据规划路径
    path = planner.plan(pose, goal)

    # 2. 在 pygame 中显示路径
    sim.set_display_path(path)

    # 3. 根据路径、位姿和反馈计算控制量
    speed, omega = controller.follow(path, pose, feedback)
    sim.set_control(speed, omega)


sim.run(callback=nav_cbk)
```
