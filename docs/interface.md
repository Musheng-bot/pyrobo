# PyRobo 接口说明

除运动学模型题目外，题目代码只需要在 `planner` 包中完成。地图、机器人运动、碰撞检测和 pygame 界面由 `sim` 包提供；运动学模型题目允许修改 `sim` 包。

## 基本约定

- 世界坐标单位是米，角度单位是弧度。
- 位姿格式是 `(x, y, yaw)`。
- 路径格式是 `[(x1, y1), (x2, y2), ...]`。
- 地图数组使用图片索引：`data[row, column]`。
- 世界坐标中 x 向右、y 向上。

## 一、地图查询

导航回调可以通过 `sim.map` 查询已知地图：

```python
world_map = sim.map
```

### 地图数据

```python
world_map.data       # numpy.ndarray，True 表示可通行，False 表示障碍物
world_map.shape      # (height, width)
world_map.resolution # 米/像素
world_map.origin     # (origin_x, origin_y)
```

地图图片默认按以下规则解释：白色区域可通行，黑色区域为障碍物。

### 世界坐标和栅格坐标转换

```python
column, row = world_map.world_to_grid(x, y)
x, y = world_map.grid_to_world(column, row)
```

`world_to_grid()` 返回 `(column, row)`，而 `data` 使用 `[row, column]` 访问。`grid_to_world()` 返回栅格中心的世界坐标。

### 查询可通行区域

```python
free = world_map.is_free(x, y)
```

地图外的位置返回 `False`。

地图接口不提供按机器人半径查询可行位置的方法。第二题需要候选人自己根据 `robot.radius` 对障碍物进行膨胀，并使用膨胀后的地图规划路径。

获取地图实际尺寸：

```python
width_m, height_m = world_map.size_meters
```

## 二、Simulator 中的机器人接口

自动导航由初始化函数和周期函数组成：

```python
context = nav_init(sim)

def nav_run(sim: Simulator, context) -> None:
    ...

sim.run(callback=lambda sim: nav_run(sim, context))
```

每个周期的顺序是：

```text
执行上一周期控制量
    ↓
更新机器人位姿和反馈
    ↓
调用 nav_run(sim)
```

因此，回调中读取到的是当前周期状态，设置的控制量在下一周期生效。

### 获取机器人位姿

```python
x, y, yaw = sim.get_pose()
x, y, yaw = sim.get_pose(robot_id="robot_2")
```

获取机器人对象及其半径：

```python
robot = sim.get_robot()
radius = robot.radius
```

### 获取目标点

```python
goal = sim.get_goal()
```

返回 `(x, y, yaw)`；没有目标点时返回 `None`。

```python
goal = sim.get_goal()
if goal is None:
    sim.set_control(0.0, 0.0)
    return

goal_x, goal_y, goal_yaw = goal
```

目标点可以由 pygame 地图点击设置，也可以由程序设置或清除：

```python
sim.set_goal((x, y))
sim.set_goal((x, y, yaw))
sim.clear_goal()
```

当前目标点接口针对默认机器人。多机器人题目中的其他目标点可以由导航代码自行保存。

### 获取实际反馈速度

```python
first, second = sim.get_feedback()
first, second = sim.get_feedback(robot_id="robot_2")
```

当前默认运动学模型下，返回最近一个周期实际执行的 `(vx, vy)`。反馈包含仿真器加入的噪声以及碰撞限制后的实际结果。

也可以获取包含角速度占位项的统一 `(vx, vy, omega)` 反馈：

```python
vx, vy, omega = sim.get_vector_feedback()
```

### 设置控制量

统一控制接口是：

```python
sim.set_control(a, b)
```

当前默认使用全向运动学模型，两个参数解释为机器人坐标系下的：

```python
sim.set_control(vx, vy)
```

多机器人控制：

```python
sim.set_control(0.2, 0.0, robot_id="robot_2")
```

### 控制量限制

`Simulator` 会读取配置文件中 `pyrobo.control.command_type` 对应的参数组，并在执行前自动限制控制量。

当前 `vx_vy` 配置使用：

```yaml
control:
  command_type: vx_vy
  vx_vy:
    vx_max: 1.5
    vx_min: 0.0
    vy_max: 1.5
    vy_min: 0.0
    acc_max: 1.0
    acc_min: -1.0
```

限制顺序是：加入仿真噪声、限制速度、限制加速度，然后执行运动学模型。`*_min` 表示非零控制量的最小绝对值，零指令仍保持为零。

获取当前控制量：

```python
a, b = sim.get_control()
```

## 三、二维雷达

```python
import math

ranges = sim.get_lidar(
    count=360,
    max_range=3.0,
    fov=2 * math.pi,
)
```

返回长度为 `count` 的距离数组，距离单位为米。第 `i` 项对应机器人坐标系中的相对角度：

```python
angle = -fov / 2 + i * fov / count
```

没有命中障碍物或地图边界时返回 `max_range`。当前雷达只读取静态地图，不读取其他机器人。

```python
front_distance = ranges[len(ranges) // 2]
ranges = sim.get_lidar(robot_id="robot_2")
```

雷达命中点的界面显示可以通过配置控制：

```yaml
pyrobo:
  display:
    show_lidar: true
```

关闭 `show_lidar` 只会关闭可视化，不会关闭 `sim.get_lidar()` 数据接口。

## 四、显示规划路径

```python
sim.set_display_path(path)
```

路径点必须使用世界坐标。清除路径显示：

```python
sim.set_display_path([])
```

## 五、推荐的导航回调结构

```python
def nav_init(sim: Simulator):
    # 只执行一次：预处理地图、初始化规划器和控制器
    planning_map = ...
    planner = ...
    controller = ...
    return planning_map, planner, controller


def nav_run(sim: Simulator, context) -> None:
    goal = sim.get_goal()
    pose = sim.get_pose()
    feedback = sim.get_feedback()

    if goal is None:
        sim.set_display_path([])
        sim.set_control(0.0, 0.0)
        return

    # 使用 sim.map、pose、goal 或 sim.get_lidar() 规划路径
    path = planner.plan()
    sim.set_display_path(path)

    # 根据路径、当前位姿和反馈计算下一周期控制量
    a, b = controller.follow(path, pose, feedback)
    sim.set_control(a, b)
```

## 六、C++ 对应接口

项目提供的 C++ 核心接口定义在 `cpp/include/pyrobo/interface.hpp`。C++ 接口与上文 Python 接口保持一一对应；答题人可以在 `cpp/src/contestant.cpp` 中实现导航逻辑。

### 基本类型

```cpp
#include "pyrobo/interface.hpp"

pyrobo::Pose pose{x, y, yaw};      // 对应 Python 的 (x, y, yaw)
pyrobo::Point point{x, y};         // 对应 Python 的 (x, y)
pyrobo::Path path{{x1, y1}, {x2, y2}};
pyrobo::Control control{a, b};
```

地图数据使用行优先一维数组保存，元素非零表示可通行，零表示障碍物：

```cpp
const std::vector<std::uint8_t>& data = world_map.data();
bool free = data[row * world_map.shape().width + column] != 0;
```

### 地图查询

Python：

```python
world_map = sim.map
```

C++：

```cpp
const pyrobo::Map& world_map = sim.map();
```

地图数据字段对应关系：

```python
world_map.data
world_map.shape
world_map.resolution
world_map.origin
```

```cpp
const auto& data = world_map.data();
pyrobo::Shape shape = world_map.shape();  // shape.height, shape.width
double resolution = world_map.resolution();
pyrobo::Point origin = world_map.origin(); // origin.x, origin.y
```

世界坐标和栅格坐标转换：

```python
column, row = world_map.world_to_grid(x, y)
x, y = world_map.grid_to_world(column, row)
```

```cpp
pyrobo::GridIndex index = world_map.world_to_grid(x, y);
int column = index.column;
int row = index.row;

pyrobo::Point center = world_map.grid_to_world(column, row);
double center_x = center.x;
double center_y = center.y;
```

查询可通行区域和地图实际尺寸：

```python
free = world_map.is_free(x, y)
width_m, height_m = world_map.size_meters
```

```cpp
bool free = world_map.is_free(x, y);
pyrobo::Point size_meters = world_map.size_meters();
double width_m = size_meters.x;
double height_m = size_meters.y;
```

### 导航回调结构

Python：

```python
context = nav_init(sim)
sim.run(callback=lambda sim: nav_run(sim, context))
```

C++：

```cpp
std::unique_ptr<pyrobo::NavigationContext> context = pyrobo::nav_init(sim);
pyrobo::nav_run(sim, *context);
```

候选人通常只需要修改 `cpp/src/contestant.cpp` 中的：

```cpp
std::unique_ptr<pyrobo::NavigationContext> nav_init(pyrobo::Simulator& sim);
void nav_run(pyrobo::Simulator& sim, pyrobo::NavigationContext& context);
```

### 获取机器人位姿和半径

Python：

```python
x, y, yaw = sim.get_pose()
x, y, yaw = sim.get_pose(robot_id="robot_2")
robot = sim.get_robot()
radius = robot.radius
```

C++：

```cpp
pyrobo::Pose pose = sim.get_pose();
pyrobo::Pose pose_2 = sim.get_pose("robot_2");

pyrobo::RobotInfo robot = sim.get_robot();
double radius = robot.radius;
```

### 获取、设置和清除目标点

Python：

```python
goal = sim.get_goal()
sim.set_goal((x, y))
sim.set_goal((x, y, yaw))
sim.clear_goal()
```

C++：

```cpp
std::optional<pyrobo::Pose> goal = sim.get_goal();
if (!goal.has_value()) {
    sim.set_control(0.0, 0.0);
    return;
}

sim.set_goal(pyrobo::Point{x, y});
sim.set_goal(pyrobo::Pose{x, y, yaw});
sim.clear_goal();
```

### 获取反馈速度

Python：

```python
first, second = sim.get_feedback()
first, second = sim.get_feedback(robot_id="robot_2")
vx, vy, omega = sim.get_vector_feedback()
```

C++：

```cpp
pyrobo::Feedback feedback = sim.get_feedback();
double first = feedback.first;
double second = feedback.second;

pyrobo::Feedback feedback_2 = sim.get_feedback("robot_2");

pyrobo::VectorFeedback vector_feedback = sim.get_vector_feedback();
double vx = std::get<0>(vector_feedback);
double vy = std::get<1>(vector_feedback);
double omega = std::get<2>(vector_feedback);
```

### 设置和读取控制量

Python：

```python
sim.set_control(a, b)
sim.set_control(0.2, 0.0, robot_id="robot_2")
a, b = sim.get_control()
```

C++：

```cpp
sim.set_control(a, b);
sim.set_control(0.2, 0.0, "robot_2");

pyrobo::Control control = sim.get_control();
double a = control.first;
double b = control.second;
```

控制量限制仍由仿真器根据配置文件自动处理；C++ 答题代码只负责输出期望控制量。

### 二维雷达

Python：

```python
ranges = sim.get_lidar(count=360, max_range=3.0, fov=2 * math.pi)
front_distance = ranges[len(ranges) // 2]
ranges = sim.get_lidar(robot_id="robot_2")
```

C++：

```cpp
constexpr double pi = 3.14159265358979323846;
std::vector<double> ranges = sim.get_lidar("robot", 360, 3.0, 2.0 * pi);
double front_distance = ranges[ranges.size() / 2];

std::vector<double> ranges_2 = sim.get_lidar("robot_2");
```

第 `i` 项对应机器人坐标系中的相对角度仍为：

```cpp
double angle = -fov / 2.0 + static_cast<double>(i) * fov / count;
```

### 显示规划路径

Python：

```python
sim.set_display_path(path)
sim.set_display_path([])
```

C++：

```cpp
pyrobo::Path path{{x1, y1}, {x2, y2}};
sim.set_display_path(path);
sim.set_display_path({});
```

路径点必须使用世界坐标，单位为米。

## 七、构建和运行

项目提供跨平台脚本，Windows/Linux/macOS 均可使用。构建脚本会用 CMake 编译 `cpp/src/contestant.cpp`，运行脚本默认先构建 C++ 模板，再启动 Python 仿真器。

Windows：

```bat
build.bat
run.bat
```

Linux/macOS：

```sh
sh build.sh
sh run.sh
```

也可以直接使用 Python 脚本：

```sh
python scripts/build.py
python scripts/run.py
```

常用参数：

```sh
python scripts/build.py --clean
python scripts/build.py --config Debug
python scripts/run.py --skip-build
python scripts/run.py --clean
```

运行前需要本机已安装 Python、CMake 和支持 C++17 的编译器。Python 仿真器还需要项目原有依赖，例如 `pygame`、`PyYAML`、`numpy` 和 `Pillow`。
