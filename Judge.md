# 评测规范

运行评测器：

```text
python judge.py --submission submission.zip --cases cases --results results
```

只评测某一道题时：

```text
python judge.py --submission submission.zip --cases cases --results results --question 3
```

## 一、测试用例

一个 `case.yaml` 表示一次独立测试。每道题可以准备多个测试用例。

建议目录结构：

```text
cases/
├── question1/
│   ├── case01/
│   │   ├── case.yaml
│   │   └── map.png
│   └── case02/
├── question2/
├── question3/
└── question4/
    └── case01/
        ├── case.yaml
        ├── display.png
        └── real.png
```

`case.yaml` 中的地图路径必须相对于 `case.yaml` 所在目录：

```yaml
question: 4
timeout_seconds: 60

map:
  display: display.png
  real: real.png
  resolution: 0.05
  origin_x: 0.0
  origin_y: 0.0

robot:
  radius: 0.3
  initial:
    x: 0.5
    y: 0.5
    yaw: 0.0
  goal:
    x: 9.5
    y: 7.5
    yaw: 0.0
```

Question 1、2、3 中，`display` 和 `real` 可以使用同一张地图。
Question 4 中，`display` 是答题人可以看到的地图，`real` 只由评测端使用。

## 二、Question 1 和 Question 2

使用以下四个指标：

```json
{
  "planning_success": true,
  "min_esdf": 0.42,
  "max_turn_angle": 1.57,
  "path_length": 13.42
}
```

### `planning_success`

表示是否规划出有效路径。路径必须不为空、起点和终点有效，并且路径不能穿过障碍物。
Question 2 还必须考虑机器人半径。

### `min_esdf`

表示路径采样点到最近障碍物的最小距离，单位为米。

Question 2 中必须满足：

```text
min_esdf >= robot_radius
```

### `max_turn_angle`

使用连续三个路径点计算离散转向角：

```text
v1 = P(i)   - P(i-1)
v2 = P(i+1) - P(i)

angle = acos(
    dot(v1, v2) / (|v1| * |v2|)
)
```

跳过重复路径点后，所有转向角中的最大值为 `max_turn_angle`，单位为弧度。
直行是 `0`，直角转弯是 `pi / 2`。

### `path_length`

所有相邻路径点之间的欧氏距离之和，单位为米：

```text
path_length = sum(|P(i+1) - P(i)|)
```

## 三、Question 3

Question 3 评判小车是否能够稳定地执行控制并到达终点：

```json
{
  "control_success": true,
  "time_to_goal": 18.42,
  "final_error": 0.12,
  "blocked_steps": 0,
  "dangerous_time": 0.35
}
```

### `control_success`

表示小车是否在规定时间内到达终点。

### `time_to_goal`

从仿真开始到首次进入终点判定范围的时间，单位为秒。越短越好。

### `final_error`

仿真结束时小车中心点到目标点的距离，单位为米。

### `blocked_steps`

小车因碰撞检测而无法执行移动的仿真周期数。越少越好。

### `dangerous_time`

小车中心点到最近地图障碍物的距离小于以下阈值时，累计对应的仿真时间：

```text
dangerous_distance = robot_radius + 0.05
```

每个仿真周期检查一次。满足危险距离的周期，就累加一个 `time_step`，单位为秒。
该时间越短越好。

## 四、Question 4

Question 4 使用真实地图进行碰撞和雷达检测，显示地图与真实地图可以不同：

```json
{
  "bonus_success": true,
  "opening_discovered": true,
  "time_to_goal": 24.18,
  "final_error": 0.14,
  "blocked_steps": 0,
  "dangerous_time": 0.55
}
```

### `bonus_success`

表示小车是否在规定时间内通过未知区域的真实缺口并到达终点。

### `opening_discovered`

表示小车的实际轨迹是否通过了显示地图中的未知区域，并在真实地图中保持可通行。
该字段用于确认是否确实发现并使用了缺口。

### `time_to_goal`

从仿真开始到首次到达终点的时间，单位为秒。

### `final_error`

仿真结束时小车中心点到目标点的距离，单位为米。

### `blocked_steps`

小车因真实地图碰撞检测而无法执行移动的仿真周期数。

### `dangerous_time`

使用真实地图计算小车中心点到最近障碍物的距离。当距离小于以下阈值时，累计当前仿真周期的时间：

```text
dangerous_distance = robot_radius + 0.05
```

`dangerous_time` 的单位为秒，表示小车处于危险距离内的总时间，越短越好。

## 五、输出格式

评测端为每道题输出一个 `result.json`：

```text
results/
├── question1/
│   └── result.json
├── question2/
│   └── result.json
├── question3/
│   └── result.json
└── question4/
    └── result.json
```

每个文件包含该题所有测试用例的结果：

同一道题的所有 case 等权计算。单个 case 通过时记为 `1`，失败时记为 `0`：

```text
score = max_score * 通过的 case 数量 / case 总数
```

Q3 和 Q4 的运行轨迹可以额外保存在对应 case 目录下的 `trajectory.csv` 中，
但评分只读取评测端直接计算出的指标，不读取答题代码生成的结果文件。

```json
{
  "question": 3,
  "max_score": 30,
  "score": 30,
  "cases": [
    {
      "case": "case01",
      "passed": true,
      "score": 1,
      "metrics": {
        "control_success": true,
        "time_to_goal": 18.42,
        "final_error": 0.12,
        "blocked_steps": 0,
        "dangerous_time": 0.35
      }
    }
  ]
}
```

`metrics` 保存原始评测数据，`score` 保存评测端根据所有用例计算出的分数。
答题代码不能创建、修改或决定 `results/` 中的文件。
