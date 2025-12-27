# ROS海龟机器人模拟项目

## 项目概述

本项目包含两个基于ROS Noetic的海龟机器人仿真实验，展示了不同的多智能体协作与控制策略。

### 1. 海龟跟随链 (turtle_follower)
- **功能**：实现多海龟的链式跟随行为，每个海龟跟随前一个海龟的运动
- **特点**：简单的领导-跟随模式，展示基础的机器人编队控制

### 2. 智能警卫与盗窃团伙 (turtle_guard)
- **功能**：模拟警卫机器人追捕盗窃团伙的动态对抗场景
- **特点**：
  - 警卫的自主巡逻与智能追捕
  - 盗窃团伙的跟随与逃跑机制
  - 动态参数配置系统
  - 人机交互控制
  - 完整的可视化监控工具

## 开发环境

### 必需环境
- **操作系统**：Ubuntu 20.04 LTS
- **ROS版本**：Noetic Ninjemys
- **Python版本**：2.7
- **仿真器**：turtlesim

### 可选工具
- rqt_reconfigure（参数动态调整）
- rqt_plot（数据可视化）
- rqt_graph（节点拓扑分析）
- teleop_twist_keyboard（键盘控制）

## 项目结构

```
turtle_follow_ws/                    # ROS工作空间
├── src/                            # 源代码目录
│   ├── turtle_follower/            # 海龟跟随链实验包
│   │   ├── CMakeLists.txt          # 构建配置文件
│   │   ├── package.xml             # 包描述文件
│   │   ├── scripts/                # Python脚本目录
│   │   │   └── follower.py         # 跟随链控制节点
│   │   └── launch/                 # 启动文件目录
│   │       └── follow_chain.launch # 跟随链启动文件
│   │
│   ├── turtle_guard/               # 警卫与盗窃团伙实验包
│   │   ├── CMakeLists.txt          # 构建配置文件
│   │   ├── package.xml             # 包描述文件
│   │   ├── scripts/                # Python脚本目录
│   │   │   ├── guard.py            # 警卫控制节点
│   │   │   └── gang.py             # 盗窃团伙控制节点
│   │   ├── cfg/                    # 动态参数配置
│   │   │   └── GuardConfig.cfg     # 参数配置文件
│   │   ├── launch/                 # 启动文件目录
│   │   │   └── turtle_gang.launch  # 完整系统启动文件
│   │   └── README.md               # 详细实验报告
│   │
│   └── CMakeLists.txt              # 工作空间级构建配置
├── .catkin_workspace               # Catkin工作空间标识
└── test_follow.sh                  # 测试脚本
```

## 安装与运行

### 1. 克隆项目
```bash
git clone https://github.com/你的用户名/ROS_Turtle_Simulation.git
cd ROS_Turtle_Simulation
```

### 2. 构建工作空间
```bash
# 进入工作空间目录
cd turtle_follow_ws

# 初始化工作空间（首次构建）
catkin_make

# 设置环境变量
source devel/setup.bash
```

### 3. 运行海龟跟随链实验
```bash
# 方法1：使用启动文件
roslaunch turtle_follower follow_chain.launch

# 方法2：手动启动
roscore &
rosrun turtlesim turtlesim_node
rosrun turtle_follower follower.py
```

### 4. 运行警卫与盗窃团伙实验
```bash
# 启动完整系统
roslaunch turtle_guard turtle_gang.launch

# 在另一个终端启动键盘控制（控制头目）
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/turtle2/cmd_vel
```

## 功能演示

### 海龟跟随链
1. 启动后，第一个海龟由键盘控制
2. 后续海龟自动跟随前一个海龟
3. 形成链式跟随效果

### 警卫与盗窃团伙
1. **警卫巡逻**：Turtle1沿预设矩形路径巡逻
2. **盗窃团伙**：
   - Turtle2（头目）：由键盘控制
   - Turtle3（小弟）：自动跟随头目
3. **交互效果**：
   - 警卫检测到入侵者（距离<3米）时开始追捕
   - 小弟在头目危险时（距离警卫<1米）恐慌逃跑
   - 警卫优先追捕最近目标

## 参数配置

### 动态参数（仅turtle_guard）
通过rqt_reconfigure实时调整：
- `kp_linear`：线性控制增益 [0.0, 5.0]
- `kp_angular`：角度控制增益 [0.0, 10.0]
- `detection_radius`：检测半径 [0.5, 8.0]米
- `wp_tolerance`：航点容差 [0.1, 2.0]米

### 静态参数
在代码中修改：
- 巡逻航点坐标：`guard.py`中的`waypoints`列表
- 跟随距离：`gang.py`中的`panic_distance`和跟随距离阈值
- 控制频率：各节点的`rospy.Rate()`参数

## 可视化工具

### 1. 参数调整
```bash
rosrun rqt_reconfigure rqt_reconfigure
```

### 2. 数据绘图
```bash
# 监控警卫与最近目标的距离
rosrun rqt_plot rqt_plot /guard/min_dist
```

### 3. 节点拓扑
```bash
rosrun rqt_graph rqt_graph
```

## 项目特点

### 技术特点
1. **模块化设计**：每个功能独立为节点，便于维护和扩展
2. **动态配置**：运行时参数调整，无需重新编译
3. **多智能体协作**：展示不同的群体行为模式
4. **人机交互**：键盘控制与自主行为结合

### 教育价值
1. **ROS入门学习**：涵盖节点、话题、服务、参数等核心概念
2. **控制算法实践**：PID控制、状态机、路径跟踪等
3. **多机器人系统**：领导-跟随、协同、对抗等模式

## 常见问题

### Q1: 启动时出现"RLException"错误
**A**: 确保已设置环境变量：`source devel/setup.bash`

### Q2: 键盘控制无响应
**A**: 检查话题映射是否正确：`cmd_vel:=/turtle2/cmd_vel`

### Q3: 图形界面卡顿
**A**: 虚拟机用户请启用3D加速并增加显存分配

### Q4: 动态参数调整无效
**A**: 确保已正确编译cfg文件并重新启动节点
