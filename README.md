# 🚀 Robotic Arm Data & Inference System

面向机器人控制的数据采集、处理与推理系统（Python / ROS / 实时控制）

---

## 🧠 项目定位

这是一个完整的机器人数据闭环系统：

```
数据采集 → 数据处理 → 推理控制 → 行为回放
```

**本人负责：**
- Python 数据采集脚本及推理设计
- 推理测试验证
- 机器臂平台可视化界面（AgileX）

---

## 🏗️ 系统架构

```
Robot Hardware (Arm + Gripper)
        │
        ▼
Data Collection (ROS / Python)
        │
        ▼
Data Management & Visualization
        │
        ▼
Inference Engine (OpenPI / RTC)
        │
        ▼
Control Execution
```

---

## 📦 项目结构

```
Robotic_arm/
├── agilex/
│   ├── demo_inference/
│   │   ├── start_songling.sh
│   │   └── kill_songling.sh
│   └── visualization_platform/
│       ├── agilex_deploy/
│       ├── agilex_inference/
│       ├── kai05_collect/
│       ├── kai05_collect_management/
│   ├── agilex_inference_openpi_rtc.py
│   ├── collect_data.py
│   ├── gripper_set.py
│   └── replay_data.py
│
├── arx/
│   ├── demo_inference/
│   │   ├── start_fangzhou.sh
│   │   └── kill_fangzhou.sh
│   ├── start_template/
│       ├── arx_start.sh
│       │── can_start.sh
│       ├── config/
│   ├── arx_openpi_inference_rtc.py
│   ├── collect_data_ros2_noimg.py
│   └── replay.py
│
├── LICENSE
└── README.md
```

---

## 🔥 核心功能

### 1️⃣ 数据采集（Data Pipeline）

- 机械臂轨迹采集
- 灵巧手动作记录
- ROS2 数据流接入
- 多源数据同步

**核心脚本：**
- `collect_data.py`
- `collect_data_ros2_noimg.py`
- `agilex_inference_openpi_rtc.py`
- `arx_openpi_inference_rtc.py`

### 2️⃣ 数据回放（Replay）

- 行为复现
- 离线调试
- 推理验证

**核心脚本：**
- `replay.py`
- `replay_data.py`

### 3️⃣ 推理系统（Inference）

- 基于 OpenPI / RTC
- 实时控制机器人动作
- 模型输出 → 控制指令

**核心脚本：**
- `arx_openpi_inference_rtc.py`
- `agilex_inference_openpi_rtc.py`

### 4️⃣ 机器人控制

- 支持 ARX / AgileX
- CAN 通信初始化
- 自动化启动 / 停止

**核心脚本：**
- `start_*.sh`
- `kill_*.sh`

### 5️⃣ 灵巧手控制（Gripper）

- 抓取控制
- 手指开合
- 协同机械臂操作

**核心脚本：**
- `gripper_set.py`

---

## 🚀 快速开始

### 1. 环境准备

```bash
# 安装依赖
pip install numpy rospy rospkg

# 配置 CAN 通信（如果需要）
sudo ./can_start.sh
```

### 2. 启动机器人

**ARX 平台：**
```bash
./arx/start_template/arx_start.sh
```

**AgileX 平台：**
```bash
./agilex/demo_inference/start_songling.sh
```

### 3. 停止机器人

```bash
# ARX
./arx/demo_inference/kill_fangzhou.sh

# AgileX
./agilex/demo_inference/kill_songling.sh
```

### 4. 数据采集

```bash
# ROS1 环境
python collect_data.py

# ROS2 环境（无图像）
python collect_data_ros2_noimg.py
```

### 5. 数据回放

```bash
python replay.py
# 或
python replay_data.py
```

### 6. 推理控制

```bash
# ARX 平台
python arx_openpi_inference_rtc.py

# AgileX 平台
python agilex_inference_openpi_rtc.py
```

---

## 🧰 技术栈

| 类别 | 技术 |
|------|------|
| 编程语言 | Python |
| 机器人中间件 | ROS1 / ROS2 |
| 操作系统 | Linux / Ubuntu |
| 硬件通信 | CAN 通信 |
| 推理框架 | OpenPI / RTC |
| 数据处理 | NumPy |

---

## 📊 项目总结

### ✅ 完成内容

- [x] 搭建完整机器人数据闭环系统（采集 → 推理 → 执行）
- [x] 支持多机器人平台（ARX / AgileX）
- [x] 实现实时推理控制（低延迟）
- [x] 设计数据回放系统用于验证
- [x] 熟悉 ROS1 / ROS2 数据流与机器人通信
- [x] 工程结构清晰，具备可扩展性

---

## 📌 可扩展方向

- [ ] **数据库存储**（PostgreSQL / MongoDB）
- [ ] **数据标注系统**
- [ ] **分布式处理**（Spark / Ray）
- [ ] **Web 可视化**（FastAPI + React）
- [ ] **仿真环境**（MuJoCo）

---

## 🙋‍♂️ 关于我

- 机器人方向工程实践
- 熟悉机械臂 + 灵巧手系统
- 专注 Python + 数据处理 + 推理系统

---

## 📄 许可证

[LICENSE](./LICENSE)

---

*Last Updated: 2026-03-25*