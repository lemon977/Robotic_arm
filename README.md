# 🚀 Robotic Arm Data & Inference System

面向机器人控制的数据采集、处理与推理系统（Python / ROS / 实时控制）

---

## 🧠 项目定位

这是一个完整的机器人数据闭环系统：

```
数据采集 → 数据处理 → 推理控制 → 行为回放
```

本项目聚焦于**机器人数据驱动控制（Data-Driven Robotics）**，实现从数据到控制的工程落地。

---

## 👨‍💻 我的工作（重点强化🔥）

在该项目中，我主要负责核心工程实现与系统落地：

* ✅ 设计并实现 **机器人数据采集 Pipeline（ROS1 / ROS2）**
* ✅ 构建 **数据回放系统（用于离线验证与调试）**
* ✅ 实现 **基于 OpenPI / RTC 的实时推理控制**
* ✅ 完成 **机械臂 + 灵巧手协同控制逻辑**
* ✅ 参与 **AgileX 平台可视化系统集成**
* ✅ 针对真实硬件进行 **调试与性能优化（低延迟控制）**

---

## 🏗️ 系统架构

```
Robot Hardware (Arm + Gripper)
        │
        ▼
Data Collection (ROS / Python)
        │
        ▼
Data Processing & Management
        │
        ▼
Inference Engine (OpenPI / RTC)
        │
        ▼
Control Execution (Real-time)
```

---

## 📦 项目结构

```
Robotic_arm/
├── agilex/
│   ├── demo_inference/
│   └── visualization_platform/
│
├── arx/
│   ├── demo_inference/
│   └── start_template/
│
├── LICENSE
├── THIRD_PARTY_LICENSES.md
└── README.md
```

---

## 🔥 核心功能

### 1️⃣ 数据采集（Data Pipeline）

* 机械臂轨迹采集
* 灵巧手动作记录
* ROS1 / ROS2 数据流接入
* 多源数据同步

---

### 2️⃣ 数据回放（Replay）

* 行为复现
* 离线调试
* 推理验证

---

### 3️⃣ 推理系统（Inference）

* 基于 OpenPI / RTC
* 模型输出 → 控制指令
* 实时机器人控制（低延迟）

---

### 4️⃣ 机器人控制

* 支持 ARX / AgileX 平台
* CAN 通信初始化
* 自动化启动 / 停止脚本

---

### 5️⃣ 灵巧手控制（Gripper）

* 抓取控制
* 手指开合
* 与机械臂协同操作

---

## 🚀 快速开始

### 1. 环境准备

```bash
pip install numpy rospy rospkg
sudo ./can_start.sh
```

---

### 2. 启动机器人

```bash
# ARX
./arx/start_template/arx_start.sh

# AgileX
./agilex/demo_inference/start_songling.sh
```

---

### 3. 数据采集

```bash
python collect_data.py
python collect_data_ros2_noimg.py
```

---

### 4. 推理控制

```bash
python arx_openpi_inference_rtc.py
python agilex_inference_openpi_rtc.py
```

---

## 🧰 技术栈

| 类别     | 技术             |
| ------ | -------------- |
| 编程语言   | Python         |
| 机器人中间件 | ROS1 / ROS2    |
| 操作系统   | Linux / Ubuntu |
| 通信     | CAN            |
| 推理框架   | OpenPI / RTC   |
| 数据处理   | NumPy          |

---

## 📊 项目亮点（面试加分🔥）

* 🚀 实现完整 **机器人数据闭环系统**
* 🤖 支持 **多机器人平台（ARX / AgileX）**
* ⚡ 实现 **实时推理控制（低延迟）**
* 🔁 数据回放系统提升调试效率
* 🧩 工程结构清晰，具备扩展能力

---

## 📌 可扩展方向

* 数据库（PostgreSQL / MongoDB）
* 分布式处理（Ray / Spark）
* Web 可视化（FastAPI + React）
* 仿真环境（MuJoCo）

---

## 🙏 Acknowledgement

本项目基于以下开源项目进行工程实践与二次开发：

* https://github.com/OpenDriveLab/kai0

感谢原作者的开源贡献。

---

## ⚖️ License Notice（很关键）

* 本项目整体遵循 **Apache License 2.0**
* 部分代码参考自 kai0（Apache 2.0）
* 若使用其数据或模型，则受 **CC BY-NC-SA 4.0（非商用）限制**

---

## 📄 License

See [LICENSE](./LICENSE) for details.

---

*Last Updated: 2026-03-25*