# Agent ROS Bridge

<p align="center">
  <img src="https://img.shields.io/badge/version-0.6.4-blue.svg" alt="Version">
  <img src="https://img.shields.io/badge/python-3.11%20%7C%203.12%20%7C%203.13%20%7C%203.14-blue.svg" alt="Python">
  <img src="https://img.shields.io/badge/tests-1,872%2B-passing.svg" alt="Tests">
  <img src="https://img.shields.io/badge/coverage-63%25-yellow.svg" alt="Coverage">
  <img src="https://img.shields.io/badge/Gate%202-PASSED-brightgreen.svg" alt="Gate 2">
  <img src="https://img.shields.io/badge/license-MIT-green.svg" alt="License">
</p>

<p align="center">
  <b>Natural Language Control for ROS Robots with AI-Human Collaboration</b><br>
  <b>自然语言控制 ROS 机器人，支持 AI-人类协作</b>
</p>

---

## 🌍 Language / 语言

- [English](#english)
- [中文](#中文)

---

<a name="english"></a>
## English

### Overview

Agent ROS Bridge enables **natural language control of ROS-based robots** through AI-powered intent parsing and human-in-the-loop validation.

**Why This Matters:**

| Without Agent ROS Bridge | With Agent ROS Bridge |
|--------------------------|----------------------|
| Write complex ROS commands | Just say "Go to the kitchen" |
| Manual safety checks | AI validates + human confirms |
| No learning from operators | Learns from 200+ hours of decisions |
| Fixed behaviors | Adapts to your preferences |
| No simulation validation | 10K scenarios tested before deployment |

**Value Proposition:**
- 🎯 **Lower Barrier**: Non-technical users can control robots
- 🛡️ **Safer**: AI proposes, human approves, system validates
- 📈 **Self-Improving**: Learns from operator corrections
- ✅ **Proven**: 95.93% success rate on 10K test scenarios

**Key Features:**
- 🗣️ **Natural Language Control** - Control robots with plain English/Chinese commands
- 🤖 **AI-Human Collaboration** - AI proposes, humans approve/reject/modify
- 🛡️ **Safety First** - Hardware-enforced limits and real-time validation
- 📊 **Shadow Mode** - Collect 200+ hours of AI vs human decision data
- 🎮 **Simulation** - 10K+ scenario validation with Gazebo/Nav2
- 🌐 **Multi-Language** - Support for English, Chinese, and 4+ other languages

### Installation

```bash
pip install agent-ros-bridge==0.6.4
```

### Quick Start

#### 🚀 NEW: Agentic Interface (High-Level)

Works with **ANY** ROS device: mobile robots, drones, arms, humanoids, sensors

```python
from agent_ros_bridge.agentic import RobotAgent

# Mobile Robot
agent = RobotAgent(
    device_id='bot1',
    device_type='mobile_robot',  # or 'drone', 'manipulator', 'humanoid', 'sensor_array'
    llm_provider='moonshot',
    require_confirmation=True,
)
result = agent.execute("Go to the kitchen and pick up the red cup")

# Drone
agent = RobotAgent(device_id='drone1', device_type='drone')
result = agent.execute("Take off to 10 meters and capture image of building")

# Robot Arm
agent = RobotAgent(device_id='arm1', device_type='manipulator')
result = agent.execute("Move to position (0.5, 0.2, 0.3) and grasp with 0.5N force")

# Humanoid
agent = RobotAgent(device_id='digit1', device_type='humanoid')
result = agent.execute("Walk forward 5 steps and pick up the box")

# Sensor Array
agent = RobotAgent(device_id='sensors1', device_type='sensor_array')
result = agent.execute("Scan the area and detect all heat signatures")

print(result.message)
print(f"AI confidence: {result.ai_confidence:.2f}")
print(f"Steps executed: {len(result.steps)}")
print(f"Human approvals: {result.human_approvals}")
```

#### Low-Level API (Direct Control)

```python
from agent_ros_bridge.gateway import AgentGateway

# Direct command sending (for advanced users)
gateway = AgentGateway()
result = gateway.send_command(
    robot_id='bot1',
    command={'type': 'navigate_to', 'parameters': {'location': 'kitchen'}}
)
```

### Gate 2 Validation Results ✅

| Metric | Result | Target | Status |
|--------|--------|--------|--------|
| Scenarios | 10,000 | 10,000 | ✅ |
| Success Rate | 95.93% | >95% | ✅ |
| Safety Violations | 0 | 0 | ✅ |

**Full validation report**: [docs/gate2_validation_10k/](docs/gate2_validation_10k/)

### Architecture

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   User Input    │────▶│  Intent Parser  │────▶│  AI Proposal    │
│  (Natural Lang) │     │   (LLM/NLP)     │     │  (Confidence)   │
└─────────────────┘     └─────────────────┘     └────────┬────────┘
                                                         │
                              ┌──────────────────────────┼──────────────────────────┐
                              │                          │                          │
                              ▼                          ▼                          ▼
                    ┌─────────────────┐        ┌─────────────────┐        ┌─────────────────┐
                    │ Human Confirms  │        │ Human Rejects   │        │ Human Modifies  │
                    │   (Approve)     │        │   (with reason) │        │   (Edit params) │
                    └────────┬────────┘        └────────┬────────┘        └────────┬────────┘
                             │                          │                          │
                             └──────────────────────────┼──────────────────────────┘
                                                        │
                                                        ▼
                                             ┌─────────────────┐
                                             │ Safety Validator │
                                             │  (Real-time)     │
                                             └────────┬────────┘
                                                      │
                                                      ▼
                                             ┌─────────────────┐
                                             │  Robot Execution │
                                             │   (ROS2/Nav2)   │
                                             └─────────────────┘
```

### Core Components

#### 1. Human Confirmation UI

Web interface for operators to review AI suggestions:

```python
from agent_ros_bridge.ui.confirmation import ConfirmationUI

ui = ConfirmationUI(port=8080)
ui.start_server()
# Access at http://localhost:8080
```

Features:
- Real-time proposal display (intent, confidence, entities)
- Approve/Reject/Modify actions
- Auto-approve high confidence (>threshold)
- Safety warnings for low confidence

#### 2. Shadow Mode Collection

Collect 200+ hours of AI vs human decision data:

```python
from agent_ros_bridge.shadow.collector import start_shadow_collection

collector = start_shadow_collection(
    output_dir='shadow_data',
    target_hours=200.0
)

# Data is collected automatically in background
# Check status: collector.get_status()
```

#### 3. Simulation & Validation

Run 10K scenario validation:

```python
from agent_ros_bridge.validation.scenario_10k import run_gate2_validation

result = run_gate2_validation(count=10000)
print(f"Success rate: {result['success_rate']*100:.2f}%")
```

Features:
- Parallel execution (4-8 Gazebo worlds)
- Procedural scenario generation
- Real-time metrics and reporting

#### 4. Safety & Validation

```python
from agent_ros_bridge.safety.safety_validator import SafetyValidator

validator = SafetyValidator(
    max_velocity=1.0,
    max_acceleration=0.5
)

result = validator.validate(command)
# {'safe': True, 'violations': []}
```

### Supported LLM Providers

- **Moonshot (Kimi)** - Recommended, optimized for Chinese
- **OpenAI (GPT-4)** - General purpose
- **Anthropic (Claude)** - General purpose
- **Local Models** - Via Ollama or similar

### Documentation

- [Quick Start Guide](docs/QUICK_START.md) - Get started in 5 minutes
- [Deployment Guide](docs/DEPLOYMENT_GUIDE.md) - Production deployment
- [API Reference](docs/API_REFERENCE.md) - Complete API documentation
- [Changelog](CHANGELOG.md) - Version history

### System Requirements

- **Python**: 3.11, 3.12, 3.13, or 3.14
- **ROS2**: Humble or Jazzy (for robot integration)
- **OS**: Ubuntu 22.04/24.04, macOS, Windows (WSL)
- **RAM**: 4GB minimum, 8GB recommended
- **Disk**: 10GB free space

### Community

- **Issues**: [GitHub Issues](https://github.com/webthree549-bot/agent-ros-bridge/issues)
- **Discussions**: [GitHub Discussions](https://github.com/webthree549-bot/agent-ros-bridge/discussions)
- **PyPI**: https://pypi.org/project/agent-ros-bridge/

### License

MIT License - see [LICENSE](LICENSE) file

---

<a name="中文"></a>
## 中文

### 概述

Agent ROS Bridge 通过 AI 驱动的意图解析和人机协同验证，实现对基于 ROS 的机器人的自然语言控制。

**为什么重要：**

| 没有 Agent ROS Bridge | 使用 Agent ROS Bridge |
|----------------------|----------------------|
| 编写复杂的 ROS 命令 | 只需说"去厨房" |
| 手动安全检查 | AI 验证 + 人类确认 |
| 无法从操作员学习 | 从 200+ 小时决策中学习 |
| 固定行为 | 适应您的偏好 |
| 无仿真验证 | 部署前测试 10K 场景 |

**核心价值：**
- 🎯 **降低门槛**：非技术用户可控制机器人
- 🛡️ **更安全**：AI 提议、人类批准、系统验证
- 📈 **自我改进**：从操作员纠正中学习
- ✅ **已验证**：10K 测试场景成功率 95.93%

**核心特性：**
- 🗣️ **自然语言控制** - 使用简单的英语/中文命令控制机器人
- 🤖 **AI-人类协作** - AI 提议，人类批准/拒绝/修改
- 🛡️ **安全第一** - 硬件强制执行限制和实时验证
- 📊 **影子模式** - 收集 200+ 小时的 AI 与人类决策数据
- 🎮 **仿真** - 使用 Gazebo/Nav2 进行 10K+ 场景验证
- 🌐 **多语言** - 支持英语、中文和 4+ 其他语言

### 安装

```bash
pip install agent-ros-bridge==0.6.4
```

### 快速开始

#### 🚀 新功能：智能体接口（高级）

```python
from agent_ros_bridge.agentic import RobotAgent

# 为您的机器人创建 AI 智能体
agent = RobotAgent(
    robot_id='bot1',
    llm_provider='moonshot',  # 或 'openai', 'anthropic'
    require_confirmation=True,  # 人机协同安全
)

# 说出您的需求 - AI 处理一切
result = agent.execute("去厨房并拿起红杯子")

print(result.message)
# "成功导航到厨房并拿起红杯子"

print(f"AI 置信度: {result.ai_confidence:.2f}")
print(f"执行步骤: {len(result.steps)}")
print(f"人工确认: {result.human_approvals}")
```

#### 底层接口（直接控制）

```python
from agent_ros_bridge.gateway import AgentGateway

# 直接命令发送（适合高级用户）
gateway = AgentGateway()
result = gateway.send_command(
    robot_id='bot1',
    command={'type': 'navigate_to', 'parameters': {'location': 'kitchen'}}
)
```

### Gate 2 验证结果 ✅

| 指标 | 结果 | 目标 | 状态 |
|------|------|------|------|
| 场景数 | 10,000 | 10,000 | ✅ |
| 成功率 | 95.93% | >95% | ✅ |
| 安全违规 | 0 | 0 | ✅ |

**完整验证报告**: [docs/gate2_validation_10k/](docs/gate2_validation_10k/)

### 架构

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   用户输入       │────▶│   意图解析器     │────▶│   AI 提议       │
│  (自然语言)      │     │   (LLM/NLP)     │     │   (置信度)      │
└─────────────────┘     └─────────────────┘     └────────┬────────┘
                                                         │
                              ┌──────────────────────────┼──────────────────────────┐
                              │                          │                          │
                              ▼                          ▼                          ▼
                    ┌─────────────────┐        ┌─────────────────┐        ┌─────────────────┐
                    │ 人类确认         │        │ 人类拒绝         │        │ 人类修改         │
                    │   (批准)        │        │   (附带原因)     │        │   (编辑参数)     │
                    └────────┬────────┘        └────────┬────────┘        └────────┬────────┘
                             │                          │                          │
                             └──────────────────────────┼──────────────────────────┘
                                                        │
                                                        ▼
                                             ┌─────────────────┐
                                             │   安全验证器     │
                                             │   (实时)        │
                                             └────────┬────────┘
                                                      │
                                                      ▼
                                             ┌─────────────────┐
                                             │   机器人执行     │
                                             │   (ROS2/Nav2)   │
                                             └─────────────────┘
```

### 核心组件

#### 1. 人类确认界面 (Human Confirmation UI)

供操作员审核 AI 建议的 Web 界面：

```python
from agent_ros_bridge.ui.confirmation import ConfirmationUI

ui = ConfirmationUI(port=8080)
ui.start_server()
# 访问 http://localhost:8080
```

特性：
- 实时提议显示（意图、置信度、实体）
- 批准/拒绝/修改操作
- 高置信度自动批准（可配置阈值）
- 低置信度安全警告

#### 2. 影子模式数据收集 (Shadow Mode Collection)

收集 200+ 小时的 AI 与人类决策数据：

```python
from agent_ros_bridge.shadow.collector import start_shadow_collection

collector = start_shadow_collection(
    output_dir='shadow_data',
    target_hours=200.0
)

# 数据在后台自动收集
# 检查状态: collector.get_status()
```

#### 3. 仿真与验证 (Simulation & Validation)

运行 10K 场景验证：

```python
from agent_ros_bridge.validation.scenario_10k import run_gate2_validation

result = run_gate2_validation(count=10000)
print(f"成功率: {result['success_rate']*100:.2f}%")
```

特性：
- 并行执行（4-8 个 Gazebo 世界）
- 程序化生成长场景
- 实时指标和报告

#### 4. 安全与验证 (Safety & Validation)

```python
from agent_ros_bridge.safety.safety_validator import SafetyValidator

validator = SafetyValidator(
    max_velocity=1.0,
    max_acceleration=0.5
)

result = validator.validate(command)
# {'safe': True, 'violations': []}
```

### 支持的 LLM 提供商

- **Moonshot (Kimi)** - 推荐，针对中文优化
- **OpenAI (GPT-4)** - 通用目的
- **Anthropic (Claude)** - 通用目的
- **本地模型** - 通过 Ollama 或类似工具

### 文档

- [快速开始指南](docs/QUICK_START.md) - 5 分钟上手
- [部署指南](docs/DEPLOYMENT_GUIDE.md) - 生产环境部署
- [API 参考](docs/API_REFERENCE.md) - 完整 API 文档
- [更新日志](CHANGELOG.md) - 版本历史

### 系统要求

- **Python**: 3.11, 3.12, 3.13, 或 3.14
- **ROS2**: Humble 或 Jazzy（用于机器人集成）
- **操作系统**: Ubuntu 22.04/24.04, macOS, Windows (WSL)
- **内存**: 4GB 最低，8GB 推荐
- **磁盘**: 10GB 可用空间

### 社区

- **问题**: [GitHub Issues](https://github.com/webthree549-bot/agent-ros-bridge/issues)
- **讨论**: [GitHub Discussions](https://github.com/webthree549-bot/agent-ros-bridge/discussions)
- **PyPI**: https://pypi.org/project/agent-ros-bridge/

### 许可证

MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

---

<p align="center">
  Made with ❤️ for the ROS community<br>
  为 ROS 社区精心打造
</p>
