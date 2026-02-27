# Agent ROS Bridge - 测试策略与测试驱动开发 (TDD) 方案

> **分析日期**: 2026-02-26  
> **项目**: Agent ROS Bridge v0.5.0  
> **目标**: 建立业界领先的测试体系，支撑生产级可靠性

---

## 1. 项目特性分析

### 1.1 系统架构特征

| 维度 | 特征 | 测试影响 |
|------|------|----------|
| **协议异构性** | WebSocket/MQTT/gRPC → ROS1/ROS2 | 需要协议转换验证、端到端契约测试 |
| **异步并发** | asyncio-based, 多Transport并行 | 需要竞态条件测试、负载压力测试 |
| **安全关键** | JWT Auth, RBAC, Safety Manager | 需要安全渗透测试、故障注入测试 |
| **AI集成** | LangChain/MCP/AutoGPT适配器 | 需要Mock外部AI服务、契约测试 |
| **分布式** | Fleet Orchestration, 多机器人 | 需要分布式一致性测试、网络分区测试 |
| **实时性** | 机器人控制延迟敏感 | 需要性能基准测试、延迟分布测试 |

### 1.2 风险热力图

```
                    影响程度
           低          中           高
      ┌─────────┬──────────┬──────────┐
  高  │         │ 协议转换 │ 安全漏洞 │ 概率
  概率│         │ (MQTT→ROS│ (JWT绕过)│
      ├─────────┼──────────┼──────────┤
  中  │ 配置错误│ 并发竞态 │ 内存泄漏 │
      │         │ 状态不一致│         │
      ├─────────┼──────────┼──────────┤
  低  │ 文档    │ AI适配器 │ 硬件兼容 │
      │ 过时    │ API变更  │ 问题    │
      └─────────┴──────────┴──────────┘
```

---

## 2. 测试金字塔设计

### 2.1 金字塔结构 (目标覆盖率)

```
                    /\
                   /  \
                  / E2E \          5%  (端到端测试)
                 /________\            - 真实ROS环境
                /          \           - 全链路验证
               / Integration \    15%  (集成测试)
              /________________\        - 跨模块交互
             /                  \       - 容器化环境
            /     Contract        \ 10% (契约测试)
           /________________________\    - Pact验证
          /                        \    - API兼容性
         /         Unit              \ 70% (单元测试)
        /______________________________\   - 纯内存/快速
                                           - 边界条件
```

### 2.2 测试分层详细规划

#### Layer 1: 单元测试 (70%)

| 模块 | 测试重点 | 技术方案 |
|------|----------|----------|
| `core.py` | Bridge生命周期、消息路由 | pytest + pytest-asyncio + 异步mock |
| `config.py` | 配置加载、环境变量覆盖 | pytest + monkeypatch + 临时文件 |
| `transports/` | 协议编解码、连接管理 | socket mock + 字节流验证 |
| `connectors/` | ROS消息转换、话题发现 | ros2 launch testing (如果可用) |
| `integrations/` | AI适配器接口、内存操作 | 内存SQLite + Mock LLM |
| `fleet/` | 任务调度、负载均衡 | 确定性调度测试 |
| `safety/` | 策略评估、紧急停止 | 状态机验证 |

**关键模式 - 异步单元测试:**

```python
# tests/unit/test_async_patterns.py
import pytest
import asyncio
from unittest.mock import AsyncMock, patch

class TestAsyncMessageRouting:
    """测试异步消息路由 - 零真实IO"""
    
    @pytest.fixture
    def event_loop():
        """每个测试独立事件循环"""
        loop = asyncio.new_event_loop()
        yield loop
        loop.close()
    
    @pytest.mark.asyncio
    async def test_message_raced_to_multiple_handlers():
        """测试消息竞态 - 高并发场景"""
        bridge = MockBridge()
        
        # 模拟100个并发消息
        messages = [create_test_message(i) for i in range(100)]
        
        # 并发发送
        await asyncio.gather(*[
            bridge.route_message(msg) for msg in messages
        ])
        
        # 验证：所有消息被处理且无重复
        assert bridge.processed_count == 100
        assert len(set(bridge.processed_ids)) == 100
```

#### Layer 2: 契约测试 (10%)

使用 Pact 进行消费者驱动契约测试，验证 WebSocket/MCP API 稳定性。

#### Layer 3: 集成测试 (15%)

容器化集成测试，使用 Docker Compose 启动完整环境。

#### Layer 4: 端到端测试 (5%)

真实场景E2E测试，验证完整机器人工作流。

---

## 3. 专项测试领域

### 3.1 性能与基准测试

- 延迟基准 (P50/P99/P999)
- 并发连接扩展性
- 内存使用模式

### 3.2 安全测试

- JWT渗透测试
- RBAC权限验证
- 传输层安全头检查

### 3.3 故障注入与混沌测试

- 网络分区模拟
- 进程崩溃恢复
- 内存压力测试

### 3.4 兼容性测试矩阵

- Python 3.9-3.13
- ROS2 Humble/Jazzy/Rolling
- LangChain 0.1.x/0.2.x

---

## 4. 测试驱动开发 (TDD) 工作流

### 4.1 TDD周期

```
1. WRITE FAILING TEST (红)
   ↓
2. MINIMAL IMPLEMENTATION (绿)
   ↓
3. REFACTOR (保持绿)
   ↓
   返回步骤1
```

### 4.2 实施路线图

| 阶段 | 时间 | 目标 |
|------|------|------|
| **Phase 1** | 1周 | 补充核心单元测试，达到70%覆盖率 |
| **Phase 2** | 1周 | 建立契约测试和集成测试框架 |
| **Phase 3** | 1周 | 实现性能测试和安全测试 |
| **Phase 4** | 持续 | TDD工作流，每次新功能先写测试 |

---

## 5. 推荐的测试工具链

| 用途 | 工具 | 理由 |
|------|------|------|
| 测试框架 | pytest + pytest-asyncio | 业界标准，异步支持好 |
| Mock | unittest.mock + pytest-mock | 标准库，无需额外依赖 |
| 契约测试 | Pact | 消费者驱动，支持多种语言 |
| 容器测试 | testcontainers | 真实服务，自动清理 |
| 性能测试 | pytest-benchmark | 集成简单，可比较历史 |
| 安全测试 | bandit + safety | 静态分析，依赖检查 |
| 突变测试 | mutmut | 验证测试质量 |
| 覆盖率 | pytest-cov | 与pytest无缝集成 |

---

请确认这个方案设计后，我将开始实施具体的测试代码。确认内容：
1. 测试金字塔分层比例是否合理？
2. 是否需要调整优先级或范围？
3. 是否有特定的测试场景需要重点关注？
