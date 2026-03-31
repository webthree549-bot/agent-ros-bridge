# OPC Skills 统一接口规范 v1.0

## 1. 标准返回格式

所有 OPC skills 必须返回以下格式：

```python
{
    "status": "success" | "error" | "needs_tool_execution",
    "result": Any,  # 根据 status 变化
    "message": str,  # 人类可读描述
    "execution_mode": "direct" | "delegated"  # 实际使用的模式
}
```

### status 定义

| status | 含义 | result 内容 |
|--------|------|-------------|
| `success` | 执行成功 | 实际结果数据 |
| `error` | 执行失败 | None 或错误详情 |
| `needs_tool_execution` | 需要 Agent 执行 tool | tool_calls 列表 |

### execution_mode 定义

| mode | 说明 |
|------|------|
| `direct` | 直接执行，已写入/读取文件 |
| `delegated` | 返回 tool_calls，由 Agent 执行 |

---

## 2. Tool Call 格式

```python
{
    "tool": "write" | "read" | "memory_search" | "memory_get",
    "params": {
        # 工具特定参数
    },
    "sequence": int  # 执行顺序，从1开始
}
```

### write 工具

```python
{
    "tool": "write",
    "params": {
        "path": str,      # 文件路径，支持 ~ 展开
        "content": str    # 文件内容
    },
    "sequence": int
}
```

### read 工具

```python
{
    "tool": "read",
    "params": {
        "path": str       # 文件路径
    },
    "sequence": int
}
```

### memory_search 工具

```python
{
    "tool": "memory_search",
    "params": {
        "query": str,     # 搜索查询
        "max_results": int  # 可选，默认10
    },
    "sequence": int
}
```

---

## 3. Context 参数规范

所有 skills 接收的 context 必须包含：

```python
{
    "customer_id": str,           # 必需，客户标识
    "input": dict,                # 必需，技能特定输入
    "execution_mode": str,        # 可选，默认 "direct"
    # ... 其他 skill 特定参数
}
```

---

## 4. 文件路径规范

### 标准目录结构

```
~/.openclaw/customers/{customer_id}/
├── memory/
│   ├── 2026-03-27.md      # 每日日志
│   ├── 2026-03-28.md
│   └── index.json         # 索引
├── patterns/
│   └── analysis.json      # 模式分析结果
├── milestones/
│   └── milestones.json    # 里程碑记录
└── tasks/
    └── active.json        # 活跃任务
```

### 路径构建函数

使用 `utils/storage.py` 中的标准函数：

```python
from utils.storage import build_memory_path

path = build_memory_path("OPC-001")  # ~/.openclaw/customers/OPC-001/memory/2026-03-27.md
```

---

## 5. 错误处理规范

### 错误返回格式

```python
{
    "status": "error",
    "result": {
        "error_type": str,      # 错误类型
        "error_detail": str     # 详细错误信息
    },
    "message": str,             # 用户友好的错误信息
    "execution_mode": str
}
```

### 标准错误类型

| error_type | 说明 |
|------------|------|
| `invalid_input` | 输入参数无效 |
| `customer_not_found` | 客户不存在 |
| `file_not_found` | 文件不存在 |
| `permission_denied` | 权限不足 |
| `storage_error` | 存储操作失败 |
| `internal_error` | 内部错误 |

---

## 6. 向后兼容性

### 默认行为

- `execution_mode` 默认为 `"direct"`
- 现有调用无需修改
- 新增功能通过参数启用

### 版本标记

返回结果中包含 `"_schema_version": "1.0"`

---

*规范版本: 1.0*  
*适用范围: opc-journal-suite 2.3.0+*  
*创建时间: 2026-03-27*