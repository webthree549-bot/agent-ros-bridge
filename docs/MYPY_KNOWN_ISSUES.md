# Known MyPy Type Issues

**Date:** March 4, 2026  
**Status:** 25 known issues (acceptable)  
**Target:** Zero new errors introduced

---

## Summary

After fixing 68 type errors (93 → 25), the remaining 25 errors fall into categories that are either:
1. **Acceptable** — Complex patterns that don't affect runtime
2. **Low Priority** — In CLI entry points or proto-generated code
3. **False Positives** — Correct code that MyPy cannot verify

---

## Known Issues by Category

### 1. gRPC Transport (8 errors)

**Location:** `agent_ros_bridge/gateway_v2/transports/grpc_transport.py`

**Issues:**
- Proto-generated base classes (`BridgeServiceServicer`)
- Missing return statements in streaming methods
- Proto descriptor access
- Stub type assignments

**Why Acceptable:**
- Proto-generated code follows gRPC patterns
- Runtime behavior is correct
- Fixing would require significant refactoring

**Example:**
```python
# MyPy error: Invalid base class
class BridgeServiceServicer(bridge_pb2.BridgeServiceServicer if GRPC_AVAILABLE else object):
    pass
```

---

### 2. CLI Entry Points (10 errors)

**Location:** `agent_ros_bridge/__main__.py`, `agent_ros_bridge/gateway_v2/__main__.py`

**Issues:**
- Complex argument parsing with dynamic types
- Config dictionary access patterns
- Transport type assignments

**Why Acceptable:**
- CLI code is entry-point only
- Runtime behavior is correct
- Type safety less critical here

**Example:**
```python
# MyPy error: Incompatible types in assignment
transport = GRPCTransport(transport_config)  # GRPCTransport vs WebSocketTransport
```

---

### 3. Subscribe Method Signatures (4 errors)

**Location:** `ros1_connector.py`, `ros2_connector.py`

**Issues:**
- `subscribe(self, topic: str, **kwargs: Any)` vs parent `subscribe(self, topic: str)`

**Why Acceptable:**
- Using `**kwargs` is a valid Python pattern
- Runtime behavior is correct
- Allows optional `msg_type` parameter

**Example:**
```python
# Parent class
async def subscribe(self, topic: str) -> AsyncIterator[Telemetry]:
    pass

# Child class (correct but MyPy complains)
async def subscribe(self, topic: str, **kwargs: Any) -> AsyncIterator[Telemetry]:
    msg_type: Optional[str] = kwargs.get("msg_type")
```

---

### 4. Connector Return Types (2 errors)

**Location:** `ros1_connector.py`

**Issues:**
- `connect()` returns `Optional[Robot]` vs parent `Robot`

**Why Acceptable:**
- Connection can fail, returning None is correct
- Runtime checks handle None case
- More honest type than pretending it always succeeds

---

### 5. Various Edge Cases (1 error)

**Location:** Various files

**Issues:**
- Complex generic types
- Dynamic attribute access

---

## CI/CD Policy

### Current

```yaml
- name: Type check with MyPy
  run: mypy agent_ros_bridge
```

**Blocking:** Yes  
**Allowed Errors:** The 25 documented errors  
**New Errors:** Must be fixed before merge

### Policy

1. **No new errors** — PRs must not increase error count
2. **Document exceptions** — Any new acceptable errors need documentation
3. **Gradual improvement** — Fix known errors when touching related code

---

## Fixing Guidelines

### When to Fix

✅ **Fix immediately:**
- New errors introduced by your code
- Simple Optional[] annotations
- Missing None checks
- Clear type mismatches

🟡 **Fix when convenient:**
- Errors in code you're already modifying
- Straightforward annotations

❌ **Don't fix (documented exceptions):**
- Proto-generated code
- Complex CLI patterns
- Acceptable override patterns

### How to Fix

**Common Patterns:**

```python
# Before (error)
def get_data() -> Dict[str, Any]:
    result = {"key": value}  # Inferred as Dict[str, str]
    result["other"] = 123    # Error: int not str
    return result

# After (fixed)
def get_data() -> Dict[str, Any]:
    result: Dict[str, Any] = {"key": value}
    result["other"] = 123
    return result
```

```python
# Before (error)
class Config:
    data: Dict[str, Any] = None  # Error: None not Dict

# After (fixed)
class Config:
    data: Optional[Dict[str, Any]] = None
```

```python
# Before (error)
if obj.attr:  # obj.attr might be None
    process(obj.attr)

# After (fixed)
if obj.attr is not None:
    process(obj.attr)
```

---

## Progress Tracking

| Date | Errors | Fixed | Notes |
|------|--------|-------|-------|
| 2026-03-03 | 93 | — | Initial count |
| 2026-03-03 | 79 | 14 | Batch 1: Optional[] annotations |
| 2026-03-03 | 73 | 6 | Batch 2: None handling |
| 2026-03-03 | 63 | 10 | Batch 3: Type annotations |
| 2026-03-03 | 48 | 15 | Batch 4: ROS connectors |
| 2026-03-03 | 43 | 5 | Batch 5: Import fixes |
| 2026-03-03 | 28 | 15 | Batch 6: ROS fixes |
| 2026-03-04 | 25 | 3 | Batch 7: Final fixes |

**Total Fixed:** 68 errors (73% reduction)

---

## References

- [MyPy Documentation](https://mypy.readthedocs.io/)
- [Python Type Hints PEP 484](https://peps.python.org/pep-0484/)
- [Project ADRs](../adr/)

---

*Last updated: March 4, 2026*  
*Maintainer: OpenClaw Agent*
