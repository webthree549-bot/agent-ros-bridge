# Agent ROS Bridge v0.6.1 Release Notes

**Release Date:** March 9, 2026  
**Version:** 0.6.1  
**Previous Version:** 0.6.0

---

## 🎉 What's New in v0.6.1

v0.6.1 is a major feature release that adds advanced AI capabilities, multi-language support, and production-ready security features.

### 🤖 LLM-Powered Intent Parsing

When rule-based parsing isn't enough, v0.6.1 can now fall back to Large Language Models:

- **OpenAI GPT-3.5/4** support
- **Anthropic Claude** support
- **Structured JSON output** for reliable parsing
- **Smart caching** - 500-entry LRU cache with 1-hour TTL
- **Rate limiting** - 100 calls/minute to control costs
- **Timeout handling** - Configurable timeouts (default 5s)

**Example:**
```python
"I need you to go to the kitchen and grab me a bottle of water"
→ NAVIGATE to kitchen + MANIPULATE pick up bottle
```

### 🌍 Multi-Language Support

Natural language understanding now works in 6 languages:

| Language | Code | Example |
|----------|------|---------|
| English | en | "go to kitchen" |
| Spanish | es | "ve a la cocina" |
| French | fr | "va à la cuisine" |
| German | de | "geh zur küche" |
| Chinese | zh | "去厨房" |
| Japanese | ja | "キッチンに行って" |

Features:
- **Auto-detection** - Automatically detects language from input
- **Native patterns** - Optimized regex patterns per language
- **Character-based detection** - Uses Unicode ranges for accurate detection

### 🧠 Context-Aware Understanding

The intent parser now understands context:

- **Conversation history** - Remembers last 10 turns
- **Pronoun resolution** - "it", "there", "here", "that" resolve to previous mentions
- **Robot state** - Knows current location, battery, task
- **Environment awareness** - Knows available locations and objects

**Example:**
```
User: "pick up the cup"
User: "place it on the table"  ← "it" resolves to "cup"
```

### 🔒 Security & Production Hardening

New security utilities for production deployments:

- **Secure API key handling** - Keys loaded from environment, masked in logs
- **Rate limiting** - Prevents API abuse and runaway costs
- **Input sanitization** - Removes dangerous characters
- **Audit logging** - Security event tracking
- **Comprehensive hardening tests** - 500+ lines of security tests

### ⚡ Performance Improvements

- **Safety validator caching** - 50x faster for repeated trajectories
- **Cache hit rate** - ~80% under typical load
- **Memory optimized** - LRU eviction prevents unbounded growth

---

## 📊 Performance Benchmarks

| Component | Target | v0.6.1 | Improvement |
|-----------|--------|--------|-------------|
| Intent Parser | <10ms | ~5ms | Baseline |
| Safety Validator | <10ms | ~0.1ms (cached) | **50x faster** |
| Motion Planner | <100ms | ~70ms | Baseline |
| LLM Fallback | <100ms | ~50-100ms | New feature |

---

## 🆕 New APIs

### LLM Intent Parser

```python
from agent_ros_bridge.ai import LLMIntentParser

parser = LLMIntentParser(
    provider="openai",  # or "anthropic"
    model="gpt-3.5-turbo",
    timeout_sec=5.0
)

result = parser.parse("go to the kitchen and get me water")
# result.intent_type = "NAVIGATE"
# result.confidence = 0.92
```

### Context-Aware Parser

```python
from agent_ros_bridge.ai import ContextAwareParser

parser = ContextAwareParser()
parser.add_conversation_turn("pick up the cup", "MANIPULATE")

resolved = parser.resolve_context("place it on the table")
# "it" → "cup"
```

### Multi-Language Parser

```python
from agent_ros_bridge.ai import MultiLanguageParser

parser = MultiLanguageParser()
result = parser.parse("ve a la cocina")  # Spanish
# result.language = "es"
# result.intent_type = "NAVIGATE"
```

---

## 🧪 Testing

New test coverage:

- **Production hardening tests** - 500+ lines
- **LLM error handling** - Timeouts, auth failures, malformed responses
- **Context edge cases** - Empty context, thread safety, memory limits
- **Multi-language validation** - Unicode handling, unsupported languages
- **Security tests** - API key masking, input sanitization, rate limiting

Run tests:
```bash
pytest tests/system/test_production_hardening.py -v
```

---

## 📚 Documentation

- [API Documentation](docs/API_AI_LAYER.md) - Complete service/action reference
- [v0.6.1 Tracking](docs/V061_TRACKING.md) - Development progress
- [CHANGELOG](CHANGELOG.md) - Full change history

---

## 🔄 Migration from v0.6.0

v0.6.1 is fully backward compatible with v0.6.0. No migration steps required.

New features are opt-in:
- LLM fallback requires `OPENAI_API_KEY` or `ANTHROPIC_API_KEY` env var
- Context awareness is automatic when available
- Multi-language detection is automatic

---

## 🐛 Known Issues

None reported.

---

## 🙏 Contributors

- Week 5-6 Advanced Features: Production hardening, LLM integration, multi-language support

---

## 📦 Installation

```bash
pip install agent-ros-bridge==0.6.1
```

With LLM support:
```bash
pip install agent-ros-bridge[llm]  # Includes openai, anthropic
```

---

**Full Changelog**: [CHANGELOG.md](CHANGELOG.md)

**GitHub Release**: https://github.com/webthree549-bot/agent-ros-bridge/releases/tag/v0.6.1
