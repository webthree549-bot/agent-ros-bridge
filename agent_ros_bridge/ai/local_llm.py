"""
Local LLM Integration for Agent ROS Bridge

Provides support for local LLMs via Ollama, LM Studio, and other local inference servers.
Falls back to cloud APIs when local models are unavailable.

Usage:
    from agent_ros_bridge.ai.local_llm import LocalLLMClient
    
    llm = LocalLLMClient(provider="ollama", model="llama3.2")
    response = await llm.generate("Navigate to the kitchen")
"""

import json
import logging
import re
from dataclasses import dataclass
from typing import Any, AsyncIterator

import aiohttp

logger = logging.getLogger(__name__)


@dataclass
class LLMResponse:
    """Response from LLM."""
    text: str
    model: str
    provider: str
    latency_ms: float
    tokens_used: int | None = None
    confidence: float = 1.0


@dataclass
class ParsedCommand:
    """Parsed robot command from natural language."""
    action: str
    parameters: dict[str, Any]
    confidence: float
    raw_text: str
    reasoning: str | None = None


class LocalLLMClient:
    """Client for local LLM inference."""
    
    SUPPORTED_PROVIDERS = ["ollama", "lmstudio", "llamafile", "openai_compatible"]
    
    def __init__(
        self,
        provider: str = "ollama",
        model: str = "llama3.2",
        base_url: str | None = None,
        temperature: float = 0.7,
        max_tokens: int = 512,
        timeout: float = 30.0,
    ):
        """
        Initialize local LLM client.
        
        Args:
            provider: "ollama", "lmstudio", "llamafile", or "openai_compatible"
            model: Model name (e.g., "llama3.2", "mistral", "phi3")
            base_url: API base URL (auto-detected if None)
            temperature: Sampling temperature
            max_tokens: Maximum tokens to generate
            timeout: Request timeout in seconds
        """
        self.provider = provider
        self.model = model
        self.temperature = temperature
        self.max_tokens = max_tokens
        self.timeout = timeout
        
        # Set default URLs
        if base_url is None:
            base_urls = {
                "ollama": "http://localhost:11434",
                "lmstudio": "http://localhost:1234",
                "llamafile": "http://localhost:8080",
                "openai_compatible": "http://localhost:8000",
            }
            self.base_url = base_urls.get(provider, "http://localhost:11434")
        else:
            self.base_url = base_url
        
        self.session: aiohttp.ClientSession | None = None
    
    async def __aenter__(self):
        self.session = aiohttp.ClientSession()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        if self.session:
            await self.session.close()
    
    async def _get_session(self) -> aiohttp.ClientSession:
        """Get or create session."""
        if self.session is None or self.session.closed:
            self.session = aiohttp.ClientSession()
        return self.session
    
    async def check_availability(self) -> bool:
        """Check if local LLM server is available."""
        try:
            session = await self._get_session()
            
            if self.provider == "ollama":
                url = f"{self.base_url}/api/tags"
            elif self.provider in ["lmstudio", "openai_compatible"]:
                url = f"{self.base_url}/v1/models"
            else:
                url = self.base_url
            
            async with session.get(url, timeout=5) as response:
                return response.status == 200
        except Exception as e:
            logger.debug(f"Local LLM not available: {e}")
            return False
    
    async def list_models(self) -> list[str]:
        """List available models."""
        try:
            session = await self._get_session()
            
            if self.provider == "ollama":
                url = f"{self.base_url}/api/tags"
                async with session.get(url, timeout=self.timeout) as response:
                    if response.status == 200:
                        data = await response.json()
                        return [m["name"] for m in data.get("models", [])]
            
            elif self.provider in ["lmstudio", "openai_compatible"]:
                url = f"{self.base_url}/v1/models"
                async with session.get(url, timeout=self.timeout) as response:
                    if response.status == 200:
                        data = await response.json()
                        return [m["id"] for m in data.get("data", [])]
        except Exception as e:
            logger.warning(f"Failed to list models: {e}")
        
        return []
    
    async def generate(self, prompt: str, system_prompt: str | None = None) -> LLMResponse:
        """
        Generate text from prompt.
        
        Args:
            prompt: User prompt
            system_prompt: Optional system prompt
        
        Returns:
            LLMResponse with generated text
        """
        import time
        start_time = time.time()
        
        if self.provider == "ollama":
            return await self._generate_ollama(prompt, system_prompt, start_time)
        elif self.provider in ["lmstudio", "openai_compatible"]:
            return await self._generate_openai_compatible(prompt, system_prompt, start_time)
        else:
            raise ValueError(f"Unsupported provider: {self.provider}")
    
    async def _generate_ollama(
        self, prompt: str, system_prompt: str | None, start_time: float
    ) -> LLMResponse:
        """Generate using Ollama API."""
        import time
        
        session = await self._get_session()
        url = f"{self.base_url}/api/generate"
        
        payload = {
            "model": self.model,
            "prompt": prompt,
            "stream": False,
            "options": {
                "temperature": self.temperature,
                "num_predict": self.max_tokens,
            },
        }
        
        if system_prompt:
            payload["system"] = system_prompt
        
        async with session.post(url, json=payload, timeout=self.timeout) as response:
            response.raise_for_status()
            data = await response.json()
            
            latency_ms = (time.time() - start_time) * 1000
            
            return LLMResponse(
                text=data.get("response", ""),
                model=self.model,
                provider=f"ollama/{self.model}",
                latency_ms=latency_ms,
                tokens_used=data.get("eval_count", 0),
            )
    
    async def _generate_openai_compatible(
        self, prompt: str, system_prompt: str | None, start_time: float
    ) -> LLMResponse:
        """Generate using OpenAI-compatible API (LM Studio, etc.)."""
        import time
        
        session = await self._get_session()
        url = f"{self.base_url}/v1/chat/completions"
        
        messages = []
        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})
        messages.append({"role": "user", "content": prompt})
        
        payload = {
            "model": self.model,
            "messages": messages,
            "temperature": self.temperature,
            "max_tokens": self.max_tokens,
        }
        
        async with session.post(url, json=payload, timeout=self.timeout) as response:
            response.raise_for_status()
            data = await response.json()
            
            latency_ms = (time.time() - start_time) * 1000
            
            choices = data.get("choices", [{}])
            text = choices[0].get("message", {}).get("content", "") if choices else ""
            
            return LLMResponse(
                text=text,
                model=self.model,
                provider=f"{self.provider}/{self.model}",
                latency_ms=latency_ms,
                tokens_used=data.get("usage", {}).get("total_tokens"),
            )
    
    async def generate_stream(
        self, prompt: str, system_prompt: str | None = None
    ) -> AsyncIterator[str]:
        """
        Generate text with streaming.
        
        Yields:
            Chunks of generated text
        """
        session = await self._get_session()
        
        if self.provider == "ollama":
            url = f"{self.base_url}/api/generate"
            payload = {
                "model": self.model,
                "prompt": prompt,
                "stream": True,
                "options": {"temperature": self.temperature},
            }
            if system_prompt:
                payload["system"] = system_prompt
            
            async with session.post(url, json=payload, timeout=self.timeout) as response:
                async for line in response.content:
                    if line:
                        try:
                            data = json.loads(line)
                            chunk = data.get("response", "")
                            if chunk:
                                yield chunk
                        except json.JSONDecodeError:
                            pass
        
        else:
            # For OpenAI-compatible, we'd need to handle SSE streaming
            # This is a simplified version
            response = await self.generate(prompt, system_prompt)
            yield response.text


class RobotCommandParser:
    """Parse robot commands using local LLM."""
    
    SYSTEM_PROMPT = """You are a robot command parser. Convert natural language commands into structured robot commands.

Available actions:
- move: Move in a direction (forward, backward, left, right)
- rotate: Rotate (left, right, angle)
- navigate: Navigate to a location or coordinates
- stop: Stop all movement
- pick: Pick up an object
- place: Place an object
- scan: Scan area or object
- status: Get robot status

Respond in JSON format:
{
    "action": "action_name",
    "parameters": {
        "key": "value"
    },
    "confidence": 0.95
}

Examples:
User: "Move forward 2 meters"
Response: {"action": "move", "parameters": {"direction": "forward", "distance": 2.0}, "confidence": 0.95}

User: "Turn left 90 degrees"
Response: {"action": "rotate", "parameters": {"direction": "left", "angle": 90}, "confidence": 0.95}

User: "Go to the kitchen"
Response: {"action": "navigate", "parameters": {"destination": "kitchen"}, "confidence": 0.85}
"""
    
    def __init__(self, llm_client: LocalLLMClient | None = None):
        self.llm = llm_client
    
    async def parse_command(self, text: str) -> ParsedCommand | None:
        """
        Parse natural language command.
        
        Args:
            text: Natural language command
        
        Returns:
            ParsedCommand or None if parsing failed
        """
        if self.llm is None:
            # Fallback to rule-based parsing
            return self._rule_based_parse(text)
        
        try:
            response = await self.llm.generate(text, self.SYSTEM_PROMPT)
            return self._extract_command(response.text)
        except Exception as e:
            logger.warning(f"LLM parsing failed: {e}, falling back to rule-based")
            return self._rule_based_parse(text)
    
    def _extract_command(self, text: str) -> ParsedCommand | None:
        """Extract command from LLM response."""
        # Try to find JSON in response
        json_match = re.search(r'\{[^}]+\}', text, re.DOTALL)
        if json_match:
            try:
                data = json.loads(json_match.group())
                return ParsedCommand(
                    action=data.get("action", "unknown"),
                    parameters=data.get("parameters", {}),
                    confidence=data.get("confidence", 0.5),
                    raw_text=text,
                    reasoning=data.get("reasoning"),
                )
            except json.JSONDecodeError:
                pass
        
        return None
    
    def _rule_based_parse(self, text: str) -> ParsedCommand | None:
        """Fallback rule-based parsing."""
        text_lower = text.lower()
        
        # Simple keyword matching
        if "forward" in text_lower:
            distance = self._extract_number(text) or 1.0
            return ParsedCommand(
                action="move",
                parameters={"direction": "forward", "distance": distance},
                confidence=0.7,
                raw_text=text,
            )
        
        if "backward" in text_lower or "back" in text_lower:
            distance = self._extract_number(text) or 1.0
            return ParsedCommand(
                action="move",
                parameters={"direction": "backward", "distance": distance},
                confidence=0.7,
                raw_text=text,
            )
        
        if "left" in text_lower:
            angle = self._extract_number(text) or 90
            return ParsedCommand(
                action="rotate",
                parameters={"direction": "left", "angle": angle},
                confidence=0.7,
                raw_text=text,
            )
        
        if "right" in text_lower:
            angle = self._extract_number(text) or 90
            return ParsedCommand(
                action="rotate",
                parameters={"direction": "right", "angle": angle},
                confidence=0.7,
                raw_text=text,
            )
        
        if "stop" in text_lower:
            return ParsedCommand(
                action="stop",
                parameters={},
                confidence=0.9,
                raw_text=text,
            )
        
        if "navigate" in text_lower or "go to" in text_lower or "drive to" in text_lower:
            # Extract destination
            destination = self._extract_destination(text)
            return ParsedCommand(
                action="navigate",
                parameters={"destination": destination},
                confidence=0.6,
                raw_text=text,
            )
        
        return None
    
    def _extract_number(self, text: str) -> float | None:
        """Extract first number from text."""
        match = re.search(r'(\d+\.?\d*)', text)
        if match:
            return float(match.group(1))
        return None
    
    def _extract_destination(self, text: str) -> str:
        """Extract destination from navigation command."""
        # Simple extraction - could be improved with NER
        patterns = [
            r'(?:to|toward|towards)\s+(.+?)(?:\s|$)',
            r'(?:navigate|go|drive)\s+(?:to\s+)?(.+?)(?:\s|$)',
        ]
        
        for pattern in patterns:
            match = re.search(pattern, text.lower())
            if match:
                return match.group(1).strip()
        
        return "unknown"


class HybridLLMClient:
    """
    Hybrid client that tries local LLM first, falls back to cloud.
    
    Usage:
        hybrid = HybridLLMClient(
            local_provider="ollama",
            local_model="llama3.2",
            cloud_provider="openai",
            cloud_model="gpt-4"
        )
        response = await hybrid.generate("Navigate to kitchen")
    """
    
    def __init__(
        self,
        local_provider: str = "ollama",
        local_model: str = "llama3.2",
        cloud_provider: str = "openai",
        cloud_model: str = "gpt-4",
        prefer_local: bool = True,
    ):
        self.local = LocalLLMClient(provider=local_provider, model=local_model)
        self.cloud_provider = cloud_provider
        self.cloud_model = cloud_model
        self.prefer_local = prefer_local
        self.last_used = "none"
    
    async def generate(self, prompt: str, system_prompt: str | None = None) -> LLMResponse:
        """Generate with fallback to cloud."""
        # Try local first if preferred
        if self.prefer_local:
            if await self.local.check_availability():
                try:
                    response = await self.local.generate(prompt, system_prompt)
                    self.last_used = "local"
                    return response
                except Exception as e:
                    logger.warning(f"Local LLM failed: {e}")
        
        # Fall back to cloud
        # This would need cloud client implementation
        logger.info("Falling back to cloud LLM")
        self.last_used = "cloud"
        
        # For now, raise error - cloud implementation would go here
        raise RuntimeError("Cloud LLM not implemented in this version")
    
    async def get_status(self) -> dict[str, Any]:
        """Get status of local and cloud providers."""
        local_available = await self.local.check_availability()
        models = await self.local.list_models() if local_available else []
        
        return {
            "local_available": local_available,
            "local_models": models,
            "local_provider": self.local.provider,
            "local_model": self.local.model,
            "cloud_provider": self.cloud_provider,
            "cloud_model": self.cloud_model,
            "prefer_local": self.prefer_local,
            "last_used": self.last_used,
        }
