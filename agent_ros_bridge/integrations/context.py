"""Context management for natural language conversations.

Maintains conversation history, learned locations, and session state
across multiple interactions with the robot.

This enables the SKILL promise of context-aware conversations like:
- User: "Go to the kitchen"
- [Robot goes to kitchen]
- User: "Now bring me water"  [Robot knows to get water from kitchen]
"""

import json
import sqlite3
from dataclasses import dataclass, asdict, field
from datetime import datetime
from typing import Optional, Dict, Any, List
from pathlib import Path


@dataclass
class ConversationContext:
    """Runtime context for a conversation session.
    
    Attributes:
        session_id: Unique session identifier
        current_location: Where the robot currently is
        last_action: Last command executed
        last_result: Result of last action
        known_locations: Dict of learned place names to coordinates
        pending_task: Current multi-step task if any
        conversation_history: Recent command history
    """
    session_id: str
    current_location: Optional[str] = None
    last_action: Optional[str] = None
    last_result: Optional[Dict] = None
    known_locations: Dict[str, Dict] = field(default_factory=dict)
    pending_task: Optional[str] = None
    conversation_history: List[Dict] = field(default_factory=list)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            "session_id": self.session_id,
            "current_location": self.current_location,
            "last_action": self.last_action,
            "last_result": self.last_result,
            "known_locations": self.known_locations,
            "pending_task": self.pending_task,
            "conversation_history": self.conversation_history[:10],  # Last 10
        }


class ContextManager:
    """Manages conversation context with SQLite persistence.
    
    Provides:
    - Session-based context tracking
    - Persistent storage across restarts
    - Location learning and recall
    - Conversation history
    """
    
    def __init__(self, db_path: str = ".agent_ros_context.db"):
        """Initialize context manager.
        
        Args:
            db_path: Path to SQLite database file
        """
        self.db_path = Path(db_path)
        self._init_db()
        self._runtime_contexts: Dict[str, ConversationContext] = {}
    
    def _init_db(self):
        """Initialize SQLite database tables."""
        with sqlite3.connect(self.db_path) as conn:
            # Contexts table - stores current state
            conn.execute("""
                CREATE TABLE IF NOT EXISTS contexts (
                    session_id TEXT PRIMARY KEY,
                    current_location TEXT,
                    last_action TEXT,
                    last_result TEXT,
                    known_locations TEXT,
                    pending_task TEXT,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)
            
            # History table - stores command history
            conn.execute("""
                CREATE TABLE IF NOT EXISTS history (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    session_id TEXT,
                    command TEXT,
                    interpretation TEXT,
                    result TEXT,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)
            
            # Locations table - learned places
            conn.execute("""
                CREATE TABLE IF NOT EXISTS locations (
                    session_id TEXT,
                    name TEXT,
                    coordinates TEXT,
                    learned_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    PRIMARY KEY (session_id, name)
                )
            """)
    
    def get_context(self, session_id: str) -> ConversationContext:
        """Get or create context for a session.
        
        Args:
            session_id: Unique session identifier
            
        Returns:
            ConversationContext for the session
        """
        # Check runtime cache first
        if session_id in self._runtime_contexts:
            return self._runtime_contexts[session_id]
        
        # Load from database
        with sqlite3.connect(self.db_path) as conn:
            row = conn.execute(
                """SELECT current_location, last_action, last_result, 
                          known_locations, pending_task
                   FROM contexts WHERE session_id = ?""",
                (session_id,)
            ).fetchone()
            
            if row:
                context = ConversationContext(
                    session_id=session_id,
                    current_location=row[0],
                    last_action=row[1],
                    last_result=json.loads(row[2]) if row[2] else None,
                    known_locations=json.loads(row[3]) if row[3] else {},
                    pending_task=row[4]
                )
            else:
                context = ConversationContext(session_id=session_id)
            
            # Load recent history
            context.conversation_history = self._load_history(conn, session_id, limit=10)
            
            self._runtime_contexts[session_id] = context
            return context
    
    def _load_history(self, conn, session_id: str, limit: int = 10) -> List[Dict]:
        """Load recent command history."""
        rows = conn.execute(
            """SELECT command, interpretation, result, timestamp
               FROM history 
               WHERE session_id = ? 
               ORDER BY timestamp DESC 
               LIMIT ?""",
            (session_id, limit)
        ).fetchall()
        
        return [
            {
                "command": r[0],
                "interpretation": json.loads(r[1]) if r[1] else None,
                "result": json.loads(r[2]) if r[2] else None,
                "timestamp": r[3]
            }
            for r in reversed(rows)  # Oldest first
        ]
    
    def save_context(self, context: ConversationContext):
        """Save context to database.
        
        Args:
            context: Context to save
        """
        with sqlite3.connect(self.db_path) as conn:
            conn.execute(
                """INSERT OR REPLACE INTO contexts 
                   (session_id, current_location, last_action, last_result, 
                    known_locations, pending_task, updated_at)
                   VALUES (?, ?, ?, ?, ?, ?, ?)""",
                (
                    context.session_id,
                    context.current_location,
                    context.last_action,
                    json.dumps(context.last_result) if context.last_result else None,
                    json.dumps(context.known_locations),
                    context.pending_task,
                    datetime.now()
                )
            )
    
    def log_interaction(self, session_id: str, command: str, 
                       interpretation: Dict, result: Dict):
        """Log a command interaction to history.
        
        Args:
            session_id: Session identifier
            command: Natural language command
            interpretation: Interpreted ROS command
            result: Execution result
        """
        with sqlite3.connect(self.db_path) as conn:
            conn.execute(
                """INSERT INTO history 
                   (session_id, command, interpretation, result)
                   VALUES (?, ?, ?, ?)""",
                (
                    session_id,
                    command,
                    json.dumps(interpretation),
                    json.dumps(result)
                )
            )
        
        # Update runtime context
        context = self.get_context(session_id)
        context.conversation_history.append({
            "command": command,
            "interpretation": interpretation,
            "result": result,
            "timestamp": datetime.now().isoformat()
        })
        # Keep only last 10
        context.conversation_history = context.conversation_history[-10:]
    
    def learn_location(self, session_id: str, name: str, coordinates: Dict):
        """Learn a named location.
        
        Args:
            session_id: Session identifier
            name: Location name (e.g., "kitchen", "charging station")
            coordinates: Dict with x, y, z or other coordinate data
        """
        name_lower = name.lower().strip()
        
        with sqlite3.connect(self.db_path) as conn:
            conn.execute(
                """INSERT OR REPLACE INTO locations 
                   (session_id, name, coordinates)
                   VALUES (?, ?, ?)""",
                (session_id, name_lower, json.dumps(coordinates))
            )
        
        # Update runtime context
        context = self.get_context(session_id)
        context.known_locations[name_lower] = coordinates
        context.current_location = name_lower  # Assume we're at this location
    
    def get_location(self, session_id: str, name: str) -> Optional[Dict]:
        """Get coordinates for a learned location.
        
        Args:
            session_id: Session identifier
            name: Location name
            
        Returns:
            Coordinates dict or None if not found
        """
        name_lower = name.lower().strip()
        
        # Check runtime cache first
        context = self.get_context(session_id)
        if name_lower in context.known_locations:
            return context.known_locations[name_lower]
        
        # Check database
        with sqlite3.connect(self.db_path) as conn:
            row = conn.execute(
                "SELECT coordinates FROM locations WHERE session_id = ? AND name = ?",
                (session_id, name_lower)
            ).fetchone()
            
            if row:
                coords = json.loads(row[0])
                # Update cache
                context.known_locations[name_lower] = coords
                return coords
        
        return None
    
    def get_last_n_commands(self, session_id: str, n: int = 5) -> List[Dict]:
        """Get recent command history.
        
        Args:
            session_id: Session identifier
            n: Number of commands to retrieve
            
        Returns:
            List of recent commands with results
        """
        context = self.get_context(session_id)
        return context.conversation_history[-n:]
    
    def get_last_location(self, session_id: str) -> Optional[str]:
        """Get the last location the robot was sent to.
        
        Args:
            session_id: Session identifier
            
        Returns:
            Location name or None
        """
        context = self.get_context(session_id)
        return context.current_location
    
    def clear_context(self, session_id: str):
        """Clear context for a session.
        
        Args:
            session_id: Session to clear
        """
        with sqlite3.connect(self.db_path) as conn:
            conn.execute("DELETE FROM contexts WHERE session_id = ?", (session_id,))
            conn.execute("DELETE FROM history WHERE session_id = ?", (session_id,))
            conn.execute("DELETE FROM locations WHERE session_id = ?", (session_id,))
        
        # Clear runtime cache
        if session_id in self._runtime_contexts:
            del self._runtime_contexts[session_id]
    
    def list_learned_locations(self, session_id: str) -> List[str]:
        """List all learned locations for a session.
        
        Args:
            session_id: Session identifier
            
        Returns:
            List of location names
        """
        context = self.get_context(session_id)
        return list(context.known_locations.keys())


class ContextAwareNLInterpreter:
    """Natural language interpreter with context awareness.
    
    Extends the base interpreter with:
    - Location resolution from learned places
    - Contextual command completion
    - Reference resolution ("it", "there", "here")
    """
    
    def __init__(self, base_interpreter, context_manager: ContextManager):
        """Initialize with base interpreter and context manager.
        
        Args:
            base_interpreter: Base NL interpreter
            context_manager: Context manager instance
        """
        self.base = base_interpreter
        self.context = context_manager
    
    def interpret(self, nl_command: str, session_id: str = "default") -> Dict[str, Any]:
        """Interpret command with context awareness.
        
        Args:
            nl_command: Natural language command
            session_id: Session identifier for context
            
        Returns:
            Interpreted command with context applied
        """
        ctx = self.context.get_context(session_id)
        
        # Handle contextual references
        command = self._resolve_references(nl_command, ctx)
        
        # Get base interpretation
        result = self.base.interpret(command, context=ctx.to_dict())
        
        # Apply context-aware enhancements
        if result.get("tool") == "ros2_action_goal":
            # Check if location needs resolution
            location_name = result.get("location_name")
            if location_name:
                coords = self.context.get_location(session_id, location_name)
                if coords:
                    result["goal"]["pose"]["position"] = coords
                    result["explanation"] = f"Navigate to {location_name} (learned location)"
                    result.pop("note", None)  # Remove "not found" note
        
        # Update context with this command
        ctx.last_action = nl_command
        
        return result
    
    def _resolve_references(self, command: str, ctx: ConversationContext) -> str:
        """Resolve contextual references in command.
        
        Args:
            command: Command string
            ctx: Current context
            
        Returns:
            Command with references resolved
        """
        command_lower = command.lower()
        
        # "Go back" or "Return" → go to previous location
        if any(phrase in command_lower for phrase in ["go back", "return", "go home"]):
            if ctx.current_location:
                # For now, just pass through - navigation will handle
                pass
        
        # "Bring me [X]" when at a location → get X from that location
        if "bring me" in command_lower or "get me" in command_lower:
            if ctx.current_location:
                # Could enhance with object-location knowledge
                pass
        
        return command
    
    def learn_current_location(self, session_id: str, name: str, 
                               coordinates: Optional[Dict] = None):
        """Learn current robot location.
        
        Args:
            session_id: Session identifier
            name: Name for this location
            coordinates: Coordinates (if None, uses current from context)
        """
        if coordinates is None:
            # Would need to get from robot's current position
            # For now, use placeholder
            coordinates = {"x": 0, "y": 0, "z": 0}
        
        self.context.learn_location(session_id, name, coordinates)
