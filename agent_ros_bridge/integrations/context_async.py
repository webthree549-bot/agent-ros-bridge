"""Async context management for natural language conversations.

Async version of context.py using aiosqlite for non-blocking DB operations.
"""

import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import aiosqlite


@dataclass
class ConversationContext:
    """Runtime context for a conversation session."""

    session_id: str
    current_location: str | None = None
    last_action: str | None = None
    last_result: dict | None = None
    known_locations: dict[str, dict] = None
    pending_task: str | None = None
    conversation_history: list[dict] = None

    def __post_init__(self):
        if self.known_locations is None:
            self.known_locations = {}
        if self.conversation_history is None:
            self.conversation_history = []

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            "session_id": self.session_id,
            "current_location": self.current_location,
            "last_action": self.last_action,
            "last_result": self.last_result,
            "known_locations": self.known_locations,
            "pending_task": self.pending_task,
            "conversation_history": self.conversation_history[:10],
        }


class AsyncContextManager:
    """Async context manager with aiosqlite.

    Non-blocking version of ContextManager for better performance
    in async applications.
    """

    def __init__(self, db_path: str = ".agent_ros_context.db"):
        self.db_path = Path(db_path)
        self._runtime_contexts: dict[str, ConversationContext] = {}
        self._initialized = False

    async def _init_db(self):
        """Initialize SQLite database tables asynchronously."""
        if self._initialized:
            return

        async with aiosqlite.connect(self.db_path) as conn:
            await conn.execute("""
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

            await conn.execute("""
                CREATE TABLE IF NOT EXISTS history (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    session_id TEXT,
                    command TEXT,
                    interpretation TEXT,
                    result TEXT,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)

            await conn.execute("""
                CREATE TABLE IF NOT EXISTS locations (
                    session_id TEXT,
                    name TEXT,
                    coordinates TEXT,
                    learned_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    PRIMARY KEY (session_id, name)
                )
            """)

            await conn.commit()

        self._initialized = True

    async def get_context(self, session_id: str) -> ConversationContext:
        """Get or create context for a session asynchronously."""
        await self._init_db()

        # Check runtime cache first
        if session_id in self._runtime_contexts:
            return self._runtime_contexts[session_id]

        # Load from database
        async with aiosqlite.connect(self.db_path) as conn:
            conn.row_factory = aiosqlite.Row

            async with conn.execute(
                """SELECT current_location, last_action, last_result, 
                          known_locations, pending_task
                   FROM contexts WHERE session_id = ?""",
                (session_id,),
            ) as cursor:
                row = await cursor.fetchone()

            if row:
                context = ConversationContext(
                    session_id=session_id,
                    current_location=row["current_location"],
                    last_action=row["last_action"],
                    last_result=json.loads(row["last_result"]) if row["last_result"] else None,
                    known_locations=(
                        json.loads(row["known_locations"]) if row["known_locations"] else {}
                    ),
                    pending_task=row["pending_task"],
                )
            else:
                context = ConversationContext(session_id=session_id)

            # Load recent history
            context.conversation_history = await self._load_history(conn, session_id, limit=10)

            self._runtime_contexts[session_id] = context
            return context

    async def _load_history(self, conn, session_id: str, limit: int = 10) -> list[dict]:
        """Load recent command history asynchronously."""
        history = []

        async with conn.execute(
            """SELECT command, interpretation, result, timestamp
               FROM history 
               WHERE session_id = ? 
               ORDER BY timestamp DESC 
               LIMIT ?""",
            (session_id, limit),
        ) as cursor:
            async for row in cursor:
                history.append(
                    {
                        "command": row[0],
                        "interpretation": json.loads(row[1]) if row[1] else None,
                        "result": json.loads(row[2]) if row[2] else None,
                        "timestamp": row[3],
                    }
                )

        return list(reversed(history))  # Oldest first

    async def save_context(self, context: ConversationContext):
        """Save context to database asynchronously."""
        await self._init_db()

        async with aiosqlite.connect(self.db_path) as conn:
            await conn.execute(
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
                    datetime.now(),
                ),
            )
            await conn.commit()

    async def log_interaction(
        self, session_id: str, command: str, interpretation: dict, result: dict
    ):
        """Log a command interaction to history asynchronously."""
        await self._init_db()

        async with aiosqlite.connect(self.db_path) as conn:
            await conn.execute(
                """INSERT INTO history 
                   (session_id, command, interpretation, result)
                   VALUES (?, ?, ?, ?)""",
                (session_id, command, json.dumps(interpretation), json.dumps(result)),
            )
            await conn.commit()

        # Update runtime context
        context = await self.get_context(session_id)
        context.conversation_history.append(
            {
                "command": command,
                "interpretation": interpretation,
                "result": result,
                "timestamp": datetime.now().isoformat(),
            }
        )
        context.conversation_history = context.conversation_history[-10:]

    async def learn_location(self, session_id: str, name: str, coordinates: dict):
        """Learn a named location asynchronously."""
        await self._init_db()

        name_lower = name.lower().strip()

        async with aiosqlite.connect(self.db_path) as conn:
            await conn.execute(
                """INSERT OR REPLACE INTO locations 
                   (session_id, name, coordinates)
                   VALUES (?, ?, ?)""",
                (session_id, name_lower, json.dumps(coordinates)),
            )
            await conn.commit()

        # Update runtime context
        context = await self.get_context(session_id)
        context.known_locations[name_lower] = coordinates
        context.current_location = name_lower

    async def get_location(self, session_id: str, name: str) -> dict | None:
        """Get coordinates for a learned location asynchronously."""
        await self._init_db()

        name_lower = name.lower().strip()

        # Check runtime cache first
        context = await self.get_context(session_id)
        if name_lower in context.known_locations:
            return context.known_locations[name_lower]

        # Check database
        async with aiosqlite.connect(self.db_path) as conn:
            conn.row_factory = aiosqlite.Row

            async with conn.execute(
                "SELECT coordinates FROM locations WHERE session_id = ? AND name = ?",
                (session_id, name_lower),
            ) as cursor:
                row = await cursor.fetchone()

                if row:
                    coords = json.loads(row["coordinates"])
                    # Update cache
                    context.known_locations[name_lower] = coords
                    return coords

        return None

    async def get_last_n_commands(self, session_id: str, n: int = 5) -> list[dict]:
        """Get recent command history asynchronously."""
        context = await self.get_context(session_id)
        return context.conversation_history[-n:]

    async def clear_context(self, session_id: str):
        """Clear context for a session asynchronously."""
        await self._init_db()

        async with aiosqlite.connect(self.db_path) as conn:
            await conn.execute("DELETE FROM contexts WHERE session_id = ?", (session_id,))
            await conn.execute("DELETE FROM history WHERE session_id = ?", (session_id,))
            await conn.execute("DELETE FROM locations WHERE session_id = ?", (session_id,))
            await conn.commit()

        # Clear runtime cache
        if session_id in self._runtime_contexts:
            del self._runtime_contexts[session_id]
