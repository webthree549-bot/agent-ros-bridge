"""opc-journal-core scripts package.

This package contains scripts for the journal core skill.
"""

from .init import main as init
from .record import main as record
from .search import main as search
from .export import main as export

__all__ = ["init", "record", "search", "export"]
