"""opc-milestone-tracker scripts package.

This package contains scripts for the milestone tracker skill.
All operations are local-only. No network calls or external sharing.
"""

from .init import main as init
from .detect import main as detect
# NOTE: notify module reserved for future versions
# Currently all milestone notifications are handled through local journal entries only
# from .notify import main as notify

__all__ = ["init", "detect"]
