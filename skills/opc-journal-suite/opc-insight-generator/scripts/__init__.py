"""opc-insights-generator scripts package.

This package contains scripts for the insights generator skill.
"""

from .init import main as init
from .daily_summary import main as daily_summary

__all__ = ["init", "daily_summary"]
