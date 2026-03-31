"""opc-pattern-recognition scripts package.

This package contains scripts for the pattern recognition skill.
"""

from .init import main as init
from .analyze import main as analyze
from .detect_outliers import main as detect_outliers

__all__ = ["init", "analyze", "detect_outliers"]
