# Minimal Sphinx configuration for API docs
project = 'Agent ROS Bridge'
copyright = '2026, Agent ROS Bridge Team'
author = 'Agent ROS Bridge Team'

version = '0.5.0'
release = '0.5.0'

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'sphinx.ext.napoleon',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

html_theme = 'alabaster'
html_static_path = ['_static']

# Auto-generate module docs
import os
import sys
sys.path.insert(0, os.path.abspath('..'))
