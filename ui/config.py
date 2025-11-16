"""
Configuration file for NodeArena Platform
"""
from pathlib import Path

# Paths
BASE_DIR = Path(__file__).parent.parent
CHALLENGES_DIR = BASE_DIR / 'challenges'
CHALLENGES_FILE = CHALLENGES_DIR / 'challenges.json'
STATE_FILE = CHALLENGES_DIR / 'state.json'

# UI Configuration
APP_TITLE = 'NodeArena'
APP_PORT = 8080
PRIMARY_COLOR = "#d66c48"  # Deep orange accent

# Theme Colors (Dark Neutral Theme)
THEME_COLORS = {
    'primary': '#d66c48',    # Deep orange accent
    'secondary': '#e88b67',  # Lighter orange
    'accent': '#d66c48',     # Deep orange
    'dark': '#262624',       # Dark neutral background
    'positive': '#06d6a0',   # Teal green
    'negative': '#ef476f',   # Pink-red
    'info': '#8a8a88',       # Light gray for info
    'warning': '#ffd166'     # Golden yellow
}

# Dark Theme Background Colors
BACKGROUND_COLOR = '#262624'    # Dark neutral background
CARD_BACKGROUND = '#30302e'     # Lighter neutral for cards
SURFACE_COLOR = '#2a2a28'       # Surface elements

# User Configuration
USER_NAME = "NodeArena Player"

# Difficulty Colors
DIFFICULTY_COLORS = {
    'Easy': 'positive',    # Teal green
    'Medium': 'warning',   # Golden yellow
    'Hard': 'negative'     # Pink-red
}

# Module Card Dimensions
MODULE_CARD_WIDTH = '320px'
MODULE_CARD_HEIGHT = '320px'