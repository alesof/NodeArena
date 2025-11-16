"""Views package for different pages"""
from .dashboard import show_dashboard
from .library import show_library
from .module import show_module
from .challenge import show_challenge
from .profile import show_profile

__all__ = [
    'show_dashboard',
    'show_library',
    'show_module',
    'show_challenge',
    'show_profile',
]
