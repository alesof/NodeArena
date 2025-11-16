"""
Navigation state management
"""
from typing import Optional


class NavigationState:
    """Manages the current view state"""
    
    def __init__(self):
        self.page = 'dashboard'
        self.challenge_id: Optional[int] = None
        self.module: Optional[str] = None
    
    def navigate_to_dashboard(self):
        """Navigate to dashboard"""
        self.page = 'dashboard'
        self.challenge_id = None
        self.module = None
    
    def navigate_to_library(self):
        """Navigate to library"""
        self.page = 'library'
        self.challenge_id = None
        self.module = None
    
    def navigate_to_profile(self):
        """Navigate to profile"""
        self.page = 'profile'
        self.challenge_id = None
        self.module = None
    
    def navigate_to_module(self, category: str):
        """Navigate to module view"""
        self.page = 'module'
        self.module = category
        self.challenge_id = None
    
    def navigate_to_challenge(self, challenge_id: int, category: str):
        """Navigate to challenge view"""
        self.page = 'challenge'
        self.challenge_id = challenge_id
        if self.module is None:
            self.module = category


# Global navigation state instance
nav_state = NavigationState()