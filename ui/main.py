"""
ROS2 CTF Platform - Main Entry Point
"""
from nicegui import ui
from pathlib import Path
import sys

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from config import APP_TITLE, APP_PORT, PRIMARY_COLOR
from models.challenge import load_challenges
from models.state import load_challenge_state
from utils.navigation import nav_state
from components.header import create_header
from views.dashboard import show_dashboard
from views.library import show_library
from views.module import show_module
from views.challenge import show_challenge
from views.profile import show_profile


class CTFApp:
    """Main CTF Platform Application"""
    
    def __init__(self):
        self.challenges = load_challenges()
        self.state = load_challenge_state()
        self.header_container = None
        self.content_container = None
    
    def reload_state(self):
        """Reload the state from file"""
        self.state = load_challenge_state()
    
    def render(self):
        """Render the current view based on navigation state"""
        self.content_container.clear()
        
        with self.content_container:
            if nav_state.page == 'dashboard':
                show_dashboard(
                    self.challenges,
                    self.state,
                    on_profile_click=self.navigate_to_profile
                )
            
            elif nav_state.page == 'library':
                show_library(
                    self.challenges,
                    self.state,
                    on_module_click=self.navigate_to_module
                )
            
            elif nav_state.page == 'profile':
                show_profile(
                    self.challenges,
                    self.state,
                    on_back_click=self.navigate_to_dashboard,
                    on_selection_change=self.on_profile_change
                )
            
            elif nav_state.page == 'module':
                show_module(
                    nav_state.module,
                    self.challenges,
                    self.state,
                    on_back_click=self.navigate_to_library,
                    on_challenge_click=self.navigate_to_challenge
                )
            
            elif nav_state.page == 'challenge':
                challenge = next(
                    (ch for ch in self.challenges if ch['id'] == nav_state.challenge_id),
                    None
                )
                if challenge:
                    show_challenge(
                        challenge,
                        self.challenges,
                        self.state,
                        on_back_click=self.navigate_to_module_from_challenge,
                        on_solve_callback=self.on_challenge_solved
                    )
    
    def navigate_to_dashboard(self):
        """Navigate to dashboard"""
        nav_state.navigate_to_dashboard()
        self.render()
    
    def navigate_to_library(self):
        """Navigate to library"""
        nav_state.navigate_to_library()
        self.render()
    
    def navigate_to_profile(self):
        """Navigate to profile"""
        nav_state.navigate_to_profile()
        self.render()
    
    def navigate_to_module(self, category: str):
        """Navigate to a specific module"""
        nav_state.navigate_to_module(category)
        self.render()
    
    def navigate_to_module_from_challenge(self):
        """Navigate back to module from challenge"""
        if nav_state.module:
            self.navigate_to_module(nav_state.module)
    
    def navigate_to_challenge(self, challenge: dict):
        """Navigate to a specific challenge"""
        nav_state.navigate_to_challenge(challenge['id'], challenge['category'])
        self.render()
    
    def on_challenge_solved(self):
        """Callback when a challenge is solved"""
        self.reload_state()
        self.navigate_to_module_from_challenge()
    
    def on_profile_change(self):
        """Callback when profile is updated"""
        self.reload_state()
        self.render()
    
    def init_ui(self):
        """Initialize the UI"""
        # Enable dark mode and set colors
        ui.dark_mode().enable()
        ui.colors(
            primary=PRIMARY_COLOR,
            secondary='#1e4d5c',
            accent='#2a6f82',
            dark='#0d1b1e',
            positive='#21BA45',
            negative='#C10015',
            info='#31CCEC',
            warning='#F2C037'
        )
        
        # Add custom dark theme CSS
        ui.add_head_html('''
            <style>
                body {
                    background-color: #0d1b1e !important;
                }
                .q-card {
                    background-color: #1a2e35 !important;
                }
                .q-page {
                    background-color: #0d1b1e !important;
                }
                .q-header {
                    background-color: #173843 !important;
                }
            </style>
        ''')
        
        # Create header
        self.header_container = ui.header().classes('items-center justify-between bg-primary')
        with self.header_container:
            create_header(
                on_dashboard_click=self.navigate_to_dashboard,
                on_library_click=self.navigate_to_library,
                on_profile_click=self.navigate_to_profile
            )
        
        # Create content container
        self.content_container = ui.column().classes('w-full')
        
        # Show initial page
        self.render()


def main():
    """Main entry point"""
    app = CTFApp()
    app.init_ui()
    ui.run(title=APP_TITLE, port=APP_PORT)


if __name__ in {"__main__", "__mp_main__"}:
    main()