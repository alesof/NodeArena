"""
Header component with navigation
"""
from nicegui import ui


def create_header(on_dashboard_click, on_library_click, on_profile_click):
    """Create the application header with navigation"""
    with ui.row().classes('items-center justify-between w-full'):
        # Logo/Name on the left
        ui.label('Node Arena - Alpha v0.0.1').classes('text-2xl font-bold cursor-pointer').on('click', on_dashboard_click)
        
        # Navigation menu on the right
        with ui.row().classes('items-center gap-4'):
            ui.button('Dashboard', on_click=on_dashboard_click).props('flat').classes('text-white')
            ui.button('Library', on_click=on_library_click).props('flat').classes('text-white')
            ui.button('Profile', icon='person', on_click=on_profile_click).props('flat').classes('text-white')