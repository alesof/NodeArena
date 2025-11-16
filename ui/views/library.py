"""
Library view - shows all challenge modules
"""
from nicegui import ui
from typing import Dict, List, Callable
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent.parent))
from models.challenge import get_categories
from models.state import get_module_stats
from components.cards import create_module_card


def show_library(
    challenges: List[Dict],
    state: Dict,
    on_module_click: Callable[[str], None]
):
    """Display the library page with all modules"""
    categories = get_categories(challenges)
    
    with ui.row().classes('w-full justify-center'):
        with ui.column().classes('w-full p-4 gap-4').style('max-width: 1056px;'):
            with ui.row().classes('items-center justify-between w-full mb-4'):
                ui.label('Challenge Library').classes('text-3xl font-bold text-white')
                ui.label(f'{len(categories)} Modules').classes('text-lg').style('color: #9aa0a6')
            
            # Grid layout for modules - max 3 per row
            with ui.element('div').classes('grid gap-6 w-full').style(
                'grid-template-columns: repeat(auto-fit, 320px); justify-content: center;'
            ):
                for category in categories:
                    stats = get_module_stats(category, challenges, state)
                    create_module_card(
                        category,
                        stats,
                        lambda cat=category: on_module_click(cat)
                    )