"""
Module view - shows challenges in a specific module
"""
from nicegui import ui
from typing import Dict, List, Callable
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent.parent))
from models.challenge import get_challenges_by_category
from models.state import get_module_stats
from components.cards import create_challenge_card, create_progress_card


def show_module(
    category: str,
    challenges: List[Dict],
    state: Dict,
    on_back_click: Callable,
    on_challenge_click: Callable[[Dict], None]
):
    """Display the module page with its challenges"""
    module_challenges = get_challenges_by_category(category, challenges)
    stats = get_module_stats(category, challenges, state)
    
    with ui.row().classes('w-full justify-center'):
        with ui.column().classes('w-full max-w-4xl p-4 gap-4'):
            # Back button and module header
            with ui.row().classes('items-center gap-4 mb-4'):
                ui.button(icon='arrow_back', on_click=on_back_click).props('flat')
                with ui.column().classes('gap-1 flex-1'):
                    ui.label(category).classes('text-3xl font-bold text-white')
                    ui.label(
                        f"{stats['solved']}/{stats['total']} Challenges Completed • "
                        f"{stats['earned_points']}/{stats['total_points']} Points"
                    ).classes('text-lg').style('color: #9aa0a6')
            
            # Progress card
            create_progress_card(stats)
            
            # Challenges
            for challenge in module_challenges:
                create_challenge_card(
                    challenge,
                    challenges,
                    state,
                    lambda ch=challenge: on_challenge_click(ch)
                )