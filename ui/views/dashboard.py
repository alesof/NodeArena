"""
Dashboard view
"""
from nicegui import ui
from typing import Dict, List, Callable
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent.parent))
from config import USER_NAME
from models.state import get_solved_count, get_total_points
from models.badges import (
    get_selected_badge, get_selected_title,
    get_badge_by_id, get_title_by_id,
    get_unlocked_badges, get_unlocked_titles,
    BADGES, TITLES
)


def show_dashboard(
    challenges: List[Dict],
    state: Dict,
    on_profile_click: Callable = None
):
    selected_badge_id = get_selected_badge(state)
    selected_title_id = get_selected_title(state)
    selected_badge = get_badge_by_id(selected_badge_id) if selected_badge_id else None
    selected_title = get_title_by_id(selected_title_id) if selected_title_id else get_title_by_id('beginner')
    
    unlocked_badges = get_unlocked_badges(state, challenges)
    unlocked_titles = get_unlocked_titles(state, challenges)
    
    solved_count = get_solved_count(challenges, state)
    total_points = get_total_points(challenges, state)
    
    current_level = (solved_count // 5) + 1
    challenges_in_level = solved_count % 5
    xp_in_level = challenges_in_level * 100
    xp_for_next_level = 500
    progress_percentage = (challenges_in_level / 5) * 100
    
    with ui.row().classes('w-full justify-center'):
        with ui.column().classes('w-full max-w-4xl p-4 gap-6'):
            ui.label('Dashboard').classes('text-3xl font-bold mb-4 text-white')
            
            # User Profile Card
            with ui.card().classes('w-full p-6'):
                with ui.row().classes('items-center gap-4'):
                    if selected_badge:
                        with ui.element('div').classes('rounded-full flex items-center justify-center').style(
                            f'width: 80px; height: 80px; background-color: {selected_badge["color"]};'
                        ):
                            ui.icon(selected_badge['icon']).classes('text-white').style('font-size: 40px;')
                    else:
                        ui.icon('account_circle', size='xl').classes('text-primary')
                    
                    with ui.column().classes('gap-1'):
                        ui.label('Welcome back!').classes('text-lg').style('color: #9aa0a6')
                        with ui.row().classes('items-center gap-2'):
                            ui.label(USER_NAME).classes('text-2xl font-bold text-white')
                            if selected_title:
                                ui.badge(selected_title['name']).classes('px-3 py-1.5 text-base font-bold').style(
                                    f'background-color: {selected_title["color"]}; box-shadow: 0 2px 6px rgba(0,0,0,0.3);'
                                )
            
            # Level and Progress Card
            with ui.card().classes('w-full p-6'):
                with ui.column().classes('items-center w-full gap-3'):
                    with ui.row().classes('items-end gap-3'):
                        ui.label(str(current_level)).classes('text-6xl font-bold text-white').style('line-height: 1;')
                        with ui.column().classes('gap-0 pb-1'):
                            ui.label('LEVEL').classes('text-xs font-bold').style('color: #14b8a6; letter-spacing: 1px;')
                            ui.label(f'{total_points:,} XP').classes('text-sm font-semibold').style('color: #9aa0a6')
                    
                    with ui.column().classes('gap-1 w-full max-w-md'):
                        with ui.row().classes('w-full items-center justify-between'):
                            ui.label(f'{xp_in_level} / {xp_for_next_level} XP').classes('text-xs font-semibold').style('color: #14b8a6')
                            ui.label(f'Level {current_level + 1}').classes('text-xs').style('color: #9aa0a6')
                        
                        with ui.element('div').classes('w-full h-2 rounded-full overflow-hidden').style('background-color: #2d3748;'):
                            ui.element('div').classes('h-full rounded-full transition-all duration-500').style(
                                f'width: {progress_percentage}%; background: #14b8a6;'
                            )
                    
                    with ui.row().classes('items-center gap-2 mt-4'):
                        ui.icon('emoji_events').classes('text-primary').style('font-size: 24px;')
                        ui.label(f'{solved_count} / {len(challenges)} Challenges Solved').classes('text-lg font-semibold text-white')
                
                ui.separator().classes('my-6')
                
                # Achievement Statistics (Numbers Only)
                with ui.column().classes('w-full gap-4 items-center'):
                    with ui.row().classes('items-center gap-2'):
                        ui.icon('military_tech').classes('text-yellow-500').style('font-size: 28px;')
                        ui.label('Achievements Unlocked').classes('text-2xl font-bold text-white')
                        ui.icon('military_tech').classes('text-yellow-500').style('font-size: 28px;')
                    
                    with ui.row().classes('gap-12 justify-center mt-4'):
                        # Badges Count
                        with ui.column().classes('gap-3 items-center'):
                            ui.label('BADGES').classes('text-xs font-bold').style('color: #14b8a6; letter-spacing: 2px;')
                            ui.label(f'{len(unlocked_badges)} / {len(BADGES)}').classes('text-3xl font-bold').style('color: #14b8a6')
                        
                        # Titles Count
                        with ui.column().classes('gap-3 items-center'):
                            ui.label('TITLES').classes('text-xs font-bold').style('color: #14b8a6; letter-spacing: 2px;')
                            ui.label(f'{len(unlocked_titles)} / {len(TITLES)}').classes('text-3xl font-bold').style('color: #14b8a6')