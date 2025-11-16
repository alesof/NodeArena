"""
Reusable card components
"""
from nicegui import ui
from typing import Dict, List, Callable
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent.parent))
from config import DIFFICULTY_COLORS, MODULE_CARD_WIDTH, MODULE_CARD_HEIGHT
from models.state import is_challenge_locked, is_challenge_solved


def create_challenge_card(
    challenge: Dict,
    challenges: List[Dict],
    state: Dict,
    on_click: Callable
):
    is_locked = is_challenge_locked(challenge['id'], challenges, state)
    challenge_solved = is_challenge_solved(challenge['id'], state)
    
    def handle_click():
        if not is_locked:
            on_click()
    
    card_classes = 'w-full transition-all'
    
    if is_locked:
        card_classes += ' opacity-50 cursor-not-allowed'
    else:
        card_classes += ' cursor-pointer hover:shadow-[0_0_20px_rgba(214,108,72,0.6)]'
    
    border_style = 'border: 2px solid transparent; transition: all 0.3s ease;'
    
    with ui.card().classes(card_classes).on('click', handle_click).style(border_style) as card:
        if not is_locked:
            card.on('mouseenter', lambda c=card: c.style(add='border-color: #d66c48;'))
            card.on('mouseleave', lambda c=card: c.style(add='border-color: transparent;'))
        
        with ui.row().classes('w-full items-start justify-between gap-4'):
            with ui.column().classes('gap-1 flex-1 max-w-[70%]'):
                with ui.row().classes('items-center gap-2'):
                    ui.label(challenge['title']).classes('text-xl font-bold').style('color: #e8eaed')
                    if is_locked:
                        ui.icon('lock').classes('text-gray-500')
                    elif challenge_solved:
                        ui.icon('check_circle').style('color: #06d6a0')
                ui.label(challenge['description']).style('color: #b0b0ae')
                with ui.row().classes('gap-2'):
                    ui.label(f"Category: {challenge['category']}").classes('text-sm').style('color: #8a8a88')
            
            with ui.column().classes('items-end gap-2 flex-shrink-0'):
                ui.badge(challenge['difficulty'], color=DIFFICULTY_COLORS[challenge['difficulty']]).classes('text-base px-3 py-1')
                with ui.column().classes('items-end gap-0'):
                    ui.label(f"{challenge['points']}").classes('text-5xl font-bold').style('color: #d66c48; line-height: 1;')
                    ui.label('XP').classes('text-2xl font-bold').style('color: #d66c48')


def create_module_card(
    category: str,
    stats: Dict,
    on_click: Callable
):
    with ui.card().classes('transition-all cursor-pointer hover:shadow-[0_0_20px_rgba(214,108,72,0.6)]').style(
        f'aspect-ratio: 1; width: {MODULE_CARD_WIDTH}; height: {MODULE_CARD_HEIGHT}; padding: 2rem; '
        f'border: 2px solid transparent; transition: all 0.3s ease;'
    ).on('click', on_click) as card:
        card.on('mouseenter', lambda c=card: c.style(add='border-color: #d66c48;'))
        card.on('mouseleave', lambda c=card: c.style(add='border-color: transparent;'))
        
        with ui.column().classes('w-full h-full justify-between gap-3'):
            with ui.row().classes('w-full justify-center'):
                with ui.element('div').classes('rounded-full flex items-center justify-center').style(
                    'width: 64px; height: 64px; '
                    'background: linear-gradient(135deg, #d66c48 0%, #e88b67 100%); '
                    'box-shadow: 0 4px 12px rgba(214, 108, 72, 0.4);'
                ):
                    ui.icon('folder').classes('text-white').style('font-size: 36px;')
            
            with ui.column().classes('gap-1 w-full'):
                ui.label(category).classes('font-bold text-center').style(
                    'font-size: 1.125rem; line-height: 1.3; color: #e8eaed; '
                    'overflow-wrap: break-word; word-break: break-word;'
                )
                ui.label(f"{stats['total']} Challenges").classes('text-center').style(
                    'font-size: 0.875rem; color: #b0b0ae;'
                )
            
            with ui.column().classes('gap-2 w-full'):
                ui.label(f"{stats['solved']}/{stats['total']}").classes('font-bold text-center').style(
                    'font-size: 1.75rem; color: #d66c48;'
                )
                with ui.element('div').classes('w-full rounded-full overflow-hidden').style(
                    'height: 8px; background-color: #4a4a48;'
                ):
                    ui.element('div').classes('h-full').style(
                        f'width: {stats["progress"]:.0f}%; '
                        'background: linear-gradient(90deg, #d66c48 0%, #e88b67 100%);'
                    )
                ui.label(f"{stats['progress']:.0f}%").classes('text-center font-semibold').style(
                    'font-size: 0.875rem; color: #e8eaed;'
                )
            
            with ui.column().classes('gap-1 w-full'):
                ui.label(f"{stats['earned_points']}/{stats['total_points']} XP").classes('text-center').style(
                    'font-size: 0.875rem; color: #b0b0ae;'
                )
                with ui.element('div').classes('w-full rounded-full overflow-hidden').style(
                    'height: 6px; background-color: #4a4a48;'
                ):
                    points_percentage = (stats['earned_points'] / stats['total_points'] * 100) if stats['total_points'] > 0 else 0
                    ui.element('div').classes('h-full').style(
                        f'width: {points_percentage:.0f}%; background-color: #06d6a0;'
                    )


def create_stats_card(label: str, value: str):
    with ui.card().classes('flex-1 p-4').style('border-width: 2px;'):
        ui.label(label).classes('text-sm').style('color: #b0b0ae')
        ui.label(value).classes('text-3xl font-bold').style('color: #d66c48')


def create_progress_card(stats: Dict):
    with ui.card().classes('w-full').style('border-width: 2px;'):
        with ui.column().classes('gap-2'):
            ui.label('Module Progress').classes('text-lg font-bold').style('color: #e8eaed')
            with ui.row().classes('w-full items-center gap-2'):
                with ui.element('div').classes('flex-1 h-3 rounded-full overflow-hidden').style(
                    'background-color: #4a4a48;'
                ):
                    ui.element('div').classes('h-full').style(
                        f'width: {stats["progress"]:.0f}%; '
                        'background: linear-gradient(90deg, #d66c48 0%, #e88b67 100%);'
                    )
                ui.label(f"{stats['progress']:.0f}%").classes('text-sm font-semibold').style('color: #e8eaed')
"""
Profile view - badge and title selection
"""
from nicegui import ui
from typing import Dict, List, Callable
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent.parent))
from models.badges import (
    BADGES, TITLES,
    get_unlocked_badges, get_unlocked_titles,
    get_selected_badge, get_selected_title,
    set_selected_badge, set_selected_title
)
from models.state import save_challenge_state


def show_profile(
    challenges: List[Dict],
    state: Dict,
    on_back_click: Callable,
    on_selection_change: Callable
):
    unlocked_badges = get_unlocked_badges(state, challenges)
    unlocked_titles = get_unlocked_titles(state, challenges)
    selected_badge_id = get_selected_badge(state)
    selected_title_id = get_selected_title(state)
    
    with ui.row().classes('w-full justify-center'):
        with ui.column().classes('w-full max-w-6xl p-4 gap-6'):
            with ui.row().classes('items-center gap-4 mb-4'):
                ui.button(icon='arrow_back', on_click=on_back_click).props('flat')
                ui.label('Profile & Achievements').classes('text-3xl font-bold text-white')
            
            with ui.card().classes('w-full'):
                ui.label('Badges').classes('text-2xl font-bold mb-4 text-white')
                ui.label(f'Unlocked: {len(unlocked_badges)}/{len(BADGES)}').classes('text-lg mb-4').style('color: #9aa0a6')
                
                with ui.element('div').classes('grid gap-4 w-full').style(
                    'grid-template-columns: repeat(auto-fill, minmax(250px, 1fr));'
                ):
                    for badge_id, badge in BADGES.items():
                        is_unlocked = any(b['id'] == badge_id for b in unlocked_badges)
                        is_selected = badge_id == selected_badge_id
                        
                        def select_badge(bid=badge_id, unlocked=is_unlocked):
                            if unlocked:
                                set_selected_badge(state, bid)
                                save_challenge_state(state)
                                on_selection_change()
                        
                        with ui.card().classes(
                            'p-4 transition-all' +
                            (' cursor-pointer hover:shadow-lg' if is_unlocked else ' opacity-40') +
                            (' ring-2 ring-primary' if is_selected else '')
                        ).on('click', select_badge):
                            with ui.column().classes('items-center gap-3 w-full'):
                                with ui.element('div').classes('rounded-full flex items-center justify-center').style(
                                    f'width: 80px; height: 80px; background-color: {badge["color"] if is_unlocked else "#424242"};'
                                ):
                                    ui.icon(badge['icon']).classes('text-white').style('font-size: 40px;')
                                
                                with ui.column().classes('items-center gap-1 w-full'):
                                    ui.label(badge['name']).classes('font-bold text-center text-white')
                                    ui.label(badge['description']).classes('text-sm text-center').style('color: #9aa0a6')
                                    
                                    if not is_unlocked:
                                        with ui.row().classes('items-center gap-1 mt-2'):
                                            ui.icon('lock').classes('text-gray-500').style('font-size: 16px;')
                                            ui.label('Locked').classes('text-sm text-gray-500')
                                    elif is_selected:
                                        ui.badge('Equipped', color='primary').classes('mt-2')
            
            with ui.card().classes('w-full'):
                ui.label('Titles').classes('text-2xl font-bold mb-4 text-white')
                ui.label(f'Unlocked: {len(unlocked_titles)}/{len(TITLES)}').classes('text-lg mb-4').style('color: #9aa0a6')
                
                with ui.element('div').classes('grid gap-3 w-full').style(
                    'grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));'
                ):
                    for title_id, title in TITLES.items():
                        is_unlocked = any(t['id'] == title_id for t in unlocked_titles)
                        is_selected = title_id == selected_title_id
                        
                        def select_title(tid=title_id, unlocked=is_unlocked):
                            if unlocked:
                                set_selected_title(state, tid)
                                save_challenge_state(state)
                                on_selection_change()
                        
                        with ui.card().classes(
                            'p-4 transition-all' +
                            (' cursor-pointer hover:shadow-lg' if is_unlocked else ' opacity-40') +
                            (' ring-2 ring-primary' if is_selected else '')
                        ).on('click', select_title):
                            with ui.row().classes('items-center justify-between w-full'):
                                with ui.column().classes('gap-1'):
                                    with ui.row().classes('items-center gap-2'):
                                        ui.label(title['name']).classes('text-xl font-bold').style(
                                            f'color: {title["color"] if is_unlocked else "#666666"};'
                                        )
                                        if is_selected:
                                            ui.badge('Equipped', color='primary')
                                    ui.label(title['description']).classes('text-sm').style('color: #9aa0a6')
                                
                                if not is_unlocked:
                                    ui.icon('lock').classes('text-gray-500')