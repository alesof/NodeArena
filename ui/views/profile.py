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
            
            # Badges Section
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
                        
                        card_classes = 'p-4 transition-all'
                        if is_unlocked:
                            card_classes += ' cursor-pointer'
                        else:
                            card_classes += ' opacity-40'
                        if is_selected:
                            card_classes += ' ring-2 ring-primary'
                        
                        with ui.card().classes(card_classes).on('click', select_badge).style(
                            'border: 2px solid transparent; transition: all 0.3s ease;'
                        ) as card:
                            if is_unlocked:
                                card.on('mouseenter', lambda c=card: c.style(add='border-color: #d66c48;'))
                                card.on('mouseleave', lambda c=card: c.style(add='border-color: transparent;'))
                            
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
            
            # Titles Section
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
                        
                        card_classes = 'p-4 transition-all'
                        if is_unlocked:
                            card_classes += ' cursor-pointer'
                        else:
                            card_classes += ' opacity-40'
                        if is_selected:
                            card_classes += ' ring-2 ring-primary'
                        
                        with ui.card().classes(card_classes).on('click', select_title).style(
                            'border: 2px solid transparent; transition: all 0.3s ease;'
                        ) as card:
                            if is_unlocked:
                                card.on('mouseenter', lambda c=card: c.style(add='border-color: #d66c48;'))
                                card.on('mouseleave', lambda c=card: c.style(add='border-color: transparent;'))
                            
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