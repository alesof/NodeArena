"""
Challenge view - shows individual challenge details and flag submission
"""
from nicegui import ui
from typing import Dict, List, Callable
from pathlib import Path
import sys
import asyncio
sys.path.insert(0, str(Path(__file__).parent.parent))
from config import DIFFICULTY_COLORS
from models.challenge import hash_flag
from models.state import get_challenge_flag_hash, mark_challenge_solved, is_challenge_solved
from components.terminal import create_terminal


def parse_challenge_readme(description: str) -> Dict[str, str]:
    """Parse the README into sections based on headers"""
    lines = description.split('\n')
    
    sections = {
        'title': '',
        'objective': '',
        'story': '',
        'mission': '',
        'commands': '',
        'learning': '',
        'hints': ''
    }
    
    current_section = None
    current_content = []
    
    for line in lines:
        # Skip the main title (first # header)
        if line.startswith('# ') and not sections['title']:
            sections['title'] = line.replace('# ', '').strip()
            continue
        
        # Detect section headers
        if '🎯 Objective' in line or line.strip().startswith('## 🎯'):
            if current_section and current_content:
                sections[current_section] = '\n'.join(current_content).strip()
            current_section = 'objective'
            current_content = []
        elif '📖 Story' in line or line.strip().startswith('## 📖'):
            if current_section and current_content:
                sections[current_section] = '\n'.join(current_content).strip()
            current_section = 'story'
            current_content = []
        elif '🧩' in line and 'Mission' in line:
            if current_section and current_content:
                sections[current_section] = '\n'.join(current_content).strip()
            current_section = 'mission'
            current_content = []
        elif '🛠️' in line and 'Commands' in line:
            if current_section and current_content:
                sections[current_section] = '\n'.join(current_content).strip()
            current_section = 'commands'
            current_content = []
        elif '📚' in line and 'Learning' in line:
            if current_section and current_content:
                sections[current_section] = '\n'.join(current_content).strip()
            current_section = 'learning'
            current_content = []
        elif '💡' in line and 'Hints' in line:
            if current_section and current_content:
                sections[current_section] = '\n'.join(current_content).strip()
            current_section = 'hints'
            current_content = []
        elif line.strip() == '---':
            # End of content marker - save current section
            if current_section and current_content:
                sections[current_section] = '\n'.join(current_content).strip()
            break
        else:
            if current_section:
                current_content.append(line)
    
    # Add the last section
    if current_section and current_content:
        sections[current_section] = '\n'.join(current_content).strip()
    
    return sections


def show_challenge(
    challenge: Dict,
    challenges: List[Dict],
    state: Dict,
    on_back_click: Callable,
    on_solve_callback: Callable
):
    is_solved = is_challenge_solved(challenge['id'], state)
    terminal = None
    
    # Parse the README into sections
    sections = parse_challenge_readme(challenge['full_description'])
    
    with ui.column().classes('w-full p-6 gap-6'):
        with ui.row().classes('items-center gap-4 mb-2'):
            ui.button(icon='arrow_back', on_click=on_back_click).props('flat color=primary')
            ui.label(challenge['title']).classes('text-4xl font-bold text-white')
            if is_solved:
                ui.icon('check_circle').classes('text-green-500').style('font-size: 32px;')
        
        with ui.row().classes('w-full gap-6 items-start'):
            with ui.column().classes('flex-1 gap-6'):
                # Objective and Story - Always visible
                with ui.card().classes('w-full p-6'):
                    if sections['objective']:
                        with ui.column().classes('gap-2 mb-4'):
                            with ui.row().classes('items-center gap-2'):
                                ui.icon('flag').classes('text-primary').style('font-size: 24px;')
                                ui.label('Objective').classes('text-xl font-bold text-white')
                            ui.markdown(sections['objective']).classes('text-base')
                    
                    if sections['story']:
                        with ui.column().classes('gap-2 mt-4'):
                            with ui.row().classes('items-center gap-2'):
                                ui.icon('auto_stories').classes('text-primary').style('font-size: 24px;')
                                ui.label('Story').classes('text-xl font-bold text-white')
                            ui.markdown(sections['story']).classes('text-base')
                    
                    if sections['mission']:
                        with ui.column().classes('gap-2 mt-4'):
                            with ui.row().classes('items-center gap-2'):
                                ui.icon('task_alt').classes('text-primary').style('font-size: 24px;')
                                ui.label('Your Mission').classes('text-xl font-bold text-white')
                            ui.markdown(sections['mission']).classes('text-base')
                
                # Commands section - Collapsible but open by default
                if sections['commands']:
                    with ui.expansion('🛠️ Commands & Tutorial', icon='terminal').classes('w-full') as commands_exp:
                        commands_exp.props('default-opened')
                        commands_exp.classes('bg-[#1a1d24]')
                        with ui.card().classes('w-full p-6 bg-transparent').style('box-shadow: none; border: none;'):
                            # Add custom CSS for better markdown rendering
                            ui.add_head_html('''
                            <style>
                                .command-section h3 {
                                    color: #fbbf24 !important;
                                    font-size: 1.1rem !important;
                                    font-weight: 600 !important;
                                    margin-top: 1.5rem !important;
                                    margin-bottom: 0.75rem !important;
                                }
                                .command-section p {
                                    color: #d1d5db !important;
                                    line-height: 1.7 !important;
                                    margin-bottom: 1rem !important;
                                }
                                .command-section strong {
                                    color: #fbbf24 !important;
                                    font-weight: 600 !important;
                                }
                                .command-section code {
                                    background: linear-gradient(135deg, #0a0e12 0%, #1a1f26 100%) !important;
                                    color: #00ff41 !important;
                                    padding: 0.2rem 0.5rem !important;
                                    border-radius: 0.25rem !important;
                                    font-family: 'Fira Code', 'Courier New', monospace !important;
                                    font-size: 0.9em !important;
                                    border: 1px solid #00ff4133 !important;
                                    box-shadow: 0 0 10px #00ff4111 !important;
                                }
                                .command-section pre {
                                    background: linear-gradient(135deg, #0a0e12 0%, #0f1419 100%) !important;
                                    border: 1px solid #00ff4144 !important;
                                    border-radius: 0.5rem !important;
                                    padding: 1.25rem !important;
                                    margin: 1rem 0 !important;
                                    overflow-x: auto !important;
                                    box-shadow: 0 4px 20px rgba(0, 255, 65, 0.1), inset 0 1px 0 rgba(0, 255, 65, 0.1) !important;
                                    position: relative !important;
                                }
                                .command-section pre::before {
                                    content: '>' !important;
                                    position: absolute !important;
                                    left: 0.75rem !important;
                                    color: #00ff41 !important;
                                    font-weight: bold !important;
                                    opacity: 0.5 !important;
                                }
                                .command-section pre code {
                                    background: transparent !important;
                                    padding: 0 !important;
                                    padding-left: 1rem !important;
                                    color: #00ff41 !important;
                                    font-size: 0.95rem !important;
                                    border: none !important;
                                    box-shadow: none !important;
                                    text-shadow: 0 0 5px rgba(0, 255, 65, 0.3) !important;
                                }
                                .command-section ul, .command-section ol {
                                    margin-left: 1.5rem !important;
                                    margin-bottom: 1rem !important;
                                }
                                .command-section li {
                                    color: #d1d5db !important;
                                    margin-bottom: 0.5rem !important;
                                    line-height: 1.6 !important;
                                }
                            </style>
                            ''')
                            ui.markdown(sections['commands']).classes('text-base command-section')
                
                # Learning section - Collapsible
                if sections['learning']:
                    with ui.expansion('📚 What You\'re Learning', icon='school').classes('w-full') as learn_exp:
                        learn_exp.classes('bg-[#1a1d24]')
                        with ui.card().classes('w-full p-6 bg-transparent').style('box-shadow: none; border: none;'):
                            ui.add_head_html('''
                            <style>
                                .learning-section h3 {
                                    color: #fbbf24 !important;
                                    font-size: 1.1rem !important;
                                    font-weight: 600 !important;
                                    margin-top: 1.5rem !important;
                                    margin-bottom: 0.75rem !important;
                                }
                                .learning-section p {
                                    color: #d1d5db !important;
                                    line-height: 1.8 !important;
                                    margin-bottom: 1rem !important;
                                }
                                .learning-section strong {
                                    color: #fbbf24 !important;
                                    font-weight: 600 !important;
                                }
                                .learning-section code {
                                    background-color: #0f1419 !important;
                                    color: #a78bfa !important;
                                    padding: 0.2rem 0.4rem !important;
                                    border-radius: 0.25rem !important;
                                    font-family: 'Fira Code', 'Courier New', monospace !important;
                                    font-size: 0.9em !important;
                                }
                                .learning-section ul, .learning-section ol {
                                    margin-left: 1.5rem !important;
                                    margin-bottom: 1rem !important;
                                }
                                .learning-section li {
                                    color: #d1d5db !important;
                                    margin-bottom: 0.5rem !important;
                                    line-height: 1.6 !important;
                                }
                            </style>
                            ''')
                            ui.markdown(sections['learning']).classes('text-base learning-section')
                
                # Terminal section
                with ui.card().classes('w-full p-0'):
                    with ui.row().classes('items-center justify-between p-6 pb-3'):
                        with ui.row().classes('items-center gap-2'):
                            ui.icon('terminal').classes('text-primary').style('font-size: 24px;')
                            ui.label('Terminal').classes('text-xl font-bold text-white')
                        
                        ui.button(icon='splitscreen', on_click=lambda: split_terminal()).props('flat round color=primary').classes('text-sm')
                    
                    terminal_container = ui.row().classes('w-full gap-2')
                    
                    with terminal_container:
                        terminal1 = create_terminal()
                        ui.timer(0.1, lambda t=terminal1: asyncio.create_task(t.start_shell('/bin/bash')), once=True)
                
                def split_terminal():
                    with terminal_container:
                        new_terminal = create_terminal()
                        asyncio.create_task(new_terminal.start_shell('/bin/bash'))
            
            # Right sidebar - Flag submission and info
            with ui.column().classes('w-96 gap-6'):
                with ui.card().classes('w-full p-6'):
                    with ui.row().classes('items-center gap-2 mb-4'):
                        ui.icon('flag').classes('text-primary').style('font-size: 28px;')
                        ui.label('Submit Flag').classes('text-2xl font-bold text-white')
                    
                    if is_solved:
                        with ui.column().classes('items-center gap-3 py-6 w-full'):
                            ui.icon('check_circle').classes('text-green-500').style('font-size: 64px;')
                            ui.label('Challenge Solved!').classes('text-xl font-bold text-green-500')
                            ui.label(f'+{challenge["points"]} XP').classes('text-lg').style('color: #d66c48')
                    else:
                        flag_input = ui.input(label='Flag', placeholder='Write the flag here, it\'s case sensitive!').classes('w-full').props('outlined')
                        result_label = ui.label('').classes('mt-2')
                        
                        async def submit_flag():
                            submitted = flag_input.value.strip()
                            submitted_hash = hash_flag(submitted)
                            correct_flag_hash = get_challenge_flag_hash(challenge['id'], state)
                            
                            if submitted_hash == correct_flag_hash:
                                result_label.text = 'Correct! Challenge solved!'
                                result_label.classes('text-green-500 font-bold text-lg')
                                mark_challenge_solved(challenge['id'], state, challenges)
                                
                                from models.state import get_solved_count, get_total_points
                                old_solved = get_solved_count(challenges, state) - 1
                                new_solved = get_solved_count(challenges, state)
                                old_level = (old_solved // 5) + 1
                                new_level = (new_solved // 5) + 1
                                old_progress = (old_solved % 5)
                                new_progress = (new_solved % 5)
                                old_percentage = (old_progress / 5) * 100
                                new_percentage = (new_progress / 5) * 100
                                
                                with ui.dialog() as dialog, ui.card().classes('items-center p-8'):
                                    ui.icon('celebration').classes('text-yellow-500').style('font-size: 80px;')
                                    ui.label('Challenge Complete!').classes('text-3xl font-bold text-white mt-4')
                                    ui.label(challenge['title']).classes('text-xl').style('color: #9aa0a6')
                                    
                                    with ui.column().classes('items-center gap-2 mt-6 mb-4'):
                                        ui.label('XP Earned').classes('text-sm font-semibold').style('color: #9aa0a6')
                                        ui.label(f'+{challenge["points"]}').classes('text-5xl font-bold').style('color: #d66c48')
                                    
                                    ui.separator().classes('w-full my-4')
                                    
                                    with ui.column().classes('items-center gap-3 w-full'):
                                        with ui.row().classes('items-center gap-2'):
                                            ui.label('Level').classes('text-sm font-semibold').style('color: #9aa0a6')
                                            ui.label(str(new_level)).classes('text-3xl font-bold text-white')
                                        
                                        with ui.column().classes('gap-1 w-full max-w-md'):
                                            with ui.row().classes('w-full items-center justify-between'):
                                                ui.label(f'{new_progress}/5 Challenges').classes('text-xs font-semibold').style('color: #14b8a6')
                                                if new_level > old_level:
                                                    ui.label('LEVEL UP!').classes('text-xs font-bold').style('color: #14b8a6')
                                                else:
                                                    ui.label(f'Level {new_level + 1}').classes('text-xs').style('color: #9aa0a6')
                                            
                                            with ui.element('div').classes('w-full h-3 rounded-full overflow-hidden').style('background-color: #2d3748;') as progress_container:
                                                progress_bar = ui.element('div').classes('h-full rounded-full transition-all').style(
                                                    f'width: {old_percentage}%; background: #14b8a6; transition: width 1.5s ease-out;'
                                                )
                                    
                                    ui.button('Continue', on_click=lambda: (dialog.close(), on_solve_callback())).props('color=primary size=lg').classes('mt-6')
                                
                                dialog.open()
                                
                                await asyncio.sleep(0.5)
                                progress_bar.style(f'width: {new_percentage}%; background: #14b8a6; transition: width 1.5s ease-out;')
                                
                            else:
                                result_label.text = 'Incorrect flag. Try again!'
                                result_label.classes('text-red-500 font-bold')
                        
                        ui.button('Submit Flag', on_click=submit_flag, icon='send').classes('w-full mt-2').props('color=primary size=lg')
                
                with ui.card().classes('w-full p-6'):
                    ui.label('Challenge Info').classes('text-xl font-bold text-white mb-4')
                    
                    with ui.column().classes('gap-3'):
                        with ui.row().classes('items-center gap-2'):
                            ui.icon('category').style('color: #9aa0a6')
                            ui.label('Category:').classes('font-semibold').style('color: #9aa0a6')
                            ui.label(challenge['category']).classes('text-white')
                        
                        with ui.row().classes('items-center gap-2'):
                            ui.icon('speed').style('color: #9aa0a6')
                            ui.label('Difficulty:').classes('font-semibold').style('color: #9aa0a6')
                            ui.badge(challenge['difficulty'], color=DIFFICULTY_COLORS[challenge['difficulty']])
                        
                        with ui.row().classes('items-center gap-2'):
                            ui.icon('stars').style('color: #9aa0a6')
                            ui.label('Points:').classes('font-semibold').style('color: #9aa0a6')
                            ui.label(f"{challenge['points']} XP").classes('font-bold').style('color: #d66c48')
                        
                        if challenge.get('author'):
                            with ui.row().classes('items-center gap-2'):
                                ui.icon('person').style('color: #9aa0a6')
                                ui.label('Author:').classes('font-semibold').style('color: #9aa0a6')
                                ui.label(challenge['author']).classes('text-white')
                
                # Hints card - Only shown when there are hints
                if sections['hints'] or challenge.get('hints'):
                    with ui.card().classes('w-full p-6'):
                        hint_icon = None
                        hints_content = None
                        
                        def toggle_hints():
                            if hints_content:
                                hints_content.set_visibility(not hints_content.visible)
                                if hints_content.visible:
                                    hint_icon.style('transform: rotate(180deg); transition: transform 0.3s;')
                                else:
                                    hint_icon.style('transform: rotate(0deg); transition: transform 0.3s;')
                        
                        with ui.row().classes('items-center gap-2 mb-4 cursor-pointer').on('click', toggle_hints):
                            ui.icon('lightbulb').classes('text-yellow-500').style('font-size: 24px;')
                            ui.label('Need a Hint?').classes('text-xl font-bold text-white')
                            hint_icon = ui.icon('expand_more').classes('text-white ml-auto').style('font-size: 24px;')
                        
                        ui.add_head_html('''
                        <style>
                            .hint-item {
                                background: linear-gradient(135deg, #fbbf2411 0%, #fbbf2405 100%) !important;
                                border-left: 3px solid #fbbf24 !important;
                                padding: 0.75rem 1rem !important;
                                border-radius: 0.375rem !important;
                                line-height: 1.6 !important;
                                font-size: 0.9rem !important;
                                margin-bottom: 0.5rem !important;
                            }
                            .hint-item p {
                                color: #fbbf24 !important;
                                margin: 0.25rem 0 !important;
                            }
                            .hint-item li {
                                color: #fbbf24 !important;
                            }
                            .hint-item code {
                                background-color: #0f1419 !important;
                                color: #fbbf24 !important;
                                padding: 0.2rem 0.4rem !important;
                                border-radius: 0.25rem !important;
                                font-family: 'Fira Code', 'Courier New', monospace !important;
                            }
                        </style>
                        ''')
                        
                        hints_content = ui.column().classes('gap-3')
                        hints_content.visible = False
                        
                        with hints_content:
                            if sections['hints']:
                                # Display the hints content as markdown in a styled container
                                with ui.element('div').classes('hint-item w-full'):
                                    ui.markdown(sections['hints']).style('color: #fbbf24;')
                            elif challenge.get('hints'):
                                for hint in challenge['hints']:
                                    with ui.element('div').classes('hint-item w-full'):
                                        ui.label(hint).style('color: #fbbf24;')