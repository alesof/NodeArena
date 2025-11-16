"""
Badge and Title data models
"""
from typing import Dict, List
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent.parent))


# Badge definitions - unlock conditions based on achievements
BADGES = {
    'starter': {
        'id': 'starter',
        'name': 'Getting Started',
        'description': 'Solve your first challenge',
        'icon': 'emoji_events',
        'color': '#4CAF50',
        'unlock_condition': lambda state, challenges: get_solved_count_from_state(state, challenges) >= 1
    },
    'explorer': {
        'id': 'explorer',
        'name': 'Explorer',
        'description': 'Solve 5 challenges',
        'icon': 'explore',
        'color': '#2196F3',
        'unlock_condition': lambda state, challenges: get_solved_count_from_state(state, challenges) >= 5
    },
    'expert': {
        'id': 'expert',
        'name': 'Expert',
        'description': 'Solve 10 challenges',
        'icon': 'workspace_premium',
        'color': '#9C27B0',
        'unlock_condition': lambda state, challenges: get_solved_count_from_state(state, challenges) >= 10
    },
    'master': {
        'id': 'master',
        'name': 'Master',
        'description': 'Solve all challenges',
        'icon': 'military_tech',
        'color': '#FF9800',
        'unlock_condition': lambda state, challenges: get_solved_count_from_state(state, challenges) == len(challenges)
    },
    'points_100': {
        'id': 'points_100',
        'name': 'Century',
        'description': 'Earn 100 points',
        'icon': 'star',
        'color': '#FFC107',
        'unlock_condition': lambda state, challenges: get_total_points_from_state(state, challenges) >= 100
    },
    'points_500': {
        'id': 'points_500',
        'name': 'Elite',
        'description': 'Earn 500 points',
        'icon': 'stars',
        'color': '#FF5722',
        'unlock_condition': lambda state, challenges: get_total_points_from_state(state, challenges) >= 500
    }
}


# Title definitions
TITLES = {
    'beginner': {
        'id': 'beginner',
        'name': 'Beginner',
        'description': 'Start your journey',
        'color': '#9E9E9E',
        'unlock_condition': lambda state, challenges: True  # Always unlocked
    },
    'apprentice': {
        'id': 'apprentice',
        'name': 'Apprentice',
        'description': 'Solve 3 challenges',
        'color': '#4CAF50',
        'unlock_condition': lambda state, challenges: get_solved_count_from_state(state, challenges) >= 3
    },
    'challenger': {
        'id': 'challenger',
        'name': 'Challenger',
        'description': 'Solve 7 challenges',
        'color': '#2196F3',
        'unlock_condition': lambda state, challenges: get_solved_count_from_state(state, challenges) >= 7
    },
    'veteran': {
        'id': 'veteran',
        'name': 'Veteran',
        'description': 'Solve 12 challenges',
        'color': '#9C27B0',
        'unlock_condition': lambda state, challenges: get_solved_count_from_state(state, challenges) >= 12
    },
    'legend': {
        'id': 'legend',
        'name': 'Legend',
        'description': 'Complete all challenges',
        'color': '#FF9800',
        'unlock_condition': lambda state, challenges: get_solved_count_from_state(state, challenges) == len(challenges)
    }
}


def get_solved_count_from_state(state: Dict, challenges: List[Dict]) -> int:
    """Get number of solved challenges"""
    return sum(1 for ch in challenges if state['challenges'].get(str(ch['id']), {}).get('solved', False))


def get_total_points_from_state(state: Dict, challenges: List[Dict]) -> int:
    """Get total points earned"""
    return sum(ch['points'] for ch in challenges if state['challenges'].get(str(ch['id']), {}).get('solved', False))


def get_unlocked_badges(state: Dict, challenges: List[Dict]) -> List[Dict]:
    """Get all unlocked badges"""
    unlocked = []
    for badge_id, badge in BADGES.items():
        if badge['unlock_condition'](state, challenges):
            unlocked.append(badge)
    return unlocked


def get_unlocked_titles(state: Dict, challenges: List[Dict]) -> List[Dict]:
    """Get all unlocked titles"""
    unlocked = []
    for title_id, title in TITLES.items():
        if title['unlock_condition'](state, challenges):
            unlocked.append(title)
    return unlocked


def get_selected_badge(state: Dict) -> str:
    """Get the currently selected badge ID"""
    return state.get('profile', {}).get('selected_badge', None)


def get_selected_title(state: Dict) -> str:
    """Get the currently selected title ID"""
    return state.get('profile', {}).get('selected_title', 'beginner')


def set_selected_badge(state: Dict, badge_id: str) -> None:
    """Set the selected badge"""
    if 'profile' not in state:
        state['profile'] = {}
    state['profile']['selected_badge'] = badge_id


def set_selected_title(state: Dict, title_id: str) -> None:
    """Set the selected title"""
    if 'profile' not in state:
        state['profile'] = {}
    state['profile']['selected_title'] = title_id


def get_badge_by_id(badge_id: str) -> Dict:
    """Get badge by ID"""
    return BADGES.get(badge_id)


def get_title_by_id(title_id: str) -> Dict:
    """Get title by ID"""
    return TITLES.get(title_id)