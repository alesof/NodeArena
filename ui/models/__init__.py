"""Models package for challenge data and state management"""
from .challenge import (
    load_challenges,
    get_challenge_by_id,
    get_challenges_by_category,
    get_categories,
    hash_flag
)
from .state import (
    load_challenge_state,
    save_challenge_state,
    is_challenge_solved,
    is_challenge_locked,
    mark_challenge_solved,
    get_module_stats
)
from .badges import (
    BADGES,
    TITLES,
    get_unlocked_badges,
    get_unlocked_titles,
    get_selected_badge,
    get_selected_title,
    set_selected_badge,
    set_selected_title,
    get_badge_by_id,
    get_title_by_id
)

__all__ = [
    'load_challenges',
    'get_challenge_by_id',
    'get_challenges_by_category',
    'get_categories',
    'hash_flag',
    'load_challenge_state',
    'save_challenge_state',
    'is_challenge_solved',
    'is_challenge_locked',
    'mark_challenge_solved',
    'get_module_stats',
    'BADGES',
    'TITLES',
    'get_unlocked_badges',
    'get_unlocked_titles',
    'get_selected_badge',
    'get_selected_title',
    'set_selected_badge',
    'set_selected_title',
    'get_badge_by_id',
    'get_title_by_id',
]