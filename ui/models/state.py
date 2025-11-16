"""
State management for challenge progress
"""
import json
from typing import Dict, List
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent.parent))
from config import STATE_FILE


def load_challenge_state() -> Dict:
    """Load challenge state from JSON file"""
    if not STATE_FILE.exists():
        raise FileNotFoundError(f"State file not found: {STATE_FILE}")
    with open(STATE_FILE, 'r') as f:
        return json.load(f)


def save_challenge_state(state: Dict) -> None:
    """Save challenge state to JSON file"""
    with open(STATE_FILE, 'w') as f:
        json.dump(state, f, indent=2)


def is_challenge_solved(challenge_id: int, state: Dict) -> bool:
    """Check if a challenge is solved"""
    return state['challenges'].get(str(challenge_id), {}).get('solved', False)


def is_challenge_locked(challenge_id: int, challenges: List[Dict], state: Dict) -> bool:
    """Check if a challenge is locked (previous challenge not solved)"""
    # First challenge is never locked
    if challenge_id == challenges[0]['id']:
        return False
    
    # Check if previous challenge is solved
    for i, challenge in enumerate(challenges):
        if challenge['id'] == challenge_id:
            if i > 0:
                prev_challenge_id = challenges[i - 1]['id']
                return not is_challenge_solved(prev_challenge_id, state)
    return True


def get_challenge_flag_hash(challenge_id: int, state: Dict) -> str:
    """Get the flag hash for a challenge"""
    challenge_state = state['challenges'].get(str(challenge_id), {})
    return challenge_state.get('flag_hash', '')


def mark_challenge_solved(challenge_id: int, state: Dict, challenges: List[Dict]) -> None:
    """Mark a challenge as solved and unlock the next one"""
    challenge_key = str(challenge_id)
    if challenge_key not in state['challenges']:
        state['challenges'][challenge_key] = {}
    state['challenges'][challenge_key]['solved'] = True
    
    # Prepare next challenge
    next_challenge_id = get_next_challenge_id(challenge_id, challenges)
    if next_challenge_id:
        next_key = str(next_challenge_id)
        if next_key not in state['challenges']:
            state['challenges'][next_key] = {}
    
    save_challenge_state(state)


def get_next_challenge_id(challenge_id: int, challenges: List[Dict]) -> int:
    """Get the ID of the next challenge in sequence"""
    for i, challenge in enumerate(challenges):
        if challenge['id'] == challenge_id:
            if i + 1 < len(challenges):
                return challenges[i + 1]['id']
    return None


def get_solved_count(challenges: List[Dict], state: Dict) -> int:
    """Get the number of solved challenges"""
    return sum(1 for ch in challenges if is_challenge_solved(ch['id'], state))


def get_total_points(challenges: List[Dict], state: Dict) -> int:
    """Get the total points earned"""
    return sum(ch['points'] for ch in challenges if is_challenge_solved(ch['id'], state))


def get_module_stats(category: str, challenges: List[Dict], state: Dict) -> Dict:
    """Get statistics for a module/category"""
    module_challenges = [ch for ch in challenges if ch['category'] == category]
    total = len(module_challenges)
    solved = sum(1 for ch in module_challenges if is_challenge_solved(ch['id'], state))
    total_points = sum(ch['points'] for ch in module_challenges)
    earned_points = sum(ch['points'] for ch in module_challenges if is_challenge_solved(ch['id'], state))
    
    return {
        'total': total,
        'solved': solved,
        'total_points': total_points,
        'earned_points': earned_points,
        'progress': (solved / total * 100) if total > 0 else 0
    }