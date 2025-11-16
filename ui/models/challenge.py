"""
Challenge data models and operations
"""
import json
import hashlib
from pathlib import Path
from typing import List, Dict, Optional
import sys
sys.path.insert(0, str(Path(__file__).parent.parent))
from config import CHALLENGES_FILE


def hash_flag(flag: str) -> str:
    """Hash a flag using SHA256"""
    return hashlib.sha256(flag.encode()).hexdigest()


def load_challenges() -> List[Dict]:
    """Load challenges from JSON file"""
    if not CHALLENGES_FILE.exists():
        raise FileNotFoundError(f"Challenges file not found: {CHALLENGES_FILE}")
    with open(CHALLENGES_FILE, 'r') as f:
        return json.load(f)


def get_challenge_by_id(challenge_id: int, challenges: List[Dict]) -> Optional[Dict]:
    """Get a challenge by its ID"""
    for challenge in challenges:
        if challenge['id'] == challenge_id:
            return challenge
    return None


def get_challenges_by_category(category: str, challenges: List[Dict]) -> List[Dict]:
    """Get all challenges for a specific category"""
    return [ch for ch in challenges if ch['category'] == category]


def get_categories(challenges: List[Dict]) -> List[str]:
    """Get unique categories from challenges"""
    return list(dict.fromkeys(ch['category'] for ch in challenges))