#!/usr/bin/env python3
"""
Script to automatically generate/update challenges.json from challenge directories.
Place this script in the challenges/ directory and run it.
"""

import json
from pathlib import Path
import re
import hashlib

def hash_flag(flag):
    """Hash a flag using SHA256"""
    return hashlib.sha256(flag.encode()).hexdigest()

def extract_challenge_id(challenge_name):
    """Extract challenge ID from challenge name (e.g., 'challenge_c00' -> 'c00')"""
    match = re.search(r'challenge_(.+)$', challenge_name)
    if match:
        return match.group(1)
    return challenge_name

def read_flag_file(challenge_dir):
    """Read flag from the 'flag' file in the challenge directory"""
    flag_file = challenge_dir / 'flag'
    
    if not flag_file.exists():
        print(f"  WARNING: No 'flag' file found in {challenge_dir.name}")
        return None
    
    try:
        with open(flag_file, 'r', encoding='utf-8') as f:
            flag = f.read().strip()
            if flag:
                return flag
            else:
                print(f"  WARNING: Empty flag file in {challenge_dir.name}")
                return None
    except Exception as e:
        print(f"  ERROR: Could not read flag file in {challenge_dir.name}: {e}")
        return None

def parse_readme(readme_path):
    """Parse README.md to extract challenge information"""
    
    with open(readme_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    info = {
        'title': 'Untitled Challenge',
        'description': '',
        'full_description': content,
        'difficulty': 'Medium',
        'points': 100
    }
    
    title_pattern = r'^#{1,6}\s+(.+)$'
    title_match = re.search(title_pattern, content, re.MULTILINE)
    if title_match:
        info['title'] = title_match.group(1).strip()
    
    desc_pattern = r'^#{1,6}.+?\n\n(.+?)(?:\n\n|$)'
    desc_match = re.search(desc_pattern, content, re.DOTALL)
    if desc_match:
        description = desc_match.group(1).strip()
        description = re.sub(r'\*\*(.+?)\*\*', r'\1', description)
        description = re.sub(r'\*(.+?)\*', r'\1', description)
        description = re.sub(r'`(.+?)`', r'\1', description)
        description = description.replace('\n', ' ')
        info['description'] = description
    
    difficulty_pattern = r'\*\*Difficulty\*\*:\s*(\w+)'
    difficulty_match = re.search(difficulty_pattern, content, re.IGNORECASE)
    if difficulty_match:
        info['difficulty'] = difficulty_match.group(1).capitalize()
    else:
        # Try to infer from keywords
        content_lower = content.lower()
        if 'easy' in content_lower or 'beginner' in content_lower:
            info['difficulty'] = 'Easy'
        elif 'hard' in content_lower or 'advanced' in content_lower:
            info['difficulty'] = 'Hard'
    
    # Extract points
    points_pattern = r'\*\*Points\*\*:\s*(\d+)'
    points_match = re.search(points_pattern, content, re.IGNORECASE)
    if points_match:
        info['points'] = int(points_match.group(1))
    
    return info

def get_category_from_path(challenge_path, base_path):
    """Extract category from the directory structure"""
    relative = challenge_path.relative_to(base_path)
    category = relative.parts[0] if len(relative.parts) > 1 else 'Uncategorized'
    # Capitalize each word
    return category.replace('_', ' ').title()

def scan_challenges(base_path):
    """Scan all challenge directories and build challenge list"""
    challenges = []
    
    # Find all challenge directories (starting with 'challenge_')
    for challenge_dir in sorted(base_path.rglob('challenge_*')):
        if not challenge_dir.is_dir():
            continue
        
        print(f"\nProcessing: {challenge_dir.name}")
        
        # Extract challenge ID
        challenge_id = extract_challenge_id(challenge_dir.name)
        
        # Get category from parent directory
        category = get_category_from_path(challenge_dir, base_path)
        
        # Parse README
        readme_path = challenge_dir / 'README.md'
        if not readme_path.exists():
            print(f"  WARNING: No README.md found, skipping...")
            continue
        
        info = parse_readme(readme_path)
        
        # Read flag from flag file
        flag = read_flag_file(challenge_dir)
        if not flag:
            print(f"  WARNING: Challenge {challenge_id} has no valid flag!")
        
        # Determine if challenge is locked (first challenge unlocked, rest locked)
        # You can customize this logic
        locked = len(challenges) > 0
        
        challenge = {
            'id': challenge_id,
            'title': info['title'],
            'description': info['description'],
            'full_description': info['full_description'],
            'difficulty': info['difficulty'],
            'category': category,
            'points': info['points'],
            'locked': locked,
            'flag': flag
        }
        
        challenges.append(challenge)
        print(f"  ✓ Added: {challenge_id} - {info['title']} ({category})")
    
    return challenges

def main():
    # Get the base path (challenges directory)
    script_dir = Path(__file__).parent
    base_path = (script_dir.parent / 'challenges').resolve()
    if not base_path.exists():
        raise FileNotFoundError(f"Challenges directory not found: {base_path}")
    
    print("=" * 60)
    print("CTF Challenge Generator")
    print("=" * 60)
    print(f"\nScanning challenges in: {base_path}")
    print("-" * 60)
    
    # Scan for challenges
    challenges = scan_challenges(base_path)
    
    if not challenges:
        print("\n⚠ WARNING: No challenges found!")
        return
    
    print("\n" + "=" * 60)
    print(f"Found {len(challenges)} challenges total")
    print("=" * 60)
    
    # Write to challenges.json (public info only)
    output_path = base_path / 'challenges.json'
    public_challenges = []
    for challenge in challenges:
        # Remove flag and locked status from public challenges.json
        public_challenge = {k: v for k, v in challenge.items() if k not in ['flag', 'locked']}
        public_challenges.append(public_challenge)
    
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(public_challenges, f, indent=2, ensure_ascii=False)
    
    print(f"\n✓ Successfully wrote public challenges to: {output_path}")
    
    # Write to state.json (private - hashed flags and solved status)
    state_path = base_path / 'state.json'
    state_data = {'challenges': {}}
    
    for challenge in challenges:
        flag = challenge['flag']
        if flag:
            flag_hash = hash_flag(flag)
            state_data['challenges'][str(challenge['id'])] = {
                'flag_hash': flag_hash,
                'solved': False  # Initially all unsolved
            }
            print(f"  Hashed flag for {challenge['id']}: {flag_hash[:16]}...")
        else:
            print(f"  ⚠ Skipped {challenge['id']} (no flag)")
    
    with open(state_path, 'w', encoding='utf-8') as f:
        json.dump(state_data, f, indent=2, ensure_ascii=False)
    
    print(f"\n✓ Successfully wrote challenge state (hashed flags) to: {state_path}")
    
    print("\n" + "=" * 60)
    print("Challenge Summary:")
    print("=" * 60)
    for challenge in challenges:
        lock_status = "🔒 LOCKED" if challenge['locked'] else "🔓 UNLOCKED"
        flag_status = "✓" if challenge['flag'] else "✗"
        print(f"  [{lock_status}] [{flag_status}] {challenge['id']}: {challenge['title']}")
        print(f"      Category: {challenge['category']} | Difficulty: {challenge['difficulty']} | Points: {challenge['points']}")

if __name__ == '__main__':
    main()