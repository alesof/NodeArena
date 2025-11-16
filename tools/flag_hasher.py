#!/usr/bin/env python3
"""
Simple tool to hash CTF flags for secure storage.
Usage: python hash_flag.py
"""

import hashlib
import sys

def hash_flag(flag):
    """Hash a flag using SHA256"""
    return hashlib.sha256(flag.encode()).hexdigest()

def create_flag(content):
    """Create a CTF flag from content"""
    return f"CTF{{{content}}}"

def main():
    print("=" * 60)
    print("CTF Flag Hasher")
    print("=" * 60)
    print()
    
    if len(sys.argv) > 1:
        # Flag content provided as command line argument
        content = sys.argv[1]
        flag = create_flag(content)
        print(f"Flag content: {content}")
        print(f"Full flag: {flag}")
        print(f"Hash: {hash_flag(flag)}")
        print()
    else:
        # Interactive mode
        print("Enter flag content (will be wrapped as CTF{content})")
        print("Press Ctrl+C or leave empty to exit")
        print()
        
        while True:
            try:
                content = input("Enter flag content: ").strip()
                
                if not content:
                    print("\nExiting...")
                    break
                
                flag = create_flag(content)
                flag_hash = hash_flag(flag)
                
                print(f"Full flag: {flag}")
                print(f"Hashed: {flag_hash}")
                print()
                
                # Offer to copy format for JSON
                choice = input("Show JSON format? (y/n): ").strip().lower()
                if choice == 'y':
                    print()
                    print('Add this to your state.json:')
                    print(f'"flag_hash": "{flag_hash}",')
                    print()
                
            except KeyboardInterrupt:
                print("\n\nExiting...")
                break
            except Exception as e:
                print(f"Error: {e}")
                break

if __name__ == '__main__':
    main()