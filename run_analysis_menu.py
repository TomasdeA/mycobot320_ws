#!/usr/bin/env python3

import subprocess
import signal

def print_menu():
    print("\n=== Singularities Analysis Menu ===")
    print("1. Analyze Shoulder Singularity")
    print("2. Analyze Elbow Singularity")
    print("3. Analyze Wrist Singularity")
    print("4. Show All 8 Configurations")
    print("0. Exit")

def launch_with_mode(mode: str):
    cmd = [
        'ros2', 'launch', 'mycobot320_analysis', 'main_analysis.launch.py',
        f'mode:={mode}',
        f'with_rviz:=true'
    ]

    print(f"\nLaunching '{mode}' analysis... (press Ctrl+C to return to menu)\n")
    try:
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print("\nProcess interrupted by user. Returning to menu...")

def main():
    rviz_launched = False

    while True:
        print_menu()
        choice = input("Select an option: ").strip()

        if choice == '1':
            launch_with_mode('shoulder')
        elif choice == '2':
            launch_with_mode('elbow')
        elif choice == '3':
            launch_with_mode('wrist')
        elif choice == '4':
            launch_with_mode('show')
        elif choice == '0':
            print("Exiting.")
            break
        else:
            print("Invalid option.")
            continue

        rviz_launched = True

if __name__ == "__main__":
    main()
