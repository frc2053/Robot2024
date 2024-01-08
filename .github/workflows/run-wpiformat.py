#!/usr/bin/env python3
import subprocess
import sys

def main():
    if sys.platform == 'win32':
        cmd = (r'./venv/Scripts/wpiformat.exe')
    else:
        cmd = (r'./venv/bin/wpiformat')
    return subprocess.call(cmd)

if __name__ == '__main__':
    exit(main())