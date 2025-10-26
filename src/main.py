#!/usr/bin/env python3
import time
from datetime import datetime

def main():
    while True:
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[AUTONOMOUS BOAT] Sistem aktif - {now}")
        time.sleep(2)

if __name__ == "__main__":
    main()

