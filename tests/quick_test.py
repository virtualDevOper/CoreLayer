#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
⚡ Быстрый запуск тестов CoreLayer
Запускает все три типа тестов без лишнего вывода
"""

import subprocess
import sys
from pathlib import Path

def main():
    """Быстрый запуск всех тестов"""
    script_dir = Path(__file__).parent
    
    print("⚡ Быстрый запуск всех тестов...")
    
    try:
        # Запускаем основной скрипт в тихом режиме
        result = subprocess.run([
            sys.executable, "run_tests.py"
        ], cwd=script_dir)
        
        if result.returncode == 0:
            print("✅ ВСЕ ТЕСТЫ ПРОШЛИ!")
        else:
            print("❌ ЕСТЬ ОШИБКИ В ТЕСТАХ!")
            
        return result.returncode
        
    except Exception as e:
        print(f"❌ ОШИБКА: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())