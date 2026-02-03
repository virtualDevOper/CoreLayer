#!/usr/bin/env python3
"""
Запуск визуализатора данных полета
Кроссплатформенная замена для run_visualizer.bat и run_visualizer.sh
"""

import sys
import subprocess
import os
from pathlib import Path


def check_python():
    """Проверка версии Python"""
    if sys.version_info < (3, 7):
        print("Ошибка: Требуется Python 3.7 или выше")
        print(f"Текущая версия: {sys.version}")
        return False
    
    print(f"Используется Python: {sys.version}")
    return True


def check_and_install_dependencies():
    """Проверка и установка зависимостей"""
    print("Проверка зависимостей...")
    
    required_packages = ['pandas', 'matplotlib', 'numpy']
    missing_packages = []
    
    for package in required_packages:
        try:
            __import__(package)
        except ImportError:
            missing_packages.append(package)
    
    if missing_packages:
        print(f"Отсутствуют пакеты: {', '.join(missing_packages)}")
        print("Установка необходимых пакетов...")
        
        requirements_file = Path(__file__).parent / 'requirements.txt'
        
        if requirements_file.exists():
            try:
                subprocess.run([sys.executable, '-m', 'pip', 'install', '-r', str(requirements_file)], 
                             check=True)
                print("Зависимости успешно установлены")
            except subprocess.CalledProcessError:
                print("Ошибка: Не удалось установить зависимости")
                print("Попробуйте выполнить вручную:")
                print(f"pip install -r {requirements_file}")
                return False
        else:
            # Устанавливаем базовые пакеты
            try:
                subprocess.run([sys.executable, '-m', 'pip', 'install'] + missing_packages, 
                             check=True)
                print("Зависимости успешно установлены")
            except subprocess.CalledProcessError:
                print("Ошибка: Не удалось установить зависимости")
                return False
    else:
        print("Все зависимости установлены")
    
    return True


def run_visualizer():
    """Запуск приложения визуализации"""
    print("Запуск приложения...")
    
    visualizer_script = Path(__file__).parent / 'flight_data_visualizer.py'
    
    if not visualizer_script.exists():
        print(f"Ошибка: Файл {visualizer_script} не найден")
        return False
    
    try:
        # Меняем рабочую директорию на папку с визуализатором
        os.chdir(Path(__file__).parent)
        
        # Запускаем скрипт
        result = subprocess.run([sys.executable, 'flight_data_visualizer.py'])
        
        if result.returncode != 0:
            print(f"\nПриложение завершилось с ошибкой (код: {result.returncode})")
            return False
        
        return True
        
    except KeyboardInterrupt:
        print("\nПриложение прервано пользователем")
        return True
    except Exception as e:
        print(f"Ошибка при запуске приложения: {e}")
        return False


def main():
    """Основная функция"""
    print("Starting Flight Data Visualizer...")
    print()
    
    # Проверка Python
    if not check_python():
        input("Нажмите Enter для выхода...")
        sys.exit(1)
    
    # Проверка и установка зависимостей
    if not check_and_install_dependencies():
        input("Нажмите Enter для выхода...")
        sys.exit(1)
    
    # Запуск приложения
    success = run_visualizer()
    
    if not success:
        input("Нажмите Enter для выхода...")
        sys.exit(1)


if __name__ == '__main__':
    main()