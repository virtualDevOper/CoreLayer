#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
🧪 Система запуска тестов CoreLayer
Упрощенная система с тремя основными тестами
"""

import os
import sys
import subprocess
import argparse
from pathlib import Path
import time
from typing import Optional

class Colors:
    """ANSI цвета для красивого вывода"""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'

class TestRunner:
    def __init__(self):
        self.script_dir = Path(__file__).parent
        self.project_root = self.script_dir.parent
        self.build_dir = self.project_root / "build"
        
        # Определяем исполняемые файлы для каждого типа тестов
        self.unit_tests_exe = self.build_dir / "tests" / "Debug" / "unit_tests.exe"
        self.integration_tests_exe = self.build_dir / "tests" / "Debug" / "integration_tests.exe"
        self.mock_tests_exe = self.build_dir / "tests" / "Debug" / "mock_tests.exe"
        
    def print_header(self):
        """Печатает красивый заголовок"""
        print(f"{Colors.HEADER}╔══════════════════════════════════════════════════════════════╗{Colors.ENDC}")
        print(f"{Colors.HEADER}║                🧪 ТЕСТОВАЯ СИСТЕМА CORELAYER                 ║{Colors.ENDC}")
        print(f"{Colors.HEADER}║                   Три основных теста                        ║{Colors.ENDC}")
        print(f"{Colors.HEADER}╚══════════════════════════════════════════════════════════════╝{Colors.ENDC}")
        print()
        
    def check_build_dir(self) -> bool:
        """Проверяет существование папки build"""
        if not self.build_dir.exists():
            print(f"{Colors.FAIL}❌ ОШИБКА: Папка build не найдена!{Colors.ENDC}")
            print(f"{Colors.WARNING}   Сначала выполните: mkdir build && cd build && cmake .. -DBUILD_TESTS=ON{Colors.ENDC}")
            return False
        return True
        
    def check_cmake_config(self) -> bool:
        """Проверяет конфигурацию CMake"""
        cmake_cache = self.build_dir / "CMakeCache.txt"
        if not cmake_cache.exists():
            print(f"{Colors.FAIL}❌ ОШИБКА: Проект не сконфигурирован!{Colors.ENDC}")
            print(f"{Colors.WARNING}   Выполните: cd build && cmake .. -DBUILD_TESTS=ON{Colors.ENDC}")
            return False
        return True
        
    def build_tests(self, test_type: str = "all", force_rebuild: bool = False) -> bool:
        """Собирает тесты"""
        print(f"{Colors.OKBLUE}🔨 Сборка тестов ({test_type})...{Colors.ENDC}")
        print("─" * 64)
        
        # Определяем цели для сборки
        targets = []
        if test_type == "all":
            targets = ["unit_tests", "integration_tests", "mock_tests"]
        elif test_type == "unit":
            targets = ["unit_tests"]
        elif test_type == "integration":
            targets = ["integration_tests"]
        elif test_type == "mock":
            targets = ["mock_tests"]
        else:
            print(f"{Colors.FAIL}❌ Неизвестный тип тестов: {test_type}{Colors.ENDC}")
            return False
            
        for target in targets:
            cmd = ["cmake", "--build", ".", "--target", target]
            if force_rebuild:
                cmd.append("--clean-first")
                
            try:
                result = subprocess.run(
                    cmd,
                    cwd=self.build_dir,
                    capture_output=True,
                    text=True,
                    encoding='utf-8'
                )
                
                if result.returncode != 0:
                    print(f"{Colors.FAIL}❌ ОШИБКА СБОРКИ: Не удалось собрать {target}{Colors.ENDC}")
                    if result.stderr:
                        print(f"\n{Colors.FAIL}Ошибки:{Colors.ENDC}")
                        print(result.stderr)
                    return False
                    
            except FileNotFoundError:
                print(f"{Colors.FAIL}❌ ОШИБКА: cmake не найден в PATH{Colors.ENDC}")
                return False
            except Exception as e:
                print(f"{Colors.FAIL}❌ ОШИБКА: {e}{Colors.ENDC}")
                return False
                
        print(f"{Colors.OKGREEN}✅ Сборка завершена успешно!{Colors.ENDC}")
        return True
            
    def run_test_suite(self, test_type: str, detailed: bool = False) -> bool:
        """Запускает конкретный набор тестов"""
        # Определяем исполняемый файл
        if test_type == "unit":
            test_exe = self.unit_tests_exe
            test_name = "Юнит тесты"
        elif test_type == "integration":
            test_exe = self.integration_tests_exe
            test_name = "Интеграционные тесты"
        elif test_type == "mock":
            test_exe = self.mock_tests_exe
            test_name = "Мок тестирование"
        else:
            print(f"{Colors.FAIL}❌ Неизвестный тип тестов: {test_type}{Colors.ENDC}")
            return False
            
        if not test_exe.exists():
            print(f"{Colors.WARNING}⚠️  {test_name}: исполняемый файл не найден, пропускаем{Colors.ENDC}")
            return True
            
        print(f"{Colors.OKGREEN}🚀 Запуск: {test_name}{Colors.ENDC}")
        print("═" * 64)
        
        # Формируем команду
        cmd = [str(test_exe)]
        if detailed:
            cmd.append("--success")
        else:
            cmd.extend(["--reporter", "compact"])
            
        try:
            start_time = time.time()
            result = subprocess.run(cmd, cwd=self.build_dir)
            end_time = time.time()
            
            print()
            if result.returncode == 0:
                print(f"{Colors.OKGREEN}✅ {test_name} прошли успешно! (время: {end_time - start_time:.2f}с){Colors.ENDC}")
            else:
                print(f"{Colors.FAIL}❌ {test_name} содержат ошибки!{Colors.ENDC}")
                
            return result.returncode == 0
            
        except Exception as e:
            print(f"{Colors.FAIL}❌ ОШИБКА при запуске {test_name}: {e}{Colors.ENDC}")
            return False
            
    def run_all_tests(self, detailed: bool = False) -> bool:
        """Запускает все три набора тестов"""
        print(f"{Colors.OKGREEN}🚀 Запуск всех тестов{Colors.ENDC}")
        print("═" * 64)
        print()
        
        results = {}
        test_types = [
            ("unit", "Юнит тесты"),
            ("integration", "Интеграционные тесты"), 
            ("mock", "Мок тестирование")
        ]
        
        for test_type, test_name in test_types:
            print(f"{Colors.OKCYAN}📋 {test_name}:{Colors.ENDC}")
            results[test_type] = self.run_test_suite(test_type, detailed)
            print()
            
        # Итоговый отчет
        print("═" * 64)
        print(f"{Colors.BOLD}📊 ИТОГОВЫЙ ОТЧЕТ:{Colors.ENDC}")
        print()
        
        all_passed = True
        for test_type, test_name in test_types:
            status = "✅ ПРОШЛИ" if results[test_type] else "❌ ОШИБКИ"
            color = Colors.OKGREEN if results[test_type] else Colors.FAIL
            print(f"   {color}{status}{Colors.ENDC} - {test_name}")
            if not results[test_type]:
                all_passed = False
                
        print()
        if all_passed:
            print(f"{Colors.OKGREEN}🎉 ВСЕ ТЕСТЫ ПРОШЛИ УСПЕШНО!{Colors.ENDC}")
        else:
            print(f"{Colors.FAIL}💥 ЕСТЬ ОШИБКИ В ТЕСТАХ!{Colors.ENDC}")
            
        return all_passed

def main():
    parser = argparse.ArgumentParser(
        description="🧪 Система запуска тестов CoreLayer - три основных теста",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument("--type", "-t", choices=["unit", "integration", "mock", "all"], 
                       default="all", help="Тип тестов для запуска")
    parser.add_argument("--detailed", "-d", action="store_true",
                       help="Подробный вывод всех утверждений")
    parser.add_argument("--rebuild", "-r", action="store_true",
                       help="Принудительная пересборка")
    parser.add_argument("--no-build", action="store_true",
                       help="Не собирать, только запустить")
    
    args = parser.parse_args()
    
    runner = TestRunner()
    runner.print_header()
    
    # Проверки
    if not runner.check_build_dir():
        return 1
        
    if not runner.check_cmake_config():
        return 1
    
    # Сборка
    if not args.no_build:
        if not runner.build_tests(test_type=args.type, force_rebuild=args.rebuild):
            return 1
        print()
    
    # Запуск тестов
    if args.type == "all":
        success = runner.run_all_tests(detailed=args.detailed)
    else:
        success = runner.run_test_suite(args.type, detailed=args.detailed)
    
    print()
    print("─" * 64)
    print(f"{Colors.OKCYAN}💡 Доступные команды:{Colors.ENDC}")
    print(f"   • Все тесты:         python run_tests.py")
    print(f"   • Только юнит:       python run_tests.py --type unit")
    print(f"   • Только интеграция: python run_tests.py --type integration")
    print(f"   • Только моки:       python run_tests.py --type mock")
    print(f"   • Подробный вывод:   python run_tests.py --detailed")
    print(f"   • Быстрый запуск:    python quick_test.py")
    print("─" * 64)
    
    return 0 if success else 1

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print(f"\n{Colors.WARNING}⚠️  Прервано пользователем{Colors.ENDC}")
        sys.exit(1)
    except Exception as e:
        print(f"\n{Colors.FAIL}❌ Неожиданная ошибка: {e}{Colors.ENDC}")
        sys.exit(1)