#!/usr/bin/env python3
"""
Сборка всех файлов проекта CoreLayer в один файл
"""

import argparse
from pathlib import Path
from datetime import datetime


def collect_files(output_filename: str = "project_all_files.txt", include_libs: bool = False):
    """Собирает все файлы проекта"""
    base_path = Path.cwd()
    output_file = base_path / output_filename
    
    print("Сборка всех файлов проекта CoreLayer...")
    print("=" * 40)
    
    if output_file.exists():
        output_file.unlink()
    
    # Заголовок
    header = f"""# CoreLayer - Полный код проекта
# Сгенерировано: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
# ==========================================

"""
    output_file.write_text(header, encoding='utf-8')
    
    def add_file(file_path: Path):
        if not file_path.exists():
            return
        
        relative_path = file_path.relative_to(base_path)
        print(f"Добавляю: {relative_path}")
        
        try:
            content = file_path.read_text(encoding='utf-8')
        except UnicodeDecodeError:
            try:
                content = file_path.read_text(encoding='cp1251')
            except:
                print(f"  ⚠ Не удалось прочитать: {relative_path}")
                return
        
        with output_file.open('a', encoding='utf-8') as f:
            f.write(f"\n### Файл: {relative_path}\n")
            f.write("```cpp\n")
            f.write(content)
            f.write("\n```\n\n")
    
    # Основные файлы
    print("\nОсновные файлы...")
    for filename in ["main.cpp", "CMakeLists.txt", "README.md"]:
        add_file(base_path / filename)
    
    # Заголовки ядра
    print("\nЗаголовки ядра...")
    with output_file.open('a', encoding='utf-8') as f:
        f.write("\n## Основные заголовочные файлы\n\n")
    
    include_core = base_path / "include" / "core"
    if include_core.exists():
        for ext in ["*.h", "*.hpp", "*.tpp"]:
            for file_path in sorted(include_core.glob(ext)):
                add_file(file_path)
    
    # Исходный код
    print("\nИсходный код...")
    with output_file.open('a', encoding='utf-8') as f:
        f.write("\n## Исходный код\n\n")
    
    src_dir = base_path / "src"
    if src_dir.exists():
        for ext in ["*.h", "*.hpp", "*.cpp", "*.tpp"]:
            for file_path in sorted(src_dir.rglob(ext)):
                add_file(file_path)
    
    # Тесты
    print("\nТесты...")
    with output_file.open('a', encoding='utf-8') as f:
        f.write("\n## Тесты\n\n")
    
    tests_dir = base_path / "tests"
    if tests_dir.exists():
        for ext in ["*.h", "*.hpp", "*.cpp", "*.tpp"]:
            for file_path in sorted(tests_dir.rglob(ext)):
                add_file(file_path)
    
    # Конфигурация
    print("\nКонфигурация...")
    with output_file.open('a', encoding='utf-8') as f:
        f.write("\n## Конфигурация\n\n")
    
    config_dir = base_path / "config"
    if config_dir.exists():
        for ext in ["*.json", "*.xml", "*.cfg", "*.ini"]:
            for file_path in sorted(config_dir.glob(ext)):
                add_file(file_path)
    
    # Библиотеки (опционально)
    if include_libs:
        print("\nБиблиотеки...")
        with output_file.open('a', encoding='utf-8') as f:
            f.write("\n## Библиотеки\n\n")
        
        libs_dir = base_path / "libs"
        if libs_dir.exists():
            count = 0
            for ext in ["*.h", "*.hpp", "*.cpp", "*.tpp"]:
                for file_path in libs_dir.rglob(ext):
                    if count >= 20:
                        break
                    add_file(file_path)
                    count += 1
    
    # Статистика
    file_size = output_file.stat().st_size
    file_size_kb = file_size / 1024
    file_size_mb = file_size / (1024 * 1024)
    
    print("\n" + "=" * 40)
    print(f"Готово! Файл: {output_filename}")
    print(f"Размер: {file_size} байт ({file_size_kb:.2f} KB / {file_size_mb:.2f} MB)")
    print("=" * 40)


def main():
    parser = argparse.ArgumentParser(description="Сборка всех файлов проекта CoreLayer")
    parser.add_argument("-o", "--output", default="project_all_files.txt",
                        help="Имя выходного файла")
    parser.add_argument("--include-libs", action="store_true",
                        help="Включить файлы библиотек")
    
    args = parser.parse_args()
    
    try:
        collect_files(args.output, args.include_libs)
    except KeyboardInterrupt:
        print("\n\nПрервано")
    except Exception as e:
        print(f"\n❌ Ошибка: {e}")


if __name__ == "__main__":
    main()
