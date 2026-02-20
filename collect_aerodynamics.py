#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Скрипт для сборки всех файлов библиотеки аэродинамики в один текстовый файл
Собирает файлы из include/aero_simpi/
"""

import os
from pathlib import Path
from datetime import datetime


OUTPUT_FILE = "aero_simpi_library.txt"
AERO_DIR = "include/aero_simpi"


def add_file_to_output(file_path, output_handle, base_path):
    """Добавляет содержимое файла в выходной файл"""
    if not file_path.exists():
        return
    
    try:
        relative_path = file_path.relative_to(base_path)
        print(f"Добавляю: {relative_path}")
        
        # Читаем содержимое файла
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
        
        # Определяем язык для подсветки синтаксиса
        ext = file_path.suffix.lower()
        lang_map = {
            '.cpp': 'cpp', '.h': 'cpp', '.hpp': 'cpp', '.tpp': 'cpp',
            '.md': 'markdown', '.txt': 'text'
        }
        lang = lang_map.get(ext, 'text')
        
        # Записываем в выходной файл
        output_handle.write(f"\n### Файл: {relative_path}\n")
        output_handle.write(f"```{lang}\n")
        output_handle.write(content)
        output_handle.write("\n```\n\n")
        
    except Exception as e:
        print(f"Ошибка при обработке {file_path}: {e}")


def main():
    base_path = Path.cwd()
    aero_path = base_path / AERO_DIR
    output_path = base_path / OUTPUT_FILE
    
    if not aero_path.exists():
        print(f"ОШИБКА: Папка {AERO_DIR} не найдена!")
        return
    
    print("=" * 60)
    print("Сборка библиотеки аэродинамики (aero_simpi)...")
    print("=" * 60)
    
    # Удаляем старый файл
    if output_path.exists():
        output_path.unlink()
    
    with open(output_path, 'w', encoding='utf-8') as out:
        # Заголовок
        header = f"""# Библиотека аэродинамики - aero_simpi
# Полный исходный код модуля
# Сгенерировано: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
# ==========================================

"""
        out.write(header)
        
        # Собираем все файлы из папки аэродинамики
        print(f"\nСканирую папку: {AERO_DIR}")
        
        # Главный заголовочный файл
        main_header = aero_path / 'aerodynamics.h'
        if main_header.exists():
            out.write("\n## Главный заголовочный файл\n\n")
            add_file_to_output(main_header, out, base_path)
        
        # Документация из docs/
        docs_path = aero_path / 'docs'
        if docs_path.exists():
            out.write("\n## Документация\n\n")
            for file_path in sorted(docs_path.glob('*.md')):
                add_file_to_output(file_path, out, base_path)
        
        # Исходники из src/
        src_path = aero_path / 'src'
        if src_path.exists():
            out.write("\n## Исходные файлы\n\n")
            
            # Заголовочные файлы
            out.write("\n### Заголовочные файлы (.h)\n\n")
            for file_path in sorted(src_path.glob('*.h')):
                add_file_to_output(file_path, out, base_path)
            
            # Файлы реализации
            out.write("\n### Файлы реализации (.cpp)\n\n")
            for file_path in sorted(src_path.glob('*.cpp')):
                add_file_to_output(file_path, out, base_path)
    
    # Статистика
    file_size = output_path.stat().st_size
    file_size_kb = round(file_size / 1024, 2)
    file_size_mb = round(file_size / 1024 / 1024, 2)
    
    # Подсчет файлов
    total_files = 0
    
    # Главный заголовок
    if (aero_path / 'aerodynamics.h').exists():
        total_files += 1
    
    # Документация
    docs_path = aero_path / 'docs'
    if docs_path.exists():
        total_files += len(list(docs_path.glob('*.md')))
    
    # Исходники
    src_path = aero_path / 'src'
    if src_path.exists():
        total_files += len(list(src_path.glob('*.h')))
        total_files += len(list(src_path.glob('*.cpp')))
    
    print("\n" + "=" * 60)
    print(f"Готово! Библиотека собрана в: {OUTPUT_FILE}")
    print(f"Обработано файлов: {total_files}")
    print(f"Размер файла: {file_size} байт ({file_size_kb} KB / {file_size_mb} MB)")
    print("=" * 60)


if __name__ == '__main__':
    main()
