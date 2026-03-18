#!/usr/bin/env python3
"""
Собирает все файлы проекта в один текстовый файл,
исключая библиотеки и служебные директории.
"""

import argparse
from pathlib import Path
from datetime import datetime


ALLOWED_EXTENSIONS = {
    ".cpp", ".h", ".hpp", ".tpp",
    ".cmake", ".txt", ".md", ".json"
}

EXCLUDED_DIRS = {
    "libs",
    ".kiro",
    "build",
    "cmake-build-debug",
}

TEXT_ENCODINGS = ("utf-8", "cp1251")


def is_hidden(path: Path) -> bool:
    return any(part.startswith(".") for part in path.parts)


def should_skip_dir(dir_name: str) -> bool:
    return dir_name in EXCLUDED_DIRS or dir_name.startswith(".")


def should_include_file(file_path: Path, output_file: Path) -> bool:
    if file_path == output_file:
        return False

    if is_hidden(file_path.relative_to(Path.cwd())):
        return False

    if file_path.suffix:
        return file_path.suffix.lower() in ALLOWED_EXTENSIONS

    return True


def guess_language(file_path: Path) -> str:
    suffix = file_path.suffix.lower()
    name = file_path.name.lower()

    mapping = {
        ".cpp": "cpp",
        ".h": "cpp",
        ".hpp": "cpp",
        ".tpp": "cpp",
        ".cmake": "cmake",
        ".txt": "text",
        ".md": "markdown",
        ".json": "json",
    }

    if suffix in mapping:
        return mapping[suffix]

    if name in {"cmakelists.txt"}:
        return "cmake"

    if name in {"readme", "license", "authors", "copying", "changelog"}:
        return "text"

    return "text"


def read_text_file(file_path: Path) -> str | None:
    for encoding in TEXT_ENCODINGS:
        try:
            return file_path.read_text(encoding=encoding)
        except UnicodeDecodeError:
            continue
        except Exception:
            return None
    return None


def collect_files(output_filename: str = "project_all_files.txt"):
    base_path = Path.cwd()
    output_file = base_path / output_filename

    if output_file.exists():
        output_file.unlink()

    collected_files = []
    skipped_files = []

    for path in base_path.rglob("*"):
        if not path.is_file():
            continue

        try:
            relative_path = path.relative_to(base_path)
        except ValueError:
            skipped_files.append((str(path), "не удалось вычислить относительный путь"))
            continue

        if any(should_skip_dir(part) for part in relative_path.parts[:-1]):
            skipped_files.append((str(relative_path), "файл в исключенной директории"))
            continue

        if relative_path.name.startswith("."):
            skipped_files.append((str(relative_path), "скрытый файл"))
            continue

        if not should_include_file(path, output_file):
            if path == output_file:
                skipped_files.append((str(relative_path), "выходной файл"))
            elif path.suffix and path.suffix.lower() not in ALLOWED_EXTENSIONS:
                skipped_files.append((str(relative_path), f"неподходящее расширение: {path.suffix}"))
            else:
                skipped_files.append((str(relative_path), "исключен правилом фильтрации"))
            continue

        collected_files.append(path)

    collected_files.sort(key=lambda p: str(p.relative_to(base_path)).lower())

    header = [
        "# Полная сборка файлов проекта",
        f"# Сгенерировано: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"# Корневая директория: {base_path}",
        f"# Выходной файл: {output_file.name}",
        "",
        "# Исключенные директории:",
        *[f"# - {name}" for name in sorted(EXCLUDED_DIRS)],
        "# - все скрытые файлы и папки",
        "",
        "# Разрешенные расширения:",
        *[f"# - {ext}" for ext in sorted(ALLOWED_EXTENSIONS)],
        "# - файлы без расширения",
        "",
        "=" * 100,
        "",
    ]

    output_file.write_text("\n".join(header), encoding="utf-8")

    written_count = 0
    unreadable_count = 0

    with output_file.open("a", encoding="utf-8") as out:
        for file_path in collected_files:
            relative_path = file_path.relative_to(base_path)
            content = read_text_file(file_path)

            if content is None:
                skipped_files.append((str(relative_path), "не удалось прочитать как текст"))
                unreadable_count += 1
                continue

            lang = guess_language(file_path)

            out.write(f"Файл: {relative_path}\n")
            out.write(f"Язык блока: {lang}\n")
            out.write("-" * 100 + "\n")
            out.write(f"```{lang}\n")
            out.write(content)
            if content and not content.endswith("\n"):
                out.write("\n")
            out.write("```\n")
            out.write("=" * 100 + "\n\n")

            written_count += 1

        total_size = output_file.stat().st_size
        total_size_kb = total_size / 1024
        total_size_mb = total_size / (1024 * 1024)

        out.write("\nСТАТИСТИКА\n")
        out.write("-" * 100 + "\n")
        out.write(f"Найдено подходящих файлов: {len(collected_files)}\n")
        out.write(f"Успешно записано: {written_count}\n")
        out.write(f"Пропущено при чтении: {unreadable_count}\n")
        out.write(f"Всего пропущено: {len(skipped_files)}\n")
        out.write(f"Размер итогового файла: {total_size} байт ({total_size_kb:.2f} KB / {total_size_mb:.2f} MB)\n")

        out.write("\nПРОПУЩЕННЫЕ ФАЙЛЫ\n")
        out.write("-" * 100 + "\n")
        if skipped_files:
            for rel_path, reason in sorted(skipped_files, key=lambda x: x[0].lower()):
                out.write(f"{rel_path} -> {reason}\n")
        else:
            out.write("Нет пропущенных файлов\n")

    print("=" * 60)
    print("Готово")
    print(f"Выходной файл: {output_file.name}")
    print(f"Подходящих файлов найдено: {len(collected_files)}")
    print(f"Успешно записано: {written_count}")
    print(f"Пропущено: {len(skipped_files)}")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="Сборка файлов проекта в один txt")
    parser.add_argument(
        "-o", "--output",
        default="project_all_files.txt",
        help="Имя выходного файла"
    )
    args = parser.parse_args()

    try:
        collect_files(args.output)
    except KeyboardInterrupt:
        print("\nПрервано")
    except Exception as e:
        print(f"\nОшибка: {e}")


if __name__ == "__main__":
    main()
