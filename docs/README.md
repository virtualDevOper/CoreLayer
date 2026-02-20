# Документация CoreLayer

## Основные документы

- **[GEOMETRY.md](GEOMETRY.md)** - Подготовка геометрии для аэродинамики
  - Быстрый старт
  - Система координат
  - Требования к геометрии
  - Частые ошибки

## API документация

- **[Аэродинамика](../include/aerodynamicsFromSTL/README.md)** - API библиотеки aerodynamicsFromSTL
- **[Конфигурация](../config/README.md)** - Руководство по конфигурационным файлам

## Утилиты

- `prepare_stl.py` - Подготовка STL геометрии
- `generate_simple_rocket.py` - Генератор тестовой геометрии
- `collect_aerodynamics.py` - Сборка кода аэродинамики
- `collect_all.py` - Сборка всего проекта

## Быстрый старт

1. Подготовьте геометрию:
```bash
python prepare_stl.py your_rocket.stl --name my_rocket
```

2. Настройте конфигурацию в `config/geometry_config.json`

3. Соберите проект:
```bash
cmake --build build-kiro --config Debug
```

4. Запустите:
```bash
cd build-kiro/Debug
./CoreLayer.exe
```
