import re

# Читаем файл
with open('tests/unit/OdeSolver/basic_rk4_test.cpp', 'r', encoding='utf-8') as f:
    content = f.read()

# Паттерн для замены: находим блок от auto initial_state до momento.addSnapshotByID
pattern = r'auto initial_state = KinematicState<metricType>::createBuilder\(\).*?\.build\(\);.*?SimulationMomento<metricType> momento;.*?momento\.addSnapshotByID\(1, std::move\(.*?\)\);'

replacement = '''SimulationMomento<metricType> momento;
        
        // Инициализируем momento через saveStartParams
        momento.saveStartParams(object_manager->getAllObjects());'''

# Заменяем с учетом многострочности
content_new = re.sub(pattern, replacement, content, flags=re.DOTALL)

# Записываем обратно
with open('tests/unit/OdeSolver/basic_rk4_test.cpp', 'w', encoding='utf-8') as f:
    f.write(content_new)

print("Замены выполнены!")
