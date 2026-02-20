# FlightMath - Руководство по использованию

## Обзор

FlightMath - это стандартизированный математический модуль для работы с координатными преобразованиями в аэрокосмических приложениях. Модуль основан на библиотеке Eigen и предоставляет единый интерфейс для работы с углами Эйлера, матрицами поворота, кватернионами и кинематическими преобразованиями.

## Подключение

```cpp
#include "math/FlightMath.h"
using namespace core::math;
```

## Основные типы данных

```cpp
// Базовые типы
Vector3 = Eigen::Vector3d;     // 3D вектор
Matrix3 = Eigen::Matrix3d;     // 3x3 матрица
Quaternion = Eigen::Quaterniond; // Кватернион
```

## Углы Эйлера

### Создание углов Эйлера

```cpp
// ZYX конвенция (аэрокосмическая): yaw-pitch-roll
auto euler = EulerAngles::ZYX(yaw, pitch, roll);

// ZXY конвенция (БПЛА): yaw-roll-pitch  
auto euler = EulerAngles::ZXY(yaw, roll, pitch);

// YXZ конвенция (альтернативная): pitch-roll-yaw
auto euler = EulerAngles::YXZ(pitch, roll, yaw);
```

### Пример для ракеты

```cpp
// Углы для ракеты: 15° тангаж, 0° рыскание, 0° крен
double pitch = 0.2617994; // 15 градусов в радианах
double yaw = 0.0;
double roll = 0.0;

auto euler = EulerAngles::ZYX(yaw, pitch, roll);
```

## Преобразования координат

### Углы Эйлера → Матрица поворота (DCM)

```cpp
auto euler = EulerAngles::ZYX(yaw, pitch, roll);
Matrix3 dcm = eulerToDCM(euler);
```

### Матрица поворота → Углы Эйлера

```cpp
Matrix3 dcm = /* ваша матрица */;
EulerAngles euler = dcmToEuler(dcm, EulerConvention::ZYX);
```

### Углы Эйлера → Кватернион

```cpp
auto euler = EulerAngles::ZYX(yaw, pitch, roll);
Quaternion quat = eulerToQuaternion(euler);
```

### Кватернион ↔ Матрица поворота

```cpp
// Кватернион → DCM
Matrix3 dcm = quatToDCM(quat);

// DCM → Кватернион
Quaternion quat = dcmToQuaternion(dcm);
```

## Поворот векторов

### Основная функция

```cpp
Vector3 rotateVector(const Vector3& vector, const Matrix3& dcm, bool body_to_ned = true);
Vector3 rotateVector(const Vector3& vector, const Quaternion& quat, bool body_to_ned = true);
```

### Примеры использования

```cpp
// Поворот вектора из связанной системы координат в земную (NED)
Vector3 velocity_body(50.0, 0.0, 0.0); // 50 м/с по оси X
auto euler = EulerAngles::ZYX(0.0, 0.2617994, 0.0); // 15° тангаж
Matrix3 dcm = eulerToDCM(euler);

Vector3 velocity_earth = rotateVector(velocity_body, dcm, true); // Body → Earth

// Обратный поворот из земной системы в связанную
Vector3 velocity_body_back = rotateVector(velocity_earth, dcm, false); // Earth → Body
```

## Кинематические преобразования

### Угловая скорость → Производные углов Эйлера

```cpp
auto euler = EulerAngles::ZYX(yaw, pitch, roll);
Vector3 omega_body(0.0, 0.08726646, 0.0); // 5 град/с по тангажу

try {
    Vector3 euler_rates = angularVelocityToEulerRates(euler, omega_body);
    // euler_rates содержит [ψ̇, θ̇, φ̇] в рад/с
} catch (const std::runtime_error& e) {
    // Обработка гимбал-лока
    std::cout << "Gimbal lock detected: " << e.what() << std::endl;
    // Переключение на кватернионную кинематику
}
```

### Производные углов Эйлера → Угловая скорость

```cpp
auto euler = EulerAngles::ZYX(yaw, pitch, roll);
Vector3 euler_rates(0.1, 0.05, 0.02); // [ψ̇, θ̇, φ̇] в рад/с

Vector3 omega_body = eulerRatesToAngularVelocity(euler, euler_rates);
// omega_body содержит [p, q, r] в рад/с
```

## Практические примеры

### Пример 1: Трансформация скорости ракеты

```cpp
#include "math/FlightMath.h"
using namespace core::math;

void transformRocketVelocity() {
    // Ориентация ракеты: 15° тангаж
    auto euler = EulerAngles::ZYX(0.0, 0.2617994, 0.0);
    Matrix3 dcm = eulerToDCM(euler);
    
    // Скорость в связанной системе координат
    Vector3 velocity_body(50.0, 0.0, 0.0); // 50 м/с вперед
    
    // Преобразование в земную систему координат (NED)
    Vector3 velocity_earth = rotateVector(velocity_body, dcm, true);
    
    std::cout << "Velocity in Earth frame: " 
              << velocity_earth.x() << ", " 
              << velocity_earth.y() << ", " 
              << velocity_earth.z() << std::endl;
}
```

### Пример 2: Обработка гимбал-лока

```cpp
Vector3 safeAngularVelocityToEulerRates(const EulerAngles& euler, const Vector3& omega) {
    try {
        return angularVelocityToEulerRates(euler, omega);
    } catch (const std::runtime_error& e) {
        // Логируем предупреждение
        std::cerr << "Warning: " << e.what() << std::endl;
        
        // Переключаемся на кватернионы
        Quaternion quat = eulerToQuaternion(euler);
        
        // Используем кватернионную кинематику (упрощенная версия)
        // В реальном коде здесь должна быть полная кватернионная кинематика
        return Vector3::Zero(); // Заглушка
    }
}
```

### Пример 3: Проверка точности преобразований

```cpp
void validateTransformations() {
    auto euler = EulerAngles::ZYX(0.1, 0.2, 0.3);
    
    // Прямое и обратное преобразование
    Matrix3 dcm = eulerToDCM(euler);
    EulerAngles euler_recovered = dcmToEuler(dcm, EulerConvention::ZYX);
    
    // Проверка точности
    double error1 = std::abs(euler_recovered.angle1 - euler.angle1);
    double error2 = std::abs(euler_recovered.angle2 - euler.angle2);
    double error3 = std::abs(euler_recovered.angle3 - euler.angle3);
    
    const double tolerance = 1e-6;
    if (error1 < tolerance && error2 < tolerance && error3 < tolerance) {
        std::cout << "Transformation accuracy: OK" << std::endl;
    } else {
        std::cout << "Transformation errors: " << error1 << ", " << error2 << ", " << error3 << std::endl;
    }
}
```

## Важные замечания

### Единицы измерения
- **Все углы в радианах** (не в градусах!)
- Для преобразования: `радианы = градусы * π / 180`

### Система координат
- **NED (North-East-Down)** - стандартная система координат
- X - North (север), Y - East (восток), Z - Down (вниз)

### Конвенции углов Эйлера
- **ZYX** - стандартная аэрокосмическая (yaw-pitch-roll)
- **ZXY** - альтернативная для БПЛА
- **YXZ** - специальная конвенция

### Обработка ошибок
- Функции кинематики могут выбрасывать `std::runtime_error` при гимбал-локе
- Всегда используйте try-catch для кинематических преобразований
- При гимбал-локе переключайтесь на кватернионы

### Численная стабильность
- Кватернионы автоматически нормализуются
- Точность преобразований: ~1e-6 для обычных углов
- Для критических вычислений используйте кватернионы

## Миграция со старого кода

### Замена ConeDirection

```cpp
// Старый код:
auto bodyToEarth = TransformationFactory<T>::createBodyToEarthTransform(
    theta, psi, gamma, vector);
auto result = bodyToEarth.result_getter();

// Новый код:
auto euler = EulerAngles::ZYX(psi, theta, gamma);
Matrix3 dcm = eulerToDCM(euler);
auto result = rotateVector(vector, dcm, true);
```

### Замена самописной кинематики

```cpp
// Старый код:
// Самописные функции для вычисления производных

// Новый код:
try {
    Vector3 euler_rates = angularVelocityToEulerRates(euler, omega);
} catch (const std::runtime_error& e) {
    // Обработка гимбал-лока
}
```

## Производительность

- Функции оптимизированы для Release сборки
- Eigen использует SIMD инструкции при наличии
- Кватернионы быстрее для множественных поворотов
- DCM матрицы лучше для однократных преобразований

---

*Для получения дополнительной информации см. исходный код в `include/math/FlightMath.h`*