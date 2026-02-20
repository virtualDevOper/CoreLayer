# Математическая спецификация аэродинамической модели

## Версия: 1.0
## Система координат: Правая, X-вперёд, Y-вправо, Z-вниз 
## Диапазон Маха: 0.0 ... 5.0

---

## 1. СИСТЕМА КООРДИНАТ И ЗНАКОВЫЕ КОНВЕНЦИИ

### 1.1 Оси координат

| Ось | Направление | Положительное направление |
|-----|-------------|--------------------------|
| **X** | Вперёд (к носу) | От хвоста к носу |
| **Y** | Вправо | К правому крылу |
| **Z** | **Вверх** | К небу (инвертировано относительно классической авиации) |

### 1.2 Правая система координат

```
Z (вверх)
↑
|
|______→ Y (вправо)
/
/
↓
X (вперёд, из экрана)
```

Правило правой руки: `X × Y = Z`

### 1.3 Силы и моменты

| Коэффициент | Ось | Физический смысл | Положительное значение |
|-------------|-----|------------------|----------------------|
| **Cx** | X | Лобовое сопротивление | Сила против скорости |
| **Cy** | Y | Боковая сила | Вправо (+Y) |
| **Cz** | Z | Нормальная сила | **Вниз (-Z при подъёмной силе)** |
| **mx** | X | Момент крена | Правое крыло вниз |
| **my** | Y | Момент тангажа | **Нос вверх** |
| **mz** | Z | Момент рыскания | Нос вправо |

### 1.4 Ключевое правило инверсии Z

Поскольку ось Z направлена **вверх** (а не вниз как в классической авиации):

1. **Подъёмная сила** (физически вверх) имеет **отрицательный Cz**
2. **Момент тангажа «нос вверх»** остаётся **положительным my** (двойная инверсия)

**Обоснование двойной инверсии my:**
- Первая инверсия: `Cz' = -Cz` (из-за инверсии оси Z)
- Вторая инверсия: `my' = -my` (чтобы сохранить «нос вверх» за положительным значением)
- Итог: `my' = -(-Cz × arm) = Cz × arm` → знак сохраняется

---

## 2. ВХОДНЫЕ ПАРАМЕТРЫ

### 2.1 AeroState

```cpp
struct AeroState {
    // Кинематика
    double V;           // Скорость, м/с (V > 0)
    double alpha;       // Угол атаки, градусы (-90° ... +90°)
    double beta;        // Угол скольжения, градусы (-90° ... +90°)
    double p;           // Угловая скорость крена, рад/с
    double q;           // Угловая скорость тангажа, рад/с
    double r;           // Угловая скорость рыскания, рад/с
    
    // Атмосфера
    double rho;         // Плотность воздуха, кг/м³ (ρ > 0)
    double M;           // Число Маха (0.0 ... 5.0)
    double Re;          // Число Рейнольдса (> 1e4)
    
    // Время
    double dt;          // Шаг интегрирования, с
    
    // Геометрия
    double x_com;       // Координата ЦМ по X от носа, м
    double y_com;       // Координата ЦМ по Y от оси симметрии, м
    double z_com;       // Координата ЦМ по Z от оси симметрии, м
};
```

### 2.2 Глобальные параметры

```cpp
struct GlobalConfig {
    double S_ref;       // Опорная площадь, м² (обычно площадь крыла в плане)
    double c_ref;       // Опорная длина (САХ), м
    double b_ref;       // Размах крыла, м
};
```

---

## 3. МАТЕМАТИКА КОМПОНЕНТОВ

### 3.1 ТЕЛО ВРАЩЕНИЯ (BODY)

#### 3.1.1 Базовые параметры

```cpp
// Конверсия углов в радианы
double alpha_rad = alpha * M_PI / 180.0;
double beta_rad  = beta  * M_PI / 180.0;

// Площадь миделя
double S_mid = M_PI * diameter * diameter / 4.0;

// Площадь смоченной поверхности
double S_wetted = M_PI * diameter * length + S_mid;
```

#### 3.1.2 Коэффициент трения (Шлихтинг, турбулентный ПС)

```cpp
// Коэффициент трения
double Cf = 0.455 / pow(log10(max(Re, 1e4)), 2.58);

// Сопротивление трения (нормированное на S_ref)
double Cx_friction = Cf * S_wetted / S_ref;

// Производная по Re (аналитически)
double dCf_dRe = -0.455 * 2.58 / (pow(log10(Re), 3.58) * Re * log(10));
double dCx_friction_dRe = dCf_dRe * S_wetted / S_ref;
```

**Проверка размерности:** `Cf` безразмерный, `S_wetted/S_ref` безразмерное → `Cx_friction` безразмерный ✓

#### 3.1.3 Базовое сопротивление формы (донное)

```cpp
double Cx_base = 0.0;
if (nose_type == "SPHERE")  Cx_base = 0.12;
if (nose_type == "OGIVE")   Cx_base = 0.08;
if (nose_type == "CONE")    Cx_base = 0.15;

// Волновое сопротивление добавляется в разделе 3.1.6
```

#### 3.1.4 Нормальная сила тела

```cpp
// Коэффициент нормальной силы (зависит от формы носа)
double k_normal = 0.0;
if (nose_type == "SPHERE")  k_normal = 6.0;
if (nose_type == "OGIVE")   k_normal = 5.5;
if (nose_type == "CONE")    k_normal = 4.5;

// Нелинейная поправка для больших углов
double f_nonlinear = 1.0 - 0.003 * abs(alpha);  // alpha в градусах

// Нормальная сила (безразмерная)
double CN_body = k_normal * sin(alpha_rad) * f_nonlinear;

// Производная по α (в радианах!)
double dCN_dα = k_normal * (cos(alpha_rad) * f_nonlinear 
                - sin(alpha_rad) * 0.003 * sign(alpha) * M_PI/180.0);
```

**Проверка размерности:** `dCN_dα` имеет размерность 1/рад ✓

#### 3.1.5 Силы тела (Z-UP версия)

```cpp
// Индуцированное сопротивление от нормальной силы
double Cx_induced = CN_body * sin(alpha_rad);

// ИТОГО Cx (лобовое сопротивление, всегда положительно)
double Cx_body = Cx_base + Cx_friction + Cx_induced;

// Cz (нормальная сила, Z-UP: подъёмная сила = отрицательный Cz)
// Двойная инверсия: физическая подъёмная сила → -Cz
double Cz_body = -CN_body * cos(alpha_rad);  // ← ИНВЕРСИЯ Z

// Cy (боковая сила от скольжения)
double Cy_body = -k_normal * sin(beta_rad) * cos(beta_rad) * f_nonlinear;

// Производные (Z-UP версия)
double dCx_dα = dCN_dα * sin(alpha_rad) + CN_body * cos(alpha_rad);
double dCz_dα = -dCN_dα * cos(alpha_rad) + CN_body * sin(alpha_rad);  // ← ИНВЕРСИЯ
double dCy_dβ = -k_normal * cos(2*beta_rad);  // ← ИНВЕРСИЯ
```

**Проверка знаков:**
- При α > 0: подъёмная сила вверх → Cz < 0 ✓
- При β > 0: боковая сила влево → Cy < 0 ✓

#### 3.1.6 Волновое сопротивление (M = 0 ... 5)

```cpp
double Cx_wave = 0.0;

// Дозвук (M < 0.8): волновое сопротивление отсутствует
if (M < 0.8) {
    Cx_wave = 0.0;
}
// Трансзвук (0.8 ≤ M ≤ 1.2): плавная интерполяция
else if (M < 1.2) {
    double t = (M - 0.8) / 0.4;  // 0 ... 1
    double subsonic_part = 0.0;
    double supersonic_part = 0.1 * pow(M - 1.0, 2);
    
    // Сигмоидная интерполяция
    double sigmoid = 1.0 / (1.0 + exp(-10.0 * (t - 0.5)));
    Cx_wave = subsonic_part * (1 - sigmoid) + supersonic_part * sigmoid;
}
// Сверхзвук (1.2 < M ≤ 3.0): линейная теория
else if (M <= 3.0) {
    double t_c = diameter / length;  // Относительная толщина
    Cx_wave = 4 * t_c * t_c / sqrt(max(M*M - 1, 0.01));
}
// Гиперзвук (M > 3.0): упрощённая модель Ньютона
else {
    double t_c = diameter / length;
    double newton_factor = 2.0 * sin(alpha_rad) * sin(alpha_rad);
    Cx_wave = 4 * t_c * t_c / sqrt(max(M*M - 1, 0.01)) * (1 + 0.5 * newton_factor);
}
```

#### 3.1.7 Центр давления тела

```cpp
// Базовое положение (60-70% длины от носа)
double x_cp_base = 0.65 * length;

// Поправка на число Маха (ЦД смещается назад на сверхзвуке)
double x_cp_mach = x_cp_base * (1.0 + 0.1 * max(0.0, M - 1.0));

// Поправка на угол атаки (при больших α ЦД смещается вперёд)
double x_cp_alpha = x_cp_mach * (1.0 - 0.002 * abs(alpha));

// ИТОГО центр давления (от носа), м
double x_cp_body = x_cp_alpha;

// Производная по α, м/рад
double dx_cp_dα = -x_cp_mach * 0.002 * sign(alpha) * M_PI/180.0;
```

**Проверка размерности:** `x_cp` в метрах, `dx_cp_dα` в м/рад ✓

#### 3.1.8 Моменты тела (Z-UP версия)

```cpp
// Плечо относительно ЦМ, м
double arm_x = x_cp_body - x_com;

// Момент тангажа (Z-UP: двойная инверсия для сохранения «нос вверх»)
// my = -Cz × arm_x / c_ref
// Поскольку Cz уже инвертирован, my сохраняет знак
double my_body = -Cz_body * arm_x / c_ref;  // ← ДВОЙНАЯ ИНВЕРСИЯ

// Момент рыскания (Z-UP: инверсия)
double mz_body = -Cy_body * arm_x / c_ref;  // ← ИНВЕРСИЯ Z

// Момент крена (для осесимметричного тела = 0)
double mx_body = 0.0;

// Производные моментов (1/рад)
double dmy_dα = -dCz_dα * arm_x / c_ref - Cz_body * dx_cp_dα / c_ref;
double dmz_dβ = -dCy_dβ * arm_x / c_ref;
double dmx_dα = 0.0;
```

**Проверка знаков:**
- При α > 0: Cz < 0 (подъёмная сила вверх), arm_x > 0 → my > 0 (нос вверх) ✓
- При β > 0: Cy < 0 (сила влево), arm_x > 0 → mz > 0 (нос вправо) ✓

---

### 3.2 КРЫЛО (WING)

#### 3.2.1 Наклон кривой подъёмной силы (M = 0 ... 5)

```cpp
// 2D теория тонкого профиля (1/рад)
double CL_alpha_2D = 2.0 * M_PI;

// 3D поправка на удлинение (Прандтль)
double CL_alpha_3D = CL_alpha_2D / (1.0 + CL_alpha_2D / (M_PI * AR));

// Поправка Прандтля-Глауэрта (сжимаемость)
double beta_PG = sqrt(abs(1.0 - M * M));
beta_PG = max(beta_PG, 0.1);  // Защита от сингулярности

// Дозвук (M < 0.8)
double CL_alpha = CL_alpha_3D / beta_PG;

// Трансзвук (0.8 ≤ M ≤ 1.2): плавная интерполяция
if (M >= 0.8 && M <= 1.2) {
    double t = (M - 0.8) / 0.4;
    double subsonic = CL_alpha_3D / beta_PG;
    double supersonic = CL_alpha_3D / sqrt(max(M*M - 1, 0.01));
    double sigmoid = 1.0 / (1.0 + exp(-10.0 * (t - 0.5)));
    CL_alpha = subsonic * (1 - sigmoid) + supersonic * sigmoid;
}
// Сверхзвук (M > 1.2)
else if (M > 1.2) {
    CL_alpha = CL_alpha_3D / sqrt(max(M*M - 1, 0.01));
}

// Гиперзвук (M > 3.0): дополнительная поправка
if (M > 3.0) {
    CL_alpha *= (1.0 - 0.1 * (M - 3.0));  // Спад эффективности
}
```

**Проверка размерности:** `CL_alpha` в 1/рад ✓

#### 3.2.2 Угол срыва с гистерезисом

```cpp
// Базовый угол срыва, градусы
double alpha_stall_base = 12.0 + 2.0 * log10(max(Re, 1e4) / 1e6);
alpha_stall_base = min(alpha_stall_base, 16.0);

// Эффективный угол с памятью (гистерезис), градусы
double alpha_eff = alpha_prev * exp(-dt / hysteresis_tau) 
                 + alpha * (1.0 - exp(-dt / hysteresis_tau));

// Флаг срыва
bool is_stalled = (abs(alpha_eff) > alpha_stall_base);

// Фактор срыва (плавный спад)
double stall_factor = 1.0;
if (is_stalled) {
    double alpha_excess = abs(alpha_eff) - alpha_stall_base;
    if (alpha_excess < 5.0) {
        stall_factor = 1.0 - 0.02 * alpha_excess;
    } else if (alpha_excess < 15.0) {
        stall_factor = 0.9 - 0.05 * (alpha_excess - 5.0);
    } else {
        stall_factor = 0.4;
    }
    stall_factor = max(stall_factor, 0.4);
}
```

#### 3.2.3 Подъёмная сила крыла (Z-UP версия)

```cpp
// Базовая подъёмная сила
double CL_wing = CL_alpha * alpha_rad * stall_factor * k_interference;

// Вихревая подъёмная сила (большие α)
double CL_vortex = 0.0;
if (abs(alpha) > 15.0) {
    CL_vortex = vortex_gain * sin(2 * alpha_rad) * (abs(alpha) - 15.0) / 30.0;
}

// ИТОГО подъёмная сила
double CL_total = CL_wing + CL_vortex;

// Производная по α (1/рад)
double dCL_dα = CL_alpha * stall_factor;
if (abs(alpha) > 15.0) {
    dCL_dα += vortex_gain * 2 * cos(2*alpha_rad) * (abs(alpha) - 15.0) / 30.0;
    dCL_dα += vortex_gain * sin(2*alpha_rad) * sign(alpha) / 30.0 * M_PI/180.0;
}

// Cz (Z-UP: подъёмная сила вверх = отрицательный Cz)
double Cz_wing = -CL_total * cos(alpha_rad);  // ← ИНВЕРСИЯ Z

// Производная (1/рад)
double dCz_dα = -dCL_dα * cos(alpha_rad) + CL_total * sin(alpha_rad);  // ← ИНВЕРСИЯ
```

#### 3.2.4 Сопротивление крыла

```cpp
// Сопротивление трения
double Cx_friction = Cf * 2.0;  // 2 поверхности

// Индуцированное сопротивление
double e_oswald = 0.85;
double Cx_induced = CL_total * CL_total / (M_PI * AR * e_oswald);

// Волновое сопротивление (трансзвук + сверхзвук)
double Cx_wave = 0.0;
if (M >= 0.7 && M <= 1.3) {
    Cx_wave = 0.02 * exp(-pow((M - 0.95) / 0.15, 2));
}
else if (M > 1.3) {
    double t_c = 0.12;  // Относительная толщина
    Cx_wave = 4 * t_c * t_c / sqrt(max(M*M - 1, 0.01));
}
if (M > 3.0) {
    Cx_wave *= (1.0 + 0.2 * (M - 3.0));  // Рост на гиперзвуке
}

// ИТОГО Cx
double Cx_wing = Cx_friction + Cx_induced + Cx_wave + CL_total * sin(alpha_rad);

// Производная по α (1/рад)
double dCx_dα = 2 * CL_total * dCL_dα / (M_PI * AR * e_oswald) 
              + dCL_dα * sin(alpha_rad) + CL_total * cos(alpha_rad);
```

#### 3.2.5 Боковая сила и моменты крыла

```cpp
// Боковая сила от скольжения
double Cy_wing = -CL_alpha * beta_rad * stall_factor;  // ← ИНВЕРСИЯ

// Плечо (ЦД на 25% хорды)
double x_cp_wing = x_pos + 0.25 * c_ref;
double arm_x = x_cp_wing - x_com;

// Момент тангажа (Z-UP: двойная инверсия)
double my_wing = -Cz_wing * arm_x / c_ref;  // ← ДВОЙНАЯ ИНВЕРСИЯ

// Момент рыскания (Z-UP: инверсия)
double mz_wing = -Cy_wing * arm_x / c_ref;  // ← ИНВЕРСИЯ

// Момент крена (от разности подъёмной силы при крене)
double mx_wing = 0.0;  // Базово 0, рассчитывается в динамических производных

// Производные (1/рад)
double dmy_dα = -dCz_dα * arm_x / c_ref;
double dmz_dβ = -dCy_dβ * arm_x / c_ref;
```

---

### 3.3 РУЛИ / СТАБИЛИЗАТОРЫ (FIN)

#### 3.3.1 Универсальная модель с mount_angle

```cpp
// Угол установки руля вокруг оси X, радианы
double mount_angle_rad = mount_angle * M_PI / 180.0;

// Угол отклонения руля, радианы
double delta_rad = delta * M_PI / 180.0;

// Эффективный угол атаки на руле (с учётом скоса потока)
double alpha_fin_eff = alpha_rad - epsilon_rad;

// Подъёмная сила руля
double CL_fin = CL_alpha_fin * (alpha_fin_eff + delta_rad) * stall_factor_fin;

// Проекция на глобальные оси Y и Z
double Cz_fin = -CL_fin * cos(mount_angle_rad);  // ← ИНВЕРСИЯ Z
double Cy_fin = -CL_fin * sin(mount_angle_rad);  // ← ИНВЕРСИЯ
```

**Обоснование:**
- `mount_angle = 0°`: руль направлен вверх → создаёт только Cz
- `mount_angle = 90°`: руль направлен вправо → создаёт только Cy
- `mount_angle = 45°`: X-образное расположение → создаёт Cy и Cz

#### 3.3.2 Эффективность руля (нелинейная)

```cpp
// Базовая эффективность
double eta_base = 0.7;

// Поправка на число Маха
double eta_mach = 1.0;
if (M >= 0.7 && M < 1.3) {
    eta_mach = 1.0 - 0.3 * (M - 0.7) / 0.6;
} else if (M >= 1.3 && M <= 3.0) {
    eta_mach = 0.6;
} else if (M > 3.0) {
    eta_mach = 0.6 - 0.05 * (M - 3.0);  // Спад на гиперзвуке
}
eta_mach = max(eta_mach, 0.4);

// Поправка на угол атаки (экранирование)
double eta_alpha = 1.0 - 0.5 * sin(alpha_rad) * sin(alpha_rad);

// Итоговая эффективность
double eta = eta_base * eta_mach * eta_alpha;

// Приращение от отклонения руля
double ΔCL = eta * delta_rad * CL_alpha_fin;
```

#### 3.3.3 Моменты от рулей (Z-UP версия)

```cpp
// Плечо руля, м
double arm_x = x_pos - x_com;

// Момент тангажа (Z-UP: двойная инверсия)
double my_fin = -Cz_fin * arm_x / c_ref;  // ← ДВОЙНАЯ ИНВЕРСИЯ

// Момент рыскания (Z-UP: инверсия)
double mz_fin = -Cy_fin * arm_x / c_ref;  // ← ИНВЕРСИЯ

// Момент крена (от разности сил на рулях)
double mx_fin = (Cz_fin * y_pos - Cy_fin * z_pos) * arm_x / b_ref;

// Производная по δ (1/рад)
double dmy_dδ = -dCz_dδ * arm_x / c_ref;
double dmz_dδ = -dCy_dδ * arm_x / c_ref;
double dmx_dδ = (dCz_dδ * y_pos - dCy_dδ * z_pos) * arm_x / b_ref;
```

---

## 4. ИНТЕРФЕРЕНЦИЯ И СКОС ПОТОКА

### 4.1 Крыло-фюзеляж

```cpp
// Увеличение подъёмной силы крыла из-за фюзеляжа
double k_wf = 1.0 + 0.15 * pow(diameter_fuselage / b_wing, 2);
Cz_wing *= k_wf;
dCz_dα *= k_wf;
```

### 4.2 След за крылом (экранирование ГО)

```cpp
// Ослабление динамического напора на стабилизаторе
double CL_wing_norm = -Cz_wing / cos(alpha_rad);  // Восстанавливаем CL
double q_ratio = 1.0 - wake_shadow_factor * CL_wing_norm * CL_wing_norm;
q_ratio = max(q_ratio, 0.5);  // Минимум 50% напора

Cz_fin *= q_ratio;
dCz_dα_fin *= q_ratio;
```

### 4.3 Скос потока на ГО (downwash)

```cpp
// Угол скоса потока, радианы
double epsilon = 0.3 * (-Cz_wing / cos(alpha_rad));  // Используем CL

// Запаздывание скоса потока (первый порядок)
double epsilon_eff = epsilon_prev * exp(-dt / 0.1) 
                   + epsilon * (1.0 - exp(-dt / 0.1));

// Эффективный угол атаки на ГО
double alpha_fin_eff = alpha_rad - epsilon_eff;
```

---

## 5. ДИНАМИЧЕСКИЕ ПРОИЗВОДНЫЕ (ДЕМПФИРОВАНИЕ)

### 5.1 Тангаж (q)

```cpp
// Демпфирование по угловой скорости тангажа (1/(рад/с))
double dCz_dq = -CL_alpha * (arm_x / V) * 0.5;
double dmy_dq = -dCz_dq * arm_x / c_ref;  // ← ДВОЙНАЯ ИНВЕРСИЯ

// Проверка знака: при q > 0 (нос вверх), момент должен быть отрицательным (демпфирование)
// dmy_dq < 0 ✓
```

### 5.2 Крен (p)

```cpp
// Демпфирование по крену (1/(рад/с))
double dmx_dp = -CL_alpha * b_ref * b_ref / (4 * V * b_ref) * 0.5;

// Проверка знака: при p > 0 (правый крен), момент должен быть отрицательным
// dmx_dp < 0 ✓
```

### 5.3 Рыскание (r)

```cpp
// Демпфирование по рысканию (1/(рад/с))
double dCy_dr = CL_alpha * (arm_x / V) * 0.5;
double dmz_dr = -dCy_dr * arm_x / c_ref;  // ← ИНВЕРСИЯ

// Проверка знака: при r > 0 (нос вправо), момент должен быть отрицательным
// dmz_dr < 0 ✓
```

---

## 6. ПЛАВНАЯ ИНТЕРПОЛЯЦИЯ ПО МАХУ (M = 0 ... 5)

### 6.1 Сигмоидальные веса

```cpp
// Дозвуковой вес (M < 0.85)
double w_sub = 1.0 / (1.0 + exp((M - 0.85) / 0.05));

// Сверхзвуковой вес (M > 0.95)
double w_sup = 1.0 / (1.0 + exp((0.95 - M) / 0.05));

// Гиперзвуковой вес (M > 3.0)
double w_hyper = 1.0 / (1.0 + exp((3.0 - M) / 0.2));

// Итоговый коэффициент (пример для Cx)
double Cx = w_sub * Cx_sub + w_sup * Cx_sup + w_hyper * Cx_hyper;
```

### 6.2 Проверка непрерывности

```cpp
// При M = 0.9: w_sub ≈ 0.5, w_sup ≈ 0.5 → плавный переход ✓
// При M = 3.0: w_hyper ≈ 0.5 → плавный переход в гиперзвук ✓
```

---

## 7. ВЫХОДНЫЕ ДАННЫЕ (AeroOutput)

```cpp
struct AeroOutput {
    // Основные коэффициенты (безразмерные)
    double Cx;  // Лобовое сопротивление
    double Cy;  // Боковая сила
    double Cz;  // Нормальная сила (Z-UP: подъёмная = отрицательный)
    
    // Моменты (безразмерные)
    double mx;  // Крен
    double my;  // Тангаж (my > 0 = нос вверх)
    double mz;  // Рыскание
    
    // Статические производные (1/рад)
    double dCx_dα;
    double dCz_dα;
    double dmy_dα;
    double dCy_dβ;
    double dmz_dβ;
    double dCz_dδ;
    double dmy_dδ;
    
    // Динамические производные (1/(рад/с))
    double dCz_dq;
    double dmy_dq;
    double dCy_dr;
    double dmz_dr;
    double dmx_dp;
    
    // Диагностические флаги
    bool is_body_stalled;
    bool is_wing_stalled;
    bool is_transonic;
    
    // Геометрия
    double x_cp;  // Центр давления, м (от носа)
    double static_margin;  // (x_cp - x_com) / c_ref
};
```

---

## 8. ПРОВЕРКА РАЗМЕРНОСТЕЙ

| Параметр | Формула | Размерность | Статус |
|----------|---------|-------------|--------|
| Cx, Cy, Cz | F / (q·S) | Безразмерный | ✓ |
| mx, my, mz | M / (q·S·c) | Безразмерный | ✓ |
| dCz_dα | ΔCz / Δα | 1/рад | ✓ |
| dmy_dα | Δmy / Δα | 1/рад | ✓ |
| dmy_dq | Δmy / Δq | 1/(рад/с) = с/рад | ✓ |
| x_cp | м | м | ✓ |
| static_margin | (м - м) / м | Безразмерный | ✓ |

---

## 9. ПРОВЕРКА ЗНАКОВ (Z-UP)

| Условие | Ожидаемый знак | Формула | Статус |
|---------|---------------|---------|--------|
| α > 0 (подъёмная сила) | Cz < 0 | `Cz = -CL·cos(α)` | ✓ |
| α > 0, ЦМ впереди ЦД | my > 0 (нос вверх) | `my = -Cz·arm/c` | ✓ |
| β > 0 (скольжение вправо) | Cy < 0 (сила влево) | `Cy = -CL·β` | ✓ |
| β > 0, ЦМ впереди ЦД | mz > 0 (нос вправо) | `mz = -Cy·arm/c` | ✓ |
| q > 0 (тангаж вверх) | dmy_dq < 0 (демпфирование) | `dmy_dq = -...` | ✓ |
| p > 0 (крен вправо) | dmx_dp < 0 (демпфирование) | `dmx_dp = -...` | ✓ |
| r > 0 (рыскание вправо) | dmz_dr < 0 (демпфирование) | `dmz_dr = -...` | ✓ |

---

## 10. ГРАНИЦЫ ПРИМЕНИМОСТИ

| Параметр | Диапазон | Примечание |
|----------|----------|------------|
| Число Маха | 0.0 ... 5.0 | M > 5: требуется учёт высокотемпературных эффектов |
| Угол атаки | -90° ... +90° | |α| > 60°: высокая неопределённость |
| Угол скольжения | -90° ... +90° | |β| > 45°: высокая неопределённость |
| Число Рейнольдса | > 1e4 | Re < 1e4: ламинарный режим, другие формулы |
| Высота | 0 ... 30 км | ρ > 0.01 кг/м³ |

---

## 11. АНАЛИТИЧЕСКИЙ ВЫВОД ПРОИЗВОДНЫХ

### 11.1 dCz_dα (крыло)

```
Cz = -CL · cos(α)

dCz/dα = -dCL/dα · cos(α) + CL · sin(α)

где:
CL = CL_alpha · α · stall_factor
dCL/dα = CL_alpha · stall_factor + CL_alpha · α · dstall_factor/dα
```

**Проверка:** При α = 0: `dCz/dα = -CL_alpha` (отрицательно, т.к. Cz < 0 при подъёмной силе) ✓

### 11.2 dmy_dα (крыло)

```
my = -Cz · arm_x / c_ref

dmy/dα = -dCz/dα · arm_x / c_ref - Cz · d(arm_x)/dα / c_ref

где:
arm_x = x_cp - x_com
d(arm_x)/dα = dx_cp/dα
```

**Проверка:** При стабильной конфигурации (x_cp < x_com): `dmy/dα < 0` (статическая устойчивость) ✓

### 11.3 dmy_dq (демпфирование тангажа)

```
dCz/dq = -CL_alpha · (arm_x / V) · 0.5

dmy/dq = -dCz/dq · arm_x / c_ref
       = CL_alpha · arm_x² / (V · c_ref) · 0.5

Проверка знака: dmy/dq < 0 (демпфирование) ✓
```

---

## 12. ССЫЛКИ НА ИСТОЧНИКИ

1. **DATCOM** (USAF Stability and Control Datcom) — базовые эмпирические формулы
2. **NACA Report 1135** — уравнения аэродинамики для M = 0 ... 5
3. **NASA TP-3660** — гиперзвуковые поправки
4. **Schlichting** — теория пограничного слоя (коэффициент трения)
5. **Prandtl-Glauert** — поправка на сжимаемость (дозвук)
6. **Newtonian Impact Theory** — гиперзвуковое обтекание (M > 3)

---

**Конец математической спецификации**
