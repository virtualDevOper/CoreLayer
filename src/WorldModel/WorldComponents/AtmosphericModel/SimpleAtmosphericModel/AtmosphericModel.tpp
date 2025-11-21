//
// Created by 4NR_Operator_3 on 16.09.2025.
//
#pragma once

template <typename metricType>
metricType AtmosphericModel<metricType>::getDensity(const Eigen::Vector3<metricType> r) const  {
    metricType h = r.z();  // Высота (предполагаем z - вверх)
    // Проверка на допустимую высоту (до тропопаузы, около 11 км)
    if (h > static_cast<metricType>(PhysicsConstants::maxHeight) || h < static_cast<metricType>(PhysicsConstants::minHeight)) {
        // Для высот выше 11 км или ниже 0м используем простую модель или возвращаем 0
        // Здесь можно добавить более сложную модель для стратосферы, но для простоты:
        return static_cast<metricType>(0.0);
    }
    // Расчёт плотности по формуле стандартной атмосферы
    metricType exponent = (PhysicsConstants::g * PhysicsConstants::molarMass) / (PhysicsConstants::R * PhysicsConstants::L) - static_cast<metricType>(1.0);
    metricType base = static_cast<metricType>(1.0) - (PhysicsConstants::L * h) / PhysicsConstants::T0;
    // Проверка, чтобы base не стал отрицательным (что может произойти при h > T0/L ≈ 44.4 км)
    if (base <= static_cast<metricType>(0.0)) {
        return static_cast<metricType>(0.0);
    }
    return PhysicsConstants::rho0 * std::pow(base, exponent);
}

// Давление: аналогично, P = P0 * exp(-h/H)
template <typename metricType>
metricType AtmosphericModel<metricType>::getPressure(const Eigen::Vector3<metricType> r) const  {
    metricType h = r.z();
    metricType rho = this->getDensity(r);
    return PhysicsConstants::P0 * std::exp(-rho * PhysicsConstants::g * h / PhysicsConstants::P0);
}

// Температура: линейное падение с высотой (T = T0_kelvin - L * h)
template <typename metricType>
metricType AtmosphericModel<metricType>::getTemperature(const Eigen::Vector3<metricType> r) const  {
    metricType h = r.z();
    return PhysicsConstants::T0 - PhysicsConstants::L * h;
}

// Скорость звука: a = sqrt(gamma * R * T)
template <typename metricType>
metricType AtmosphericModel<metricType>::getSpeedOfSound(const Eigen::Vector3<metricType> r) const  {
    metricType T = getTemperature(r);
    // Газовая постоянная тут рассчитана через R/M
    return std::sqrt(PhysicsConstants::gamma * (PhysicsConstants::R/PhysicsConstants::molarMass) * T);
}