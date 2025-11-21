//
// Created by 4NR_Operator_3 on 16.09.2025.
//
#pragma once

template <typename metricType>
Eigen::Vector3<metricType> NoWindModel<metricType>::getWindVector( Eigen::Vector3<metricType> r, metricType t)const{
      return Eigen::Vector3<metricType>::Zero();  // Используем встроенный метод Eigen для нуля
}

template <typename metricType>
metricType NoWindModel<metricType>::getWindSpeed( Eigen::Vector3<metricType> r, metricType t)const {
       return static_cast<metricType>(0);  // Явное приведение к metricType
   }

template <typename metricType>// Возвращает нулевое направление (азимут и угол возвышения)
std::pair<metricType, metricType> NoWindModel<metricType>::getWindDirection( Eigen::Vector3<metricType> r, metricType t)const {
       return {static_cast<metricType>(0), static_cast<metricType>(0)};  // Явное приведение
   }