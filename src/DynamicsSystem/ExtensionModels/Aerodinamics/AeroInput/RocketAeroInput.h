//
// Created by 4NR_Operator_3 on 13.10.2025.
//

#pragma once

template <typename metricType>
struct RocketAeroInput {

    metricType L = 0.0;
    metricType D_mid = 0.0;
    metricType L_har = 0.0;
    metricType Xdp = 0.0;
    metricType Xdst = 0.0;

    metricType L_sum_kr = 0.0;
    metricType b_0_kr = 0.0;
    metricType b_1_kr = 0.0;
    metricType hi_st = 0.0;
    metricType d_kr = 0.0;
    metricType S_kr = 0.0;
    metricType l2_kr = 0.0;
    metricType X_p_kr_st = 0.0;

    int rudder_count = 0;
    metricType L_sum_p = 0.0;
    metricType b_0_p = 0.0;
    metricType b_1_p = 0.0;
    metricType hi_p = 0.0;
    metricType d_p = 0.0;
    metricType S_p = 0.0;
    metricType l_2_p = 0.0;
    metricType X_p_kr_p = 0.0;
    metricType X_ov_p = 0.0;
    metricType b_sah_p = 0.0;
};