//
// Created by ubuntu on 24. 6. 9.
//

#include "degree_convert.hpp"
#define SCALE_IMU_OFFSET 1000
#define STANDARD_IMU_OFFSET 500
#define DIGIT_IMU_OFFSET 10
#define DIGIT_STR "0."

/**
 * @brief 입력값을 각도와 소수 부분으로 분리하는 함수
 * @param str_input
 * @return 각도, 오프셋
 */
std::pair<int, double> DegreeConvert::parse_input(float float_input) {
    int degrees = static_cast<int>(float_input);
    float fraction = float_input - static_cast<float>(degrees);
    return {degrees, fraction};
}
/**
 * @brief 소수 부분을 1000을 기준으로 변환하는 함수
 * */
double DegreeConvert::convert_fraction(double fraction) {
    double scaled_fraction = fraction * SCALE_IMU_OFFSET;
    if (scaled_fraction > STANDARD_IMU_OFFSET) {
        scaled_fraction -= SCALE_IMU_OFFSET;
    }
    return scaled_fraction / DIGIT_IMU_OFFSET;
}