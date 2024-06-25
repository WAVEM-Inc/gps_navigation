//
// Created by ubuntu on 24. 6. 9.
//

#include "degree_convert.hpp"
#include <cmath>
#include <tuple>
#define SCALE_IMU_OFFSET 1000
#define STANDARD_IMU_OFFSET 500
#define DIGIT_IMU_OFFSET 10
#define DIGIT_STR "0."
#define DIGIT_DIVIDE 10000.0
#define MAX_DATA 2000
#define ZERO 0

#define MAX_ANGLE 45.0
#define MAX_WHEEL 30.0
/**
 * @brief 입력값을 각도와 소수 부분으로 분리하는 함수
 * @param str_input
 * @return 각도, 오프셋
 */
std::tuple<int, double> DegreeConvert::parse_input(double input) {
    int degrees = static_cast<int>(input);
    double fraction = extract_fractional_part(input);
    if (fraction == ZERO) {
        return std::make_tuple(degrees, ZERO);
    }
    // Adjust the fractional part based on the specified logic
    fraction -= SCALE_IMU_OFFSET;
    fraction /= DIGIT_IMU_OFFSET;
    return std::make_tuple(degrees, fraction);
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

int DegreeConvert::extract_fractional_part(double input) {
    // Calculate the fractional part
    double fractional_part = input - static_cast<int>(input);
    // Round the fifth decimal place
    fractional_part = std::round(fractional_part * DIGIT_DIVIDE) / DIGIT_DIVIDE;
    // Convert the fractional part to an integer by shifting the decimal point
    int fractional_digits = static_cast<int>(fractional_part * DIGIT_DIVIDE); // Multiply by 10000 to capture up to 4 decimal places
    if (fractional_digits>=MAX_DATA) {
        return ZERO;
    }
    return fractional_digits;
}

/**
 * @brief 45도 이내 각도를 30도 내로 변환하여 반환 _ 비율 계산
 * @brief 45도 이상 각도는 45도로 처리
 * @param input_angle
 * @return double
 */
double DegreeConvert::convert_wheel_angle(double input_angle) {
    if (input_angle > MAX_ANGLE) {
        input_angle = MAX_ANGLE;
    }
    // 0도에서 45도 사이의 입력 각도를 0에서 30도로 변환
    return (input_angle / MAX_ANGLE) * MAX_WHEEL;
}
