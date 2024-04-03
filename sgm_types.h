#pragma once

#include <cstdint>
#include <limits>

/** \brief float无效值 */
constexpr auto Invalid_Float = std::numeric_limits<float>::infinity();      //无穷大的值
//constexpr表达式是指值不会改变并且在编译过程就能得到计算结果的表达式。
//声明为constexpr的变量一定是一个const变量，而且必须用常量表达式初始化

/** \brief 基础类型别名 */
typedef int8_t      sint8;      // 有符号8位整数    unsigned char
typedef uint8_t     uint8;      // 无符号8位整数    uint8_tsigned char
typedef int16_t     sint16;     // 有符号16位整数   int16_tshort int
typedef uint16_t    uint16;     // 无符号16位整数   unsigned short int
typedef int32_t     sint32;     // 有符号32位整数   int
typedef uint32_t    uint32;     // 无符号32位整数   unsigned int
typedef int64_t     sint64;     // 有符号64位整数   long int
typedef uint64_t    uint64;     // 无符号64位整数   unsigned long int
typedef float       float32;    // 单精度浮点
typedef double      float64;    // 双精度浮点