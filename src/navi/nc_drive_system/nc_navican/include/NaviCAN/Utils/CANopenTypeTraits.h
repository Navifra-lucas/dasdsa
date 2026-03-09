#ifndef NAVIFRA_NAVICAN_CANOPEN_TYPE_TRAITS_H
#define NAVIFRA_NAVICAN_CANOPEN_TYPE_TRAITS_H

#include <lely/co/type.h>
#include <lely/coapp/type_traits.hpp>

#include <cstdint>
#include <string>
#include <type_traits>

namespace NaviFra {
namespace NaviCAN {
namespace Utils {

/**
 * @brief CANopen Type Traits
 *

 * // 런타임 타입
 * if (typeid(T) == typeid(uint16_t)) {
 *     value = dict->getVal<CO_DEFTYPE_UNSIGNED16>(index, subindex);
 * }
 *
 * // 컴파일 타임
 * value = dict->getVal<CANopenTypeTraits<T>::deftype>(index, subindex);
 */

// Primary template - 지원하지 않는 타입에 대한 기본 템플릿
template <typename T, typename = void>
struct CANopenTypeTraits {
    // 8 바이트 데이터 타입 확인 필요 !!!
    static_assert(sizeof(T) == 0, "Unsupported CANopen type");
};

// uint8_t 특수화
template <>
struct CANopenTypeTraits<uint8_t> {
    static constexpr auto deftype = CO_DEFTYPE_UNSIGNED8;
    static constexpr const char* name = "uint8_t";
    static constexpr bool is_signed = false;
    static constexpr size_t size = 1;
};

// uint16_t 특수화
template <>
struct CANopenTypeTraits<uint16_t> {
    static constexpr auto deftype = CO_DEFTYPE_UNSIGNED16;
    static constexpr const char* name = "uint16_t";
    static constexpr bool is_signed = false;
    static constexpr size_t size = 2;
};

// uint32_t 특수화
template <>
struct CANopenTypeTraits<uint32_t> {
    static constexpr auto deftype = CO_DEFTYPE_UNSIGNED32;
    static constexpr const char* name = "uint32_t";
    static constexpr bool is_signed = false;
    static constexpr size_t size = 4;
};

// uint64_t 특수화
template <>
struct CANopenTypeTraits<uint64_t> {
    static constexpr auto deftype = CO_DEFTYPE_UNSIGNED64;
    static constexpr const char* name = "uint64_t";
    static constexpr bool is_signed = false;
    static constexpr size_t size = 8;
};

// int8_t 특수화
template <>
struct CANopenTypeTraits<int8_t> {
    static constexpr auto deftype = CO_DEFTYPE_INTEGER8;
    static constexpr const char* name = "int8_t";
    static constexpr bool is_signed = true;
    static constexpr size_t size = 1;
};

// int16_t 특수화
template <>
struct CANopenTypeTraits<int16_t> {
    static constexpr auto deftype = CO_DEFTYPE_INTEGER16;
    static constexpr const char* name = "int16_t";
    static constexpr bool is_signed = true;
    static constexpr size_t size = 2;
};

// int32_t 특수화
template <>
struct CANopenTypeTraits<int32_t> {
    static constexpr auto deftype = CO_DEFTYPE_INTEGER32;
    static constexpr const char* name = "int32_t";
    static constexpr bool is_signed = true;
    static constexpr size_t size = 4;
};

// int64_t 특수화
template <>
struct CANopenTypeTraits<int64_t> {
    static constexpr auto deftype = CO_DEFTYPE_INTEGER64;
    static constexpr const char* name = "int64_t";
    static constexpr bool is_signed = true;
    static constexpr size_t size = 8;
};

// float 특수화
template <>
struct CANopenTypeTraits<float> {
    static constexpr auto deftype = CO_DEFTYPE_REAL32;
    static constexpr const char* name = "float";
    static constexpr bool is_signed = true;
    static constexpr size_t size = 4;
};

// double 특수화
template <>
struct CANopenTypeTraits<double> {
    static constexpr auto deftype = CO_DEFTYPE_REAL64;
    static constexpr const char* name = "double";
    static constexpr bool is_signed = true;
    static constexpr size_t size = 8;
};

// bool 특수화
template <>
struct CANopenTypeTraits<bool> {
    static constexpr auto deftype = CO_DEFTYPE_BOOLEAN;
    static constexpr const char* name = "bool";
    static constexpr bool is_signed = false;
    static constexpr size_t size = 1;
};

/**
 * @brief 타입 이름 '문자열'에 대한 헬퍼
 *
 */
class CANopenTypeHelper {
public:
    /**
     * @brief 타입 이름 문자열이 유효한 CANopen 타입인지 확인
     */
    static bool isValidTypeName(const std::string& type_name)
    {
        return type_name == "uint8_t" || type_name == "uint16_t" || type_name == "uint32_t" || type_name == "uint64_t" ||
            type_name == "int8_t" || type_name == "int16_t" || type_name == "int32_t" || type_name == "int64_t" || type_name == "float" ||
            type_name == "double" || type_name == "bool";
    }

    /**
     * @brief 타입 이름 문자열에 해당하는 크기(바이트) 반환
     */
    static size_t getTypeSize(const std::string& type_name)
    {
        if (type_name == "uint8_t" || type_name == "int8_t" || type_name == "bool")
            return 1;
        if (type_name == "uint16_t" || type_name == "int16_t")
            return 2;
        if (type_name == "uint32_t" || type_name == "int32_t" || type_name == "float")
            return 4;
        if (type_name == "uint64_t" || type_name == "int64_t" || type_name == "double")
            return 8;
        return 0;
    }

    /**
     * @brief 타입 이름이 부호 있는 타입인지 확인
     */
    static bool isSigned(const std::string& type_name)
    {
        return type_name.find("int") != std::string::npos && type_name[0] != 'u' || type_name == "float" || type_name == "double";
    }
};

/**
 * @brief 템플릿 타입에 대한 편의 별칭
 */
template <typename T>
using CANopenDefType = typename std::integral_constant<decltype(CANopenTypeTraits<T>::deftype), CANopenTypeTraits<T>::deftype>::type;

/**
 * @brief 타입 체크를 위한 컴파일 타임 헬퍼
 */
template <typename T>
struct is_canopen_type
    : std::bool_constant<
          std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t> || std::is_same_v<T, uint32_t> || std::is_same_v<T, uint64_t> ||
          std::is_same_v<T, int8_t> || std::is_same_v<T, int16_t> || std::is_same_v<T, int32_t> || std::is_same_v<T, int64_t> ||
          std::is_same_v<T, float> || std::is_same_v<T, double> || std::is_same_v<T, bool>> {
};

template <typename T>
inline constexpr bool is_canopen_type_v = is_canopen_type<T>::value;

}  // namespace Utils
}  // namespace NaviCAN
}  // namespace NaviFra

#endif  // NAVIFRA_NAVICAN_CANOPEN_TYPE_TRAITS_H
