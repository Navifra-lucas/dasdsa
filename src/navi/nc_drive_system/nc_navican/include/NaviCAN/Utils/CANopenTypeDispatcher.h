#ifndef NAVIFRA_NAVICAN_CANOPEN_TYPE_DISPATCHER_H
#define NAVIFRA_NAVICAN_CANOPEN_TYPE_DISPATCHER_H

#include <lely/co/type.h>
#include <cstdint>
#include <stdexcept>
#include <type_traits>

namespace NaviFra {
namespace NaviCAN {
namespace Utils {

/**
 * @brief CANopen Type Dispatcher
 *
 * CO_DEFTYPE 값을 기반으로 적절한 C++ 타입으로 함수를 디스패치합니다.
 * 이를 통해 타입별 분기 코드의 중복을 제거합니다.
 *
 * @example
 * // 기존 방식 (중복 코드)
 * if (co_def == CO_DEFTYPE_UNSIGNED8) {
 *     uint8_t val = ...;
 *     sub->setVal<CO_DEFTYPE_UNSIGNED8>(val);
 * }
 * if (co_def == CO_DEFTYPE_UNSIGNED16) {
 *     uint16_t val = ...;
 *     sub->setVal<CO_DEFTYPE_UNSIGNED16>(val);
 * }
 * // ... 반복
 *
 * // 새로운 방식 (dispatcher 사용)
 * dispatchByCoDefType(co_def, [&]<typename T>() {
 *     T val = ...;
 *     sub->setVal<CANopenTypeTraits<T>::deftype>(val);
 * });
 */

/**
 * @brief CO_DEFTYPE을 C++ 타입으로 매핑하는 헬퍼
 */
template<uint8_t CoDefType>
struct CoDefTypeToCppType;

template<> struct CoDefTypeToCppType<CO_DEFTYPE_UNSIGNED8> { using type = uint8_t; };
template<> struct CoDefTypeToCppType<CO_DEFTYPE_INTEGER8> { using type = int8_t; };
template<> struct CoDefTypeToCppType<CO_DEFTYPE_UNSIGNED16> { using type = uint16_t; };
template<> struct CoDefTypeToCppType<CO_DEFTYPE_INTEGER16> { using type = int16_t; };
template<> struct CoDefTypeToCppType<CO_DEFTYPE_UNSIGNED32> { using type = uint32_t; };
template<> struct CoDefTypeToCppType<CO_DEFTYPE_INTEGER32> { using type = int32_t; };
template<> struct CoDefTypeToCppType<CO_DEFTYPE_UNSIGNED64> { using type = uint64_t; };
template<> struct CoDefTypeToCppType<CO_DEFTYPE_INTEGER64> { using type = int64_t; };
template<> struct CoDefTypeToCppType<CO_DEFTYPE_REAL32> { using type = float; };
template<> struct CoDefTypeToCppType<CO_DEFTYPE_REAL64> { using type = double; };

/**
 * @brief CO_DEFTYPE 값에 따라 적절한 타입으로 함수를 디스패치
 *
 * @tparam Func 템플릿 람다 또는 functor 타입
 * @param co_def CO_DEFTYPE 값
 * @param func 타입을 템플릿 파라미터로 받는 함수 객체
 * @return 함수의 반환값
 *
 * @example
 * uint32_t result = dispatchByCoDefType(co_def, [&]<typename T>() -> uint32_t {
 *     T value = getValue<T>();
 *     return static_cast<uint32_t>(value);
 * });
 */
template<typename Func>
auto dispatchByCoDefType(uint8_t co_def, Func&& func) {
    switch (co_def) {
        case CO_DEFTYPE_UNSIGNED8:
            return func.template operator()<uint8_t>();
        case CO_DEFTYPE_INTEGER8:
            return func.template operator()<int8_t>();
        case CO_DEFTYPE_UNSIGNED16:
            return func.template operator()<uint16_t>();
        case CO_DEFTYPE_INTEGER16:
            return func.template operator()<int16_t>();
        case CO_DEFTYPE_UNSIGNED32:
            return func.template operator()<uint32_t>();
        case CO_DEFTYPE_INTEGER32:
            return func.template operator()<int32_t>();
        case CO_DEFTYPE_REAL32:
            return func.template operator()<float>();
        // 8바이트 타입은 COData(uint32_t)에 맞지 않아 제거
        // case CO_DEFTYPE_UNSIGNED64:
        // case CO_DEFTYPE_INTEGER64:
        // case CO_DEFTYPE_REAL64:
        default:
            throw std::runtime_error("Unsupported CO_DEFTYPE: " + std::to_string(co_def));
    }
}

/**
 * @brief void 반환 타입을 위한 특수화
 *
 * 반환값이 없는 함수를 위한 편의 오버로드
 */
template<typename Func>
void dispatchByCoDefTypeVoid(uint8_t co_def, Func&& func) {
    switch (co_def) {
        case CO_DEFTYPE_UNSIGNED8:
            func.template operator()<uint8_t>();
            break;
        case CO_DEFTYPE_INTEGER8:
            func.template operator()<int8_t>();
            break;
        case CO_DEFTYPE_UNSIGNED16:
            func.template operator()<uint16_t>();
            break;
        case CO_DEFTYPE_INTEGER16:
            func.template operator()<int16_t>();
            break;
        case CO_DEFTYPE_UNSIGNED32:
            func.template operator()<uint32_t>();
            break;
        case CO_DEFTYPE_INTEGER32:
            func.template operator()<int32_t>();
            break;
        case CO_DEFTYPE_REAL32:
            func.template operator()<float>();
            break;
        // 8바이트 타입은 COData(uint32_t)에 맞지 않아 제거
        // case CO_DEFTYPE_UNSIGNED64:
        // case CO_DEFTYPE_INTEGER64:
        // case CO_DEFTYPE_REAL64:
        default:
            throw std::runtime_error("Unsupported CO_DEFTYPE: " + std::to_string(co_def));
    }
}

/**
 * @brief CO_DEFTYPE에 해당하는 타입의 크기(바이트) 반환
 */
inline constexpr size_t getCoDefTypeSize(uint8_t co_def) {
    switch (co_def) {
        case CO_DEFTYPE_UNSIGNED8:
        case CO_DEFTYPE_INTEGER8:
        case CO_DEFTYPE_BOOLEAN:
            return 1;
        case CO_DEFTYPE_UNSIGNED16:
        case CO_DEFTYPE_INTEGER16:
            return 2;
        case CO_DEFTYPE_UNSIGNED32:
        case CO_DEFTYPE_INTEGER32:
        case CO_DEFTYPE_REAL32:
            return 4;
        case CO_DEFTYPE_UNSIGNED64:
        case CO_DEFTYPE_INTEGER64:
        case CO_DEFTYPE_REAL64:
            return 8;
        default:
            return 0;
    }
}

/**
 * @brief CO_DEFTYPE이 부호 있는 타입인지 확인
 */
inline constexpr bool isCoDefTypeSigned(uint8_t co_def) {
    switch (co_def) {
        case CO_DEFTYPE_INTEGER8:
        case CO_DEFTYPE_INTEGER16:
        case CO_DEFTYPE_INTEGER32:
        case CO_DEFTYPE_INTEGER64:
        case CO_DEFTYPE_REAL32:
        case CO_DEFTYPE_REAL64:
            return true;
        default:
            return false;
    }
}

/**
 * @brief CO_DEFTYPE의 이름을 문자열로 반환 (디버깅용)
 */
inline const char* getCoDefTypeName(uint8_t co_def) {
    switch (co_def) {
        case CO_DEFTYPE_BOOLEAN: return "BOOLEAN";
        case CO_DEFTYPE_UNSIGNED8: return "UNSIGNED8";
        case CO_DEFTYPE_INTEGER8: return "INTEGER8";
        case CO_DEFTYPE_UNSIGNED16: return "UNSIGNED16";
        case CO_DEFTYPE_INTEGER16: return "INTEGER16";
        case CO_DEFTYPE_UNSIGNED32: return "UNSIGNED32";
        case CO_DEFTYPE_INTEGER32: return "INTEGER32";
        case CO_DEFTYPE_UNSIGNED64: return "UNSIGNED64";
        case CO_DEFTYPE_INTEGER64: return "INTEGER64";
        case CO_DEFTYPE_REAL32: return "REAL32";
        case CO_DEFTYPE_REAL64: return "REAL64";
        default: return "UNKNOWN";
    }
}

}  // namespace Utils
}  // namespace NaviCAN
}  // namespace NaviFra

#endif  // NAVIFRA_NAVICAN_CANOPEN_TYPE_DISPATCHER_H
