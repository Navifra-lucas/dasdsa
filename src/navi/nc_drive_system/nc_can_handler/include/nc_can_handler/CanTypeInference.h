#pragma once

#include <linux/can.h>

#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

namespace NaviFra::CanHandler {

// 엔디안 변환 최적화
namespace endian {
template <typename T>
constexpr T from_little_endian(const uint8_t* data) noexcept
{
    if constexpr (sizeof(T) == 1) {
        return static_cast<T>(*data);
    }
    else if constexpr (sizeof(T) == 2) {
        return static_cast<T>(data[0] | (data[1] << 8));
    }
    else if constexpr (sizeof(T) == 4) {
        return static_cast<T>(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
    }
    else {
        T result = 0;
        for (size_t i = 0; i < sizeof(T); ++i) {
            result |= static_cast<T>(data[i]) << (i * 8);
        }
        return result;
    }
}

template <typename T>
constexpr T from_big_endian(const uint8_t* data) noexcept
{
    if constexpr (sizeof(T) == 1) {
        return static_cast<T>(*data);
    }
    else if constexpr (sizeof(T) == 2) {
        return static_cast<T>((data[0] << 8) | data[1]);
    }
    else if constexpr (sizeof(T) == 4) {
        return static_cast<T>((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
    }
    else {
        T result = 0;
        for (size_t i = 0; i < sizeof(T); ++i) {
            result = (result << 8) | data[i];
        }
        return result;
    }
}
}  // namespace endian

enum class EndianType : uint8_t
{
    LittleEndian,
    BigEndian
};

// 함수 특성 추론 최적화
template <typename R, typename... Args>
struct function_traits_impl {
    using args_tuple = std::tuple<Args...>;
    static constexpr size_t arity = sizeof...(Args);
};

template <typename R, typename... Args>
function_traits_impl<R, Args...> function_traits_helper(R (*)(Args...));

template <typename C, typename R, typename... Args>
function_traits_impl<R, Args...> function_traits_helper(R (C::*)(Args...) const);

template <typename C, typename R, typename... Args>
function_traits_impl<R, Args...> function_traits_helper(R (C::*)(Args...));

template <typename F>
auto function_traits_helper(F&& f) -> decltype(function_traits_helper(&std::decay_t<F>::operator()));

template <typename F>
using function_traits = decltype(function_traits_helper(std::declval<F>()));

template <typename F>
using function_args_tuple = typename function_traits<F>::args_tuple;

// 튜플 크기 계산
template <typename... Types>
constexpr size_t tuple_size_bytes_v = (sizeof(Types) + ...);

template <typename Tuple>
struct tuple_size_bytes;

template <typename... Types>
struct tuple_size_bytes<std::tuple<Types...>> {
    static constexpr size_t value = tuple_size_bytes_v<Types...>;
};

// 튜플 파서 최적화
template <typename Tuple, size_t Index = 0>
struct TupleParser {
    static void parse(Tuple& tuple, const uint8_t* data, size_t& offset, EndianType endian) noexcept
    {
        if constexpr (Index < std::tuple_size_v<Tuple>) {
            using ElementType = std::tuple_element_t<Index, Tuple>;

            std::get<Index>(tuple) = parseElement<ElementType>(data + offset, endian);
            offset += sizeof(ElementType);

            TupleParser<Tuple, Index + 1>::parse(tuple, data, offset, endian);
        }
    }

private:
    template <typename T>
    static constexpr T parseElement(const uint8_t* data, EndianType endian) noexcept
    {
        if constexpr (sizeof(T) == 1) {
            return static_cast<T>(*data);
        }
        else if constexpr (std::is_same_v<T, float>) {
            uint32_t raw_value =
                (endian == EndianType::BigEndian) ? endian::from_big_endian<uint32_t>(data) : endian::from_little_endian<uint32_t>(data);

            float result;
            std::memcpy(&result, &raw_value, sizeof(float));
            return result;
        }
        else {
            return (endian == EndianType::BigEndian) ? endian::from_big_endian<T>(data) : endian::from_little_endian<T>(data);
        }
    }
};

// 튜플 적용 최적화
template <typename F, typename Tuple, size_t... I>
constexpr void apply_tuple_impl(F&& f, Tuple&& t, std::index_sequence<I...>)
{
    f(std::get<I>(t)...);
}

template <typename F, typename Tuple>
constexpr void apply_tuple(F&& f, Tuple&& t)
{
    apply_tuple_impl(std::forward<F>(f), std::forward<Tuple>(t), std::make_index_sequence<std::tuple_size_v<std::decay_t<Tuple>>>{});
}

// 자동 콜백 클래스 최적화
template <typename F>
class AutoCallback {
    F callback_;
    EndianType endian_type_;

    using ArgsTuple = function_args_tuple<F>;
    static constexpr size_t required_bytes = tuple_size_bytes<ArgsTuple>::value;

public:
    constexpr AutoCallback(F callback, EndianType endian = EndianType::LittleEndian) noexcept
        : callback_(std::move(callback))
        , endian_type_(endian)
    {
    }

    void operator()(const can_frame& frame)
    {
        assert(frame.can_dlc >= required_bytes && "CAN frame DLC insufficient for callback");

        ArgsTuple data_tuple;
        size_t offset = 0;
        TupleParser<ArgsTuple>::parse(data_tuple, frame.data, offset, endian_type_);
        apply_tuple(callback_, data_tuple);
    }

    static constexpr size_t getRequiredBytes() noexcept { return required_bytes; }

    std::string getTypeInfo() const { return generateTypeInfo<ArgsTuple>(std::make_index_sequence<std::tuple_size_v<ArgsTuple>>{}); }

private:
    template <typename Tuple, size_t... I>
    std::string generateTypeInfo(std::index_sequence<I...>) const
    {
        std::string result = "auto(";
        // ((result += getTypeName<std::tuple_element_t<I, Tuple>>() + (I < sizeof...(I) - 1 ? "," : "")), ...);
        ((result += std::string(getTypeName<std::tuple_element_t<I, Tuple>>()) + (I < sizeof...(I) - 1 ? "," : "")), ...);
        return result + ")";
    }

    template <typename T>
    static constexpr const char* getTypeName() noexcept
    {
        if constexpr (std::is_same_v<T, uint8_t>)
            return "u8";
        else if constexpr (std::is_same_v<T, uint16_t>)
            return "u16";
        else if constexpr (std::is_same_v<T, uint32_t>)
            return "u32";
        else if constexpr (std::is_same_v<T, int8_t>)
            return "i8";
        else if constexpr (std::is_same_v<T, int16_t>)
            return "i16";
        else if constexpr (std::is_same_v<T, int32_t>)
            return "i32";
        else if constexpr (std::is_same_v<T, float>)
            return "f32";
        else
            return "?";
    }
};

// 베이스 클래스
class CallbackWrapperBase {
public:
    virtual ~CallbackWrapperBase() = default;
    virtual void operator()(const can_frame& frame) = 0;
    virtual std::string getTypeInfo() const = 0;
    virtual size_t getRequiredBytes() const noexcept = 0;
};

// 래퍼 클래스
template <typename F>
class AutoCallbackWrapper final : public CallbackWrapperBase {
    AutoCallback<F> callback_;

public:
    constexpr AutoCallbackWrapper(F callback, EndianType endian = EndianType::LittleEndian) noexcept
        : callback_(std::move(callback), endian)
    {
    }

    void operator()(const can_frame& frame) override { callback_(frame); }
    std::string getTypeInfo() const override { return callback_.getTypeInfo(); }
    size_t getRequiredBytes() const noexcept override { return AutoCallback<F>::getRequiredBytes(); }
};

}  // namespace NaviFra::CanHandler