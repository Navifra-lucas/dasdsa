//  BitField.hpp에서 포함됩니다.
#ifndef NAVICAN_NAVICAN_OBJECT_BITFIELD_TPP
#define NAVICAN_NAVICAN_OBJECT_BITFIELD_TPP

namespace NaviFra {
namespace NaviCAN {
namespace Object {

template <typename T>
BitField<T>::BitField() : value_(0) {}

template <typename T>
BitField<T>::BitField(T value) : value_(value) {}

// 복사 생성자 구현
template <typename T>
BitField<T>::BitField(const BitField<T>& other) : value_(other.getValue()) {}

// 복사 대입 연산자 구현
template <typename T>
BitField<T>& BitField<T>::operator=(const BitField<T>& other) {
    if (this != &other) {
        setValue(other.getValue());
    }
    return *this;
}

// 기본 타입 T에 대한 대입 연산자 구현
template <typename T>
BitField<T>& BitField<T>::operator=(T value) {
    setValue(value);
    return *this;
}

// 이동 생성자 구현
template <typename T>
BitField<T>::BitField(BitField<T>&& other) noexcept : value_(other.exchange(0)) {}

// 이동 대입 연산자 구현
template <typename T>
BitField<T>& BitField<T>::operator=(BitField<T>&& other) noexcept {
    if (this != &other) {
        setValue(other.exchange(0));
    }
    return *this;
}

template <typename T>
T BitField<T>::getValue() const {
    return value_.load(std::memory_order_relaxed);
}

template <typename T>
void BitField<T>::setValue(T value) {
    value_.store(value, std::memory_order_relaxed);
}

template <typename T>
bool BitField<T>::getBit(unsigned bit) const {
    T mask = static_cast<T>(1) << bit;
    return (value_.load(std::memory_order_relaxed) & mask) != 0;
}

template <typename T>
void BitField<T>::setBit(unsigned bit, bool value) {
    T mask = static_cast<T>(1) << bit;
    if (value) {
        value_.fetch_or(mask, std::memory_order_relaxed);
    } else {
        value_.fetch_and(~mask, std::memory_order_relaxed);
    }
}

template <typename T>
T BitField<T>::exchange(T desired) {
    return value_.exchange(desired, std::memory_order_acq_rel);
}

template <typename T>
template <typename Func>
void BitField<T>::update(Func func) {
    T oldValue = value_.load(std::memory_order_relaxed);
    T newValue;
    do {
        newValue = func(oldValue);
    } while (!value_.compare_exchange_weak(
        oldValue, newValue, 
        std::memory_order_relaxed, 
        std::memory_order_relaxed));
}

// 비트 OR 연산자 구현
template <typename T>
BitField<T>& BitField<T>::operator|=(const BitField<T>& other) {
    update([&other](T current) {
        return current | other.getValue();
    });
    return *this;
}

template <typename T>
BitField<T>& BitField<T>::operator|=(T value) {
    update([value](T current) {
        return current | value;
    });
    return *this;
}

template <typename T>
BitField<T> BitField<T>::operator|(const BitField<T>& other) const {
    return BitField<T>(getValue() | other.getValue());
}

template <typename T>
BitField<T> BitField<T>::operator|(T value) const {
    return BitField<T>(getValue() | value);
}

// 비트 AND 연산자 구현
template <typename T>
BitField<T>& BitField<T>::operator&=(const BitField<T>& other) {
    update([&other](T current) {
        return current & other.getValue();
    });
    return *this;
}

template <typename T>
BitField<T>& BitField<T>::operator&=(T value) {
    update([value](T current) {
        return current & value;
    });
    return *this;
}

template <typename T>
BitField<T> BitField<T>::operator&(const BitField<T>& other) const {
    return BitField<T>(getValue() & other.getValue());
}

template <typename T>
BitField<T> BitField<T>::operator&(T value) const {
    return BitField<T>(getValue() & value);
}

// 비트 XOR 연산자 구현
template <typename T>
BitField<T>& BitField<T>::operator^=(const BitField<T>& other) {
    update([&other](T current) {
        return current ^ other.getValue();
    });
    return *this;
}

template <typename T>
BitField<T>& BitField<T>::operator^=(T value) {
    update([value](T current) {
        return current ^ value;
    });
    return *this;
}

template <typename T>
BitField<T> BitField<T>::operator^(const BitField<T>& other) const {
    return BitField<T>(getValue() ^ other.getValue());
}

template <typename T>
BitField<T> BitField<T>::operator^(T value) const {
    return BitField<T>(getValue() ^ value);
}

// 비트 NOT 연산자 구현
template <typename T>
BitField<T> BitField<T>::operator~() const {
    return BitField<T>(~getValue());
}

// 왼쪽 시프트 연산자 구현
template <typename T>
BitField<T>& BitField<T>::operator<<=(unsigned shift) {
    update([shift](T current) {
        return current << shift;
    });
    return *this;
}

template <typename T>
BitField<T> BitField<T>::operator<<(unsigned shift) const {
    return BitField<T>(getValue() << shift);
}

// 오른쪽 시프트 연산자 구현
template <typename T>
BitField<T>& BitField<T>::operator>>=(unsigned shift) {
    update([shift](T current) {
        return current >> shift;
    });
    return *this;
}

template <typename T>
BitField<T> BitField<T>::operator>>(unsigned shift) const {
    return BitField<T>(getValue() >> shift);
}

// 전역 연산자 오버로딩 구현
template <typename T>
BitField<T> operator|(T lhs, const BitField<T>& rhs) {
    return BitField<T>(lhs | rhs.getValue());
}

template <typename T>
BitField<T> operator&(T lhs, const BitField<T>& rhs) {
    return BitField<T>(lhs & rhs.getValue());
}

template <typename T>
BitField<T> operator^(T lhs, const BitField<T>& rhs) {
    return BitField<T>(lhs ^ rhs.getValue());
}

} // namespace Object
} // namespace NaviCAN
} // namespace NaviFra

#endif // NAVICAN_OBJECT_BITFIELD_TPP