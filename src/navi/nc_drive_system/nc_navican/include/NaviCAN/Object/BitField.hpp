#ifndef NAVICAN_NAVICAN_OBJECT_BITFIELD_HPP
#define NAVICAN_NAVICAN_OBJECT_BITFIELD_HPP

#include <atomic>

namespace NaviFra {
namespace NaviCAN {
namespace Object {

template <typename T>
class BitField 
{
public:
    BitField();
    BitField(T value);

    // 복사 생성자와 대입 연산자 추가
    BitField(const BitField<T>& other);
    BitField<T>& operator=(const BitField<T>& other);
    
    // 기본 타입 T에 대한 대입 연산자 추가
    BitField<T>& operator=(T value);
    
    // 이동 생성자와 이동 대입 연산자 추가
    BitField(BitField<T>&& other) noexcept;
    BitField<T>& operator=(BitField<T>&& other) noexcept;

    T getValue() const;
    void setValue(T value);

    bool getBit(unsigned bit) const;
    void setBit(unsigned bit, bool value);

    T exchange(T desired);

    template <typename Func>
    void update(Func func);

    // 비트 연산자 오버로딩
    // 비트 OR 연산자
    BitField<T>& operator|=(const BitField<T>& other);
    BitField<T>& operator|=(T value);
    BitField<T> operator|(const BitField<T>& other) const;
    BitField<T> operator|(T value) const;

    // 비트 AND 연산자
    BitField<T>& operator&=(const BitField<T>& other);
    BitField<T>& operator&=(T value);
    BitField<T> operator&(const BitField<T>& other) const;
    BitField<T> operator&(T value) const;

    // 비트 XOR 연산자
    BitField<T>& operator^=(const BitField<T>& other);
    BitField<T>& operator^=(T value);
    BitField<T> operator^(const BitField<T>& other) const;
    BitField<T> operator^(T value) const;

    // 비트 NOT 연산자
    BitField<T> operator~() const;

    // 왼쪽 시프트 연산자
    BitField<T>& operator<<=(unsigned shift);
    BitField<T> operator<<(unsigned shift) const;

    // 오른쪽 시프트 연산자
    BitField<T>& operator>>=(unsigned shift);
    BitField<T> operator>>(unsigned shift) const;
private:
    std::atomic<T> value_;
};

// 전역 연산자 오버로딩 (T와 BitField 간의 연산을 위해)
template <typename T>
BitField<T> operator|(T lhs, const BitField<T>& rhs);

template <typename T>
BitField<T> operator&(T lhs, const BitField<T>& rhs);

template <typename T>
BitField<T> operator^(T lhs, const BitField<T>& rhs);

} // namespace Object
} // namespace NaviCAN
} // namespace NaviFra

// 중요: 템플릿 구현을 포함
#include "BitField.tpp"

#endif // NAVICAN_OBJECT_BITFIELD_HPP