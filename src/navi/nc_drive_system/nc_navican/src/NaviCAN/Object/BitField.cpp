#include "NaviCAN/Object/BitField.hpp"

namespace NaviFra {
namespace NaviCAN {
namespace Object {

// 자주 사용되는 타입에 대한 명시적 인스턴스화
template class BitField<uint8_t>;
template class BitField<uint16_t>;
template class BitField<uint32_t>;
template class BitField<uint64_t>;
template class BitField<int8_t>;
template class BitField<int16_t>;
template class BitField<int32_t>;
template class BitField<int64_t>;

// 전역 연산자에 대한 명시적 인스턴스화
template BitField<uint8_t> operator|(uint8_t, const BitField<uint8_t>&);
template BitField<uint16_t> operator|(uint16_t, const BitField<uint16_t>&);
template BitField<uint32_t> operator|(uint32_t, const BitField<uint32_t>&);
template BitField<uint64_t> operator|(uint64_t, const BitField<uint64_t>&);
template BitField<int8_t> operator|(int8_t, const BitField<int8_t>&);
template BitField<int16_t> operator|(int16_t, const BitField<int16_t>&);
template BitField<int32_t> operator|(int32_t, const BitField<int32_t>&);
template BitField<int64_t> operator|(int64_t, const BitField<int64_t>&);

template BitField<uint8_t> operator&(uint8_t, const BitField<uint8_t>&);
template BitField<uint16_t> operator&(uint16_t, const BitField<uint16_t>&);
template BitField<uint32_t> operator&(uint32_t, const BitField<uint32_t>&);
template BitField<uint64_t> operator&(uint64_t, const BitField<uint64_t>&);
template BitField<int8_t> operator&(int8_t, const BitField<int8_t>&);
template BitField<int16_t> operator&(int16_t, const BitField<int16_t>&);
template BitField<int32_t> operator&(int32_t, const BitField<int32_t>&);
template BitField<int64_t> operator&(int64_t, const BitField<int64_t>&);

template BitField<uint8_t> operator^(uint8_t, const BitField<uint8_t>&);
template BitField<uint16_t> operator^(uint16_t, const BitField<uint16_t>&);
template BitField<uint32_t> operator^(uint32_t, const BitField<uint32_t>&);
template BitField<uint64_t> operator^(uint64_t, const BitField<uint64_t>&);
template BitField<int8_t> operator^(int8_t, const BitField<int8_t>&);
template BitField<int16_t> operator^(int16_t, const BitField<int16_t>&);
template BitField<int32_t> operator^(int32_t, const BitField<int32_t>&);
template BitField<int64_t> operator^(int64_t, const BitField<int64_t>&);

} // namespace Object
} // namespace NaviCAN
} // namespace NaviFra