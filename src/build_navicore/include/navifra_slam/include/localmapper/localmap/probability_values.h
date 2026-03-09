/**
 * @class probability values
 * @brief probability grid map information for updating prob. of localmap
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */
#ifndef PROBABILITY_VALUES_H
#define PROBABILITY_VALUES_H

#include <math.h>

#include <cmath>
#include <memory>
#include <vector>

#include "glog/logging.h"
#include "util/logger.hpp"

namespace NaviFra {
namespace SLAM2D {

constexpr float kMinProbability = 0.1f;
constexpr float kMaxProbability = 1.f - kMinProbability;
constexpr float kMinCorrespondenceCost = 1.f - kMaxProbability;
constexpr float kMaxCorrespondenceCost = 1.f - kMinProbability;
constexpr uint16_t kUnknownProbabilityValue = 0;
constexpr uint16_t kUnknownCorrespondenceValue = kUnknownProbabilityValue;
constexpr uint16_t kUpdateMarker = 1u << 15;
constexpr int kValueCount = 32768;
// Clamps 'value' to be in the range ['min', 'max'].
template <typename T>
T Clamp(const T value, const T min, const T max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}
class ProbabilityValues {
  ProbabilityValues();
  ~ProbabilityValues();

 public:
  static ProbabilityValues *GetInstance() {
    static ProbabilityValues s;
    return &s;
  }
  inline uint16_t BoundedFloatToValue(const float float_value,
                                      const float lower_bound,
                                      const float upper_bound) {
    const int value =
        std::lround(
            (Clamp(float_value, lower_bound, upper_bound) - lower_bound) *
            (32766.f / (upper_bound - lower_bound))) +
        1;
    // DCHECK for performance.
    // DCHECK_GE(value, 1);
    // DCHECK_LE(value, 32767);
    return value;
  }

  inline float Odds(float probability) {
    return probability / (1.f - probability);
  }

  inline float ProbabilityFromOdds(const float odds) {
    return odds / (odds + 1.f);
  }

  inline float ProbabilityToCorrespondenceCost(const float probability) {
    return 1.f - probability;
  }

  inline float CorrespondenceCostToProbability(
      const float correspondence_cost) {
    return 1.f - correspondence_cost;
  }

  // Clamps probability to be in the range [kMinProbability, kMaxProbability].
  inline float ClampProbability(const float probability) {
    return Clamp(probability, kMinProbability, kMaxProbability);
  }
  // Clamps correspondece cost to be in the range [kMinCorrespondenceCost,
  // kMaxCorrespondenceCost].
  inline float ClampCorrespondenceCost(const float correspondence_cost) {
    return Clamp(correspondence_cost, kMinCorrespondenceCost,
                 kMaxCorrespondenceCost);
  }

  // Converts a correspondence_cost to a uint16_t in the [1, 32767] range.
  inline uint16_t CorrespondenceCostToValue(const float correspondence_cost) {
    return BoundedFloatToValue(correspondence_cost, kMinCorrespondenceCost,
                               kMaxCorrespondenceCost);
  }

  // Converts a probability to a uint16_t in the [1, 32767] range.
  inline uint16_t ProbabilityToValue(const float probability) {
    return BoundedFloatToValue(probability, kMinProbability, kMaxProbability);
  }

  // Converts a uint16_t (which may or may not have the update marker set) to a
  // probability in the range [kMinProbability, kMaxProbability].
  inline float ValueToProbability(const uint16_t value) {
    return (*kValueToProbability)[value];
  }

  // Converts a uint16_t (which may or may not have the update marker set) to a
  // correspondence cost in the range [kMinCorrespondenceCost,
  // kMaxCorrespondenceCost].
  inline float ValueToCorrespondenceCost(const uint16_t value) {
    return (*kValueToCorrespondenceCost)[value];
  }

  inline uint16_t ProbabilityValueToCorrespondenceCostValue(
      uint16_t probability_value) {
    if (probability_value == kUnknownProbabilityValue) {
      return kUnknownCorrespondenceValue;
    }
    bool update_carry = false;
    if (probability_value > kUpdateMarker) {
      probability_value -= kUpdateMarker;
      update_carry = true;
    }
    uint16_t result = CorrespondenceCostToValue(
        ProbabilityToCorrespondenceCost(ValueToProbability(probability_value)));
    if (update_carry) result += kUpdateMarker;
    return result;
  }

  inline uint16_t CorrespondenceCostValueToProbabilityValue(
      uint16_t correspondence_cost_value) {
    if (correspondence_cost_value == kUnknownCorrespondenceValue)
      return kUnknownProbabilityValue;
    bool update_carry = false;
    if (correspondence_cost_value > kUpdateMarker) {
      correspondence_cost_value -= kUpdateMarker;
      update_carry = true;
    }
    uint16_t result = ProbabilityToValue(CorrespondenceCostToProbability(
        ValueToCorrespondenceCost(correspondence_cost_value)));
    if (update_carry) result += kUpdateMarker;
    return result;
  }

  std::unique_ptr<std::vector<float>> PrecomputeValueToProbability();
  std::unique_ptr<std::vector<float>> PrecomputeValueToCorrespondenceCost();
  std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(
      const uint16_t unknown_value, const float unknown_result,
      const float lower_bound, const float upper_bound);
  float SlowValueToBoundedFloat(const uint16_t value,
                                const uint16_t unknown_value,
                                const float unknown_result,
                                const float lower_bound,
                                const float upper_bound);
  std::vector<uint16_t> ComputeLookupTableToApplyOdds(float odds);
  std::vector<uint16_t> ComputeLookupTableToApplyCorrespondenceCostOdds(
      float odds);

  std::vector<float> *kValueToProbability;
  std::vector<float> *kValueToCorrespondenceCost;
};
}  // namespace SLAM2D
}  // namespace NaviFra

#endif  // CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
