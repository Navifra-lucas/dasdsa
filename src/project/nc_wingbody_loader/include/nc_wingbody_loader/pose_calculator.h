#ifndef NC_WINGBODY_LOADER_POSE_CALCULATOR_H
#define NC_WINGBODY_LOADER_POSE_CALCULATOR_H

#include "nc_wingbody_loader/detected_plane.h"
#include "nc_wingbody_loader/cargo_classifier.h"
#include "nc_wingbody_loader/transform_manager.h"
#include <string>

namespace nc_wingbody_loader {

class PoseCalculator {
public:
  PoseCalculator(double pallet_size, double safety_margin, double door_to_floor_offset,
                 double cargo_depth, TransformManager* transform_manager);

  PlacementResult calculate(const CargoClassifier& classifier,
                            const std::string& loading_mode);

private:
  double calculateYaw(const CargoClassifier& classifier);
  double calculateZ(const CargoClassifier& classifier);
  void calculateXY(const CargoClassifier& classifier,
                   const std::string& loading_mode,
                   double& x, double& y);

  double pallet_size_;
  double safety_margin_;
  double door_to_floor_offset_;
  double cargo_depth_;
  TransformManager* transform_manager_;
};

} // namespace nc_wingbody_loader

#endif // NC_WINGBODY_LOADER_POSE_CALCULATOR_H
