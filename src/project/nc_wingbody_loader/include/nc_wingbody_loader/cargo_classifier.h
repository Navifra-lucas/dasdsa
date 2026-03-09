#ifndef NC_WINGBODY_LOADER_CARGO_CLASSIFIER_H
#define NC_WINGBODY_LOADER_CARGO_CLASSIFIER_H

#include "nc_wingbody_loader/detected_plane.h"
#include <vector>

namespace nc_wingbody_loader {

class CargoClassifier {
public:
  CargoClassifier(double door_max_distance, double cargo_width, double cargo_depth,
                  double pallet_size);

  void classify(std::vector<DetectedPlane>& planes);

  const DetectedPlane* getDoor() const { return door_; }
  const DetectedPlane* getHeadWall() const { return head_wall_; }
  const DetectedPlane* getSideWallLeft() const { return side_wall_left_; }
  const DetectedPlane* getSideWallRight() const { return side_wall_right_; }
  const DetectedPlane* getFloor() const { return floor_; }
  const std::vector<const DetectedPlane*>& getExistingPallets() const { return existing_pallets_; }

private:
  double door_max_distance_;
  double cargo_width_;
  double cargo_depth_;
  double pallet_size_;

  const DetectedPlane* door_ = nullptr;
  const DetectedPlane* head_wall_ = nullptr;
  const DetectedPlane* side_wall_left_ = nullptr;
  const DetectedPlane* side_wall_right_ = nullptr;
  const DetectedPlane* floor_ = nullptr;
  std::vector<const DetectedPlane*> existing_pallets_;
};

} // namespace nc_wingbody_loader

#endif // NC_WINGBODY_LOADER_CARGO_CLASSIFIER_H
