/**
* @class occupied space cost function
* @brief Creates a cost function for matching the 'point_cloud' to the 'grid' with
        a 'pose'. The cost increases with poorer correspondence of the grid and the
        point observation (e.g. points falling into less occupied space).
* @author logan (donghak lee)
* contact : donghak.lee@navifra.com
*/
#ifndef OCCUPIED_SPACE_COST_FUNCTION_2D_H_
#define OCCUPIED_SPACE_COST_FUNCTION_2D_H_

#include "ceres/ceres.h"
#include "common/scan2d.h"
#include "localmapper/localmap/grid2d.h"
#include "localmapper/localmap/localmap2d.h"
namespace NaviFra {
namespace SLAM2D {

ceres::CostFunction* CreateOccupiedSpaceCostFunction2D(const double scaling_factor, const Scan2D& point_cloud, const Localmap2D* grid);

}
}  // namespace NaviFra
#endif  // OCCUPIED_SPACE_COST_FUNCTION_2D_H_
