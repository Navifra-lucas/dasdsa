#ifndef MAPINFO_HPP_
#define MAPINFO_HPP_

#include <stdint.h>
#include <vector>

namespace NaviFra
{

struct MapInfo_t
{
  int n_size_x;
  int n_size_y;
  float f_size_x_m;
  float f_size_y_m;
  float f_resolution_m;
  float f_origin_x_m;
  float f_origin_y_m;

};

// void SetMapOrigin(float f_origin_x_m, int f_origin_y_m) {
//       f_origin_x_m_=f_origin_x_m;
//       f_origin_y_m_=f_origin_y_m;
//     };

}


#endif