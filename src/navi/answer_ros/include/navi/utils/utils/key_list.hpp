
/*
 * @file	: key_list.hpp
 * @date	: Jan. 22, 2025
 * @author	: "HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: key list for PublishCb or DataStorage
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef KEY_LIST_HPP_
#define KEY_LIST_HPP_

namespace NVFR
{

/*
0xABXX

 >> A (Communication Type)
  0 : PublishCb
  1 : DataStorage

 >> B (Package Type)
  0 : Mission
  1 : Path
  2 : Motion
  3 : Collision
  4 : Explore

 >> XX (Key Number)
*/
namespace KEY
{

// [Mission]
// PublishCb
const int NS = 0x0001;
const int NE = 0x0010;
const int NI = 0x0011;
// DataStorage
const int GN = 0x1001;

// [Path]
// PublishCb
//
// DataStorage
//

// [Motion]
// PublishCb
const int MC = 0x0201;
const int TF = 0x02F1;
// DataStorage
const int RC = 0x1201;
const int RP = 0x1202;
const int RV = 0x1203;
const int SP = 0x1204;

// [Collision]
// PublishCb
//
// DataStorage
//

// [Explore]
// PublishCb
const int XT = 0x0400;
const int XD = 0x0401;
const int XS = 0x0402;
const int XP = 0x0403;
const int XE = 0x0404;
const int XR = 0x0405;
const int XM = 0x0406;

}

}

#endif
