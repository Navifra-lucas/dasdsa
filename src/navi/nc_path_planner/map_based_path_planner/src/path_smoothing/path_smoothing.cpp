#include "path_smoothing.h"

#include "./matrix/MatrixAlgebra.h"

namespace NaviFra {
PathSmoothing::PathSmoothing()
    : ptr_map_(nullptr)
{
}
PathSmoothing::~PathSmoothing()
{
}
void PathSmoothing::SetMap(std::shared_ptr<Map> o_map)
{
    ptr_map_ = std::make_shared<MapInfo2>();
    ptr_map_->n_x_pixel = o_map->GetXpx();
    ptr_map_->n_y_pixel = o_map->GetYpx();
    ptr_map_->n_map_size = o_map->GetXpx() * o_map->GetYpx();
    ptr_map_->vec_map = std::make_shared<std::vector<int8_t>>(o_map->GetMap());
    ptr_map_->f_resolution_m = o_map->getresolutionM();
}

void PathSmoothing::SetParam(float f_pos_dist, int WAY_POINT_INTERVAL_cnt)
{
    f_pos_dist_ = f_pos_dist;
    WAY_POINT_INTERVAL = WAY_POINT_INTERVAL_cnt;
}

vector<NaviFra::Pos> PathSmoothing::smoothingPath(vector<NaviFra::Pos> originalPathList)
{
    vector<NaviFra::Pos> WayPointList = generateWaypointFromPath(originalPathList, WAY_POINT_INTERVAL);
    vector<NaviFra::Pos> WayPointListAfterLeastSquare = doLeastSquare(WayPointList);
    vector<NaviFra::Pos> PathListAfterSmoothing = doCubicSplineInterpolation(f_pos_dist_, WayPointListAfterLeastSquare);
    return PathListAfterSmoothing;
}

vector<NaviFra::Pos> PathSmoothing::smoothingCubicSplinePath(vector<NaviFra::Pos> originalPathList)
{
    vector<NaviFra::Pos> WayPointList = generateWaypointFromPath(originalPathList, WAY_POINT_INTERVAL);
    vector<NaviFra::Pos> PathListAfterSmoothing = doCubicSplineInterpolation(f_pos_dist_, WayPointList);

    return PathListAfterSmoothing;
}

vector<NaviFra::Pos> PathSmoothing::generateWaypointFromPath(vector<NaviFra::Pos> originalPathList, int ninterval)
{
    vector<NaviFra::Pos> WayPointList;
    WayPointList.push_back(originalPathList.front());
    vector<NaviFra::Pos>::iterator it;
    int i = 0;
    for (it = originalPathList.begin(); it != originalPathList.end(); it++) {
        if (i % ninterval == 0)
            WayPointList.push_back(*it);
        i++;
    }
    WayPointList.push_back(originalPathList.back());
    return WayPointList;
}

vector<NaviFra::Pos> PathSmoothing::getWayPointList()
{
    return m_WayPointList;
}

vector<NaviFra::Pos> PathSmoothing::getInflectionWayPointList()
{
    return m_InflectionWayPointList;
}

vector<NaviFra::Pos> PathSmoothing::doLeastSquare(vector<NaviFra::Pos> WayPointList)
{
    vector<NaviFra::Pos> newWayPointList;
    vector<NaviFra::Pos>::iterator itFirstWayPoint;
    vector<NaviFra::Pos>::iterator itSecondWayPoint;
    vector<NaviFra::Pos>::iterator itThirdWayPoint;
    const int ntotalwaypointnum = 3;

    NaviFra::Pos WayPoint;
    NaviFra::Pos updatedWayPoint;

    itFirstWayPoint = itSecondWayPoint = itThirdWayPoint = WayPointList.begin();
    itSecondWayPoint++;
    itThirdWayPoint++;
    itThirdWayPoint++;
    double dWayPointX[ntotalwaypointnum];
    double dWayPointY[ntotalwaypointnum];
    double da = 0;
    double db = 0;
    double dSigmaX2 = 0;
    double dSigmaX = 0;
    double dSigmaY = 0;
    double dSigmaXY = 0;
    double dmidPointX = 0;
    double dmidPointY = 0;
    int i = 0;
    bool binitposflag = true;
    newWayPointList.push_back(WayPointList.front());

    for (; itThirdWayPoint != WayPointList.end(); itFirstWayPoint++, itSecondWayPoint++, itThirdWayPoint++) {
        dSigmaX2 = 0;
        dSigmaX = 0;
        dSigmaY = 0;
        dSigmaXY = 0;
        if (binitposflag) {
            binitposflag = false;
            dWayPointX[0] = itFirstWayPoint->GetXm() * 1000;
            dWayPointY[0] = itFirstWayPoint->GetYm() * 1000;
        }
        else {
            updatedWayPoint = newWayPointList.back();
            dWayPointX[0] = updatedWayPoint.GetXm() * 1000;
            dWayPointY[0] = updatedWayPoint.GetYm() * 1000;
        }
        dWayPointX[1] = itSecondWayPoint->GetXm() * 1000;
        dWayPointY[1] = itSecondWayPoint->GetYm() * 1000;
        dWayPointX[2] = itThirdWayPoint->GetXm() * 1000;
        dWayPointY[2] = itThirdWayPoint->GetYm() * 1000;
        for (i = 0; i < ntotalwaypointnum; i++) {
            dSigmaX2 += powf(dWayPointX[i], 2);
        }
        for (i = 0; i < ntotalwaypointnum; i++) {
            dSigmaX += dWayPointX[i];
        }
        for (i = 0; i < ntotalwaypointnum; i++) {
            dSigmaY += dWayPointY[i];
        }
        for (i = 0; i < ntotalwaypointnum; i++) {
            dSigmaXY += (dWayPointX[i] * dWayPointY[i]);
        }

        if ((3 * dSigmaX2 - dSigmaX * dSigmaX) == 0 || dSigmaX * dSigmaX - 3 * dSigmaX == 0
            /*||dWayPointX[1]==dWayPointX[2]||dWayPointY[1]==dWayPointY[2*]*/) {
            WayPoint.SetXm(dWayPointX[1] / 1000.0);
            WayPoint.SetYm(dWayPointY[1] / 1000.0);
            newWayPointList.push_back(WayPoint);
            continue;
        }
        da = (ntotalwaypointnum * dSigmaXY - dSigmaX * dSigmaY) / (ntotalwaypointnum * dSigmaX2 - dSigmaX * dSigmaX);
        db = (dSigmaXY * dSigmaX - dSigmaX2 * dSigmaY) / (dSigmaX * dSigmaX - 3 * dSigmaX2);

        dmidPointX = (-db * da + (dWayPointX[1] + da * dWayPointY[1])) / (da * da + 1);
        dmidPointY = ((da * dWayPointX[1] + da * da * dWayPointY[1]) + db) / (da * da + 1);

        if (Obstacleflag((int)dmidPointX / 1000.0, (int)dmidPointY / 1000.0) == true) {
            WayPoint.SetXm(dWayPointX[1] / 1000.0);
            WayPoint.SetYm(dWayPointY[1] / 1000.0);
        }
        else {
            WayPoint.SetXm((int)dmidPointX / 1000.0);
            WayPoint.SetYm((int)dmidPointY / 1000.0);
        }

        newWayPointList.push_back(WayPoint);
    }
    newWayPointList.push_back(WayPointList.back());

    return newWayPointList;
}

bool PathSmoothing::Obstacleflag(float f_midPointX, float f_midPointY)
{
    if (ptr_map_ == nullptr)
        return false;
    int x;
    int y;
    // char c_tmp_val;
    int8_t n8_tmp_val;

    float f_map_resolution = ptr_map_->f_resolution_m;  // cm per pixel.
    float robot_size_x = 1.2;
    float robot_size_y = 1.2;
    int barrier_x = (robot_size_x / f_map_resolution) / 2;
    int barrier_y = (robot_size_y / f_map_resolution) / 2;
    int n_point_x = f_midPointX / f_map_resolution;
    int n_point_y = f_midPointY / f_map_resolution;

    for (int i = -barrier_x; i < barrier_x; i += 1) {
        for (int j = -barrier_y; j < barrier_y; j += 1) {
            x = n_point_x + i;
            y = n_point_y + j;
            if (x < 0 || x >= ptr_map_->n_x_pixel || y < 0 || y >= ptr_map_->n_y_pixel) {
                return true;
            }
            // c_tmp_val = ptr_map_->vec_map->at(x + y * ptr_map_->n_x_pixel);
            n8_tmp_val = ptr_map_->vec_map->at(x + y * ptr_map_->n_x_pixel);
            if (n8_tmp_val >= (100)) {
                return true;
            }

            // if (c_tmp_val >= static_cast<char>(100)) {
            //     return true;
            // }
        }
    }
    return false;
}

vector<NaviFra::Pos> PathSmoothing::doCubicSplineInterpolation(double dinterval, vector<NaviFra::Pos> stlWayPoint)
{
    vector<NaviFra::Pos> WayPointlist = stlWayPoint;

    if (dinterval < 0.001)
        dinterval = 0.001;
    if (WayPointlist.size() < 2)
        return stlWayPoint;

    unsigned int nWayPonitVectosize = WayPointlist.size() - 1;

    vector<NaviFra::Pos>::iterator itPastWayPoint;
    vector<NaviFra::Pos>::iterator itCurrentWayPoint;
    vector<NaviFra::Pos>::iterator itAftertWayPoint;

    double dTemp[3] = {0.0, 0.0, 0.0};
    vector<NaviFra::Pos>::iterator itwaypoint;
    for (itwaypoint = WayPointlist.begin(); itwaypoint != WayPointlist.end(); itwaypoint++) {
        itwaypoint->SetZm(
            sqrt(
                (dTemp[0] - itwaypoint->GetXm()) * (dTemp[0] - itwaypoint->GetXm()) +
                (dTemp[1] - itwaypoint->GetYm()) * (dTemp[1] - itwaypoint->GetYm())) +
            dTemp[2]);
        dTemp[0] = itwaypoint->GetXm();
        dTemp[1] = itwaypoint->GetYm();
        dTemp[2] = itwaypoint->GetZm();
    }
    //========================================================================================================

    itCurrentWayPoint = itAftertWayPoint = WayPointlist.begin();
    itAftertWayPoint++;

    vector<double> WayPonitDistanceGap(nWayPonitVectosize);

    for (int i = 0; itAftertWayPoint != WayPointlist.end(); itCurrentWayPoint++, itAftertWayPoint++, i++) {
        WayPonitDistanceGap[i] = itAftertWayPoint->GetZm() - itCurrentWayPoint->GetZm();
        if (WayPonitDistanceGap[i] < dinterval)
            WayPonitDistanceGap[i] = dinterval;
    }  // WayPonitDistanceGap

    // dMatrix A
    dMatrix dMatrixA(nWayPonitVectosize + 1, nWayPonitVectosize + 1);
    dMatrixA.null();
    {
        unsigned int i = 0;
        dMatrixA(0, 0) = 2 * WayPonitDistanceGap[i];
        dMatrixA(0, 1) = WayPonitDistanceGap[i];
        for (i = 1; i < nWayPonitVectosize; ++i) {
            dMatrixA(i, i - 1) = WayPonitDistanceGap[i - 1];
            dMatrixA(i, i + 0) = 2 * (WayPonitDistanceGap[i - 1] + WayPonitDistanceGap[i]);
            dMatrixA(i, i + 1) = WayPonitDistanceGap[i];
        }
        dMatrixA(i, i - 1) = WayPonitDistanceGap[i - 1];
        dMatrixA(i, i) = 2. * WayPonitDistanceGap[i - 1];
    }
    //======================================================================================================================

    // dMatrix b
    int mcolum = 3;
    itPastWayPoint = itCurrentWayPoint = itAftertWayPoint = WayPointlist.begin();
    itCurrentWayPoint++;
    itAftertWayPoint++;
    itAftertWayPoint++;
    dMatrix dMatrixb(nWayPonitVectosize + 1, mcolum);
    {
        unsigned int i = 0;

        dMatrixb(i, 0) = 6. * (itCurrentWayPoint->GetZm() - itPastWayPoint->GetZm()) / WayPonitDistanceGap[i];
        dMatrixb(i, 1) = 6. * (itCurrentWayPoint->GetXm() - itPastWayPoint->GetXm()) / WayPonitDistanceGap[i];
        dMatrixb(i, 2) = 6. * (itCurrentWayPoint->GetYm() - itPastWayPoint->GetYm()) / WayPonitDistanceGap[i];

        for (i = 1; itAftertWayPoint != WayPointlist.end(); itPastWayPoint++, itCurrentWayPoint++, itAftertWayPoint++, i++) {
            dMatrixb(i, 0) = 6. *
                ((itAftertWayPoint->GetZm() - itCurrentWayPoint->GetZm()) / WayPonitDistanceGap[i] -
                 (itCurrentWayPoint->GetZm() - itPastWayPoint->GetZm()) / WayPonitDistanceGap[i - 1]);
            dMatrixb(i, 1) = 6. *
                ((itAftertWayPoint->GetXm() - itCurrentWayPoint->GetXm()) / WayPonitDistanceGap[i] -
                 (itCurrentWayPoint->GetXm() - itPastWayPoint->GetXm()) / WayPonitDistanceGap[i - 1]);
            dMatrixb(i, 2) = 6. *
                ((itAftertWayPoint->GetYm() - itCurrentWayPoint->GetYm()) / WayPonitDistanceGap[i] -
                 (itCurrentWayPoint->GetYm() - itPastWayPoint->GetYm()) / WayPonitDistanceGap[i - 1]);
        }
        dMatrixb(i, 0) = 6. * (-(itCurrentWayPoint->GetZm() - itPastWayPoint->GetZm())) / WayPonitDistanceGap[i - 1];
        dMatrixb(i, 1) = 6. * (-(itCurrentWayPoint->GetXm() - itPastWayPoint->GetXm())) / WayPonitDistanceGap[i - 1];
        dMatrixb(i, 2) = 6. * (-(itCurrentWayPoint->GetYm() - itPastWayPoint->GetYm())) / WayPonitDistanceGap[i - 1];
    }
    //======================================================================================================================

    dMatrix dMatrixz = !dMatrixA * dMatrixb;  // dMatrix z ���

    // PathList
    vector<NaviFra::Pos> PathList;

    itCurrentWayPoint = itAftertWayPoint = WayPointlist.begin();
    itAftertWayPoint++;
    bool b_initial = true;
    double realPathPoint = itCurrentWayPoint->GetZm();
    for (int i = 0; itAftertWayPoint != WayPointlist.end(); itCurrentWayPoint++, itAftertWayPoint++, i++) {
        for (; realPathPoint < itAftertWayPoint->GetZm(); realPathPoint += dinterval) {
            double dDistancePathnFeuturePoint = realPathPoint - itCurrentWayPoint->GetZm();
            double tdDistancePathnCurrentPointp = itAftertWayPoint->GetZm() - realPathPoint;

            double dDistancePathnFeuturePoint3 = powf(dDistancePathnFeuturePoint, 3);
            double tdDistancePathnCurrentPointp3 = powf(tdDistancePathnCurrentPointp, 3);

            NaviFra::Pos WayPoint = WayPointlist[i];
            WayPoint.SetZm(realPathPoint);

            WayPoint.SetXm(
                (dMatrixz(i + 1, 1) * dDistancePathnFeuturePoint3 + dMatrixz(i, 1) * tdDistancePathnCurrentPointp3) /
                    (6. * WayPonitDistanceGap[i]) +
                (itAftertWayPoint->GetXm() / WayPonitDistanceGap[i] - WayPonitDistanceGap[i] * dMatrixz(i + 1, 1) / 6.) *
                    dDistancePathnFeuturePoint +
                (itCurrentWayPoint->GetXm() / WayPonitDistanceGap[i] - WayPonitDistanceGap[i] * dMatrixz(i, 1) / 6.) *
                    tdDistancePathnCurrentPointp);
            WayPoint.SetYm(
                (dMatrixz(i + 1, 2) * dDistancePathnFeuturePoint3 + dMatrixz(i, 2) * tdDistancePathnCurrentPointp3) /
                    (6. * WayPonitDistanceGap[i]) +
                (itAftertWayPoint->GetYm() / WayPonitDistanceGap[i] - WayPonitDistanceGap[i] * dMatrixz(i + 1, 2) / 6.) *
                    dDistancePathnFeuturePoint +
                (itCurrentWayPoint->GetYm() / WayPonitDistanceGap[i] - WayPonitDistanceGap[i] * dMatrixz(i, 2) / 6.) *
                    tdDistancePathnCurrentPointp);
            if (b_initial == false)
                PathList.push_back(WayPoint);
            b_initial = false;
        }
    }
    if (PathList.size() > 2)
        PathList.resize(PathList.size() - 2);
    // if(PathList.size()>2){ PathList.pop_front(); PathList.pop_back();}
    // else{PathList.push_back(stlWayPoint.front());}
    //======================================================================================================================
    // LOG_INFO("the end maybe......\n");
    return PathList;
}

}  // namespace NaviFra
