/*  
 * This project uses the Blaze library (https://bitbucket.org/blaze-lib/blaze)  
 * Copyright (c) Blaze development group  
 * Licensed under the New BSD License. See LICENSE_blaze for details.  
 */

#ifndef SPEED_OPTIMIZER_HPP_
#define SPEED_OPTIMIZER_HPP_

#include <vector>
#include <memory>
#include <blaze/Math.h>

class SpeedOptimizer
{
public:
  /**
   * @note type: unsigned char
   * @param SUCCESS, INACCURATE, OVER_TRY, OVER_TIME, ERROR
  */
  enum STATUS : unsigned char {
    SUCCESS = 0,
    INACCURATE = 1,
    OVER_TRY,
    OVER_TIME,
    ERROR
  };
  struct Weight_t {
    double v=100.0;
  };
  struct Param_t {
    size_t un_step_size = 0;
    unsigned int un_max_iter=100; // [N]
    int n_time_limit_msec=200; // [msec]
    double d_eps=1e-5;
    double d_alpha=0.5;
    double d_beta=0.25;

    // weight
    Weight_t st_w;
  };
  struct Result_t {
    std::shared_ptr< std::vector<double> > vecVptr;
    std::shared_ptr< std::vector<double> > vecAptr;
  };

  explicit SpeedOptimizer(size_t un_step_size);
  ~SpeedOptimizer() {};

  size_t GetStepSize() const { return st_param_.un_step_size; };

  static std::vector<double> CalcAfromV(double v_now, double ds, const std::vector<double>& vecV);

  void SetStepSize(size_t un_step_size);
  void SetParam(const Param_t& st_param);

  SpeedOptimizer::STATUS solve(double v_now, double a_now, double ds,
    const std::vector<double>& vecVr, const std::vector<double>& vecAr);
  SpeedOptimizer::STATUS solve(double v_now, double a_now, double ds,
    const std::vector<double>& vecVr) {
    if (vecVr.empty() || vecVr.size() == 1UL) return STATUS::ERROR;
    std::vector<double> vecV0(st_param_.un_step_size, v_now);
    std::vector<double> vecA0(st_param_.un_step_size, 0.0);
    std::vector<double> vecAr = CalcAfromV(v_now, ds, vecVr);
    return solve(v_now, a_now, ds, vecVr, vecAr);
  };

  Result_t result() const {
    return Result_t{std::make_shared< std::vector<double> >(std::vector<double>(V_.begin(),V_.end())), std::make_shared< std::vector<double> >(std::vector<double>(A_.begin(),A_.end()))};
  };

private:
  // param
  Param_t st_param_;

  // input data
  blaze::DynamicVector<double> Vr_;
  blaze::DynamicVector<double> Ar_;

  // solution-variable
  blaze::DynamicVector<double> V_;
  blaze::DynamicVector<double> A_;
  blaze::DynamicVector<double> X_;

  // preset memory
  blaze::DynamicMatrix<double> dV_dX;
  blaze::DynamicMatrix<double> dA_dX;
  blaze::DynamicMatrix<double> dV_dX_2;
  blaze::DynamicMatrix<double> dA_dX_2;
  blaze::DynamicMatrix<double> H_A;

  // Newton-variables
  blaze::DynamicVector<double> dJ_dX_;
  blaze::DynamicMatrix<double> inv_H_J_;

  // model-functions
  bool CalcDevModel(const size_t N, double ds, double v_now, double a_now, const Weight_t st_w,
    const blaze::DynamicVector<double>& Vr, const blaze::DynamicVector<double>& Ar,
    const blaze::DynamicVector<double>& V, const blaze::DynamicVector<double>& A, const blaze::DynamicVector<double>& X,
    blaze::DynamicVector<double>& dJ_dX, blaze::DynamicMatrix<double>& inv_H_J);
  double CalcJ(const size_t N, const Weight_t st_w,
    const blaze::DynamicVector<double>& Vr, const blaze::DynamicVector<double>& Ar,
    const blaze::DynamicVector<double>& V, const blaze::DynamicVector<double>& A);
  void CalcVAfromX(size_t N, double ds, double v_now, double a_now,
    const blaze::DynamicVector<double>& newX, blaze::DynamicVector<double>& V, blaze::DynamicVector<double>& A);
  // NewtonMethod-function
  double NewtonMethod(
    const size_t N, double alpha, double beta, double ds, double v_now, double a_now, const Weight_t st_w,
    const blaze::DynamicVector<double>& Vr, const blaze::DynamicVector<double>& Ar,
    const blaze::DynamicVector<double>& dJ_dX, const blaze::DynamicMatrix<double>& inv_H_J,
    blaze::DynamicVector<double>& V, blaze::DynamicVector<double>& A, blaze::DynamicVector<double>& X);
};

#endif
