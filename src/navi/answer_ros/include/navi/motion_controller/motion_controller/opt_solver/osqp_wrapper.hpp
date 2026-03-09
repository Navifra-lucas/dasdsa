/*  
 * This project uses the OSQP library (https://osqp.org)  
 * Copyright (c) OSQP development group  
 * Licensed under the Apache-2.0 License. See LICENSE_osqp for details.  
 */

#ifndef OSQP_WRAPPER_HPP_
#define OSQP_WRAPPER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <utility>

#include <osqp.h>

typedef std::vector<OSQPInt> OSQPVecInt;
typedef std::vector<OSQPFloat> OSQPVecFloat;

namespace OsqpWrapper {

struct SparseMatrixElement {
  OSQPInt r, c;
  OSQPFloat v;

  inline bool operator<(const SparseMatrixElement &rhs) {
    return (c == rhs.c) ? (r < rhs.r) : (c < rhs.c);
  }
}; // struct SparseMatrixElement

class SparseMatrix {
private:
  OSQPInt n_;
  OSQPInt m_;

  std::vector<SparseMatrixElement> elements_;

  std::vector<OSQPFloat> osqp_csc_data_;
  std::vector<OSQPInt> osqp_csc_row_idx_;
  std::vector<OSQPInt> osqp_csc_col_start_;
  OSQPCscMatrix *osqp_csc_instance = nullptr;

  void freeOSQPCSCInstance();

public:
  SparseMatrix();
  ~SparseMatrix();

  void initialize(OSQPInt m, OSQPInt n);
  void addElement(OSQPInt r, OSQPInt c, OSQPFloat v);
  OSQPCscMatrix *toOSQPCSC();

  std::string toStr(char* name) const;
}; // class SparseMatrix

class QPProblem {
private:
  OSQPSettings  *osqp_settings_= nullptr;
  OSQPSolver *osqp_solver_ = nullptr;

public:
  //number of variables and constraints
  OSQPInt n_;
  OSQPInt m_;

  //cost function
  SparseMatrix P_;
  OSQPVecFloat q_;

  //constraints
  SparseMatrix A_;
  OSQPVecFloat l_, u_;

  QPProblem();
  ~QPProblem();
  void initialize(OSQPInt n, OSQPInt m);
  OSQPInt solve();

  OSQPInt get_rows() { return m_; };
  OSQPInt get_cols() { return n_; };
  OSQPSolution* get_solution() const;

  std::string toStr(char* name, const OSQPVecFloat& vec) const;
  std::string toStr() const;
}; // class QPProblem

} // namespace MpcWrapper

#endif
