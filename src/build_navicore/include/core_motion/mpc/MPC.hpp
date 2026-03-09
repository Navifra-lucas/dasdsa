#ifndef MPC_H
#define MPC_H

#include <eigen3/Eigen/Core>
#include <map>
#include <mutex>
#include <vector>

using namespace std;
namespace NaviFra {

struct MPCparam_t {
    double DT;
    double STEPS;
    double REF_CTE;
    double REF_ETHETA;
    double REF_V;
    double W_CTE;
    double W_EPSI;
    double W_V;
    double W_ANGVEL;
    double W_A;
    double W_DANGVEL;
    double W_DA;
    double ANGVEL;
    double MAXTHR;
    double BOUND;
};

class MPC {
public:
    MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
    vector<double> mpc_x;
    vector<double> mpc_y;
    vector<double> mpc_theta;

    double _mpc_totalcost = 0;
    double _mpc_ctecost = 0;
    double _mpc_ethetacost = 0;
    double _mpc_velcost = 0;
    std::mutex mtx_param_mpc_;

    void LoadParams(const MPCparam_t& params);
    void SetRefVel(const double f_val) { _params.REF_V = f_val; }
    void SetHighWCTE(const double f_val) { _params.W_CTE = f_val; }

private:
    // Parameters for mpc solver
    double _max_angvel, _max_throttle, _bound_value;
    int _mpc_steps, _x_start, _y_start, _theta_start, _v_start, _cte_start, _etheta_start, _angvel_start, _a_start;
    MPCparam_t _params;
    unsigned int dis_cnt = 0;
};
}  // namespace NaviFra
#endif /* MPC_H */
