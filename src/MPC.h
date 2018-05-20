#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

/**
 * State vector: [x, y, psi, v, cte, epsi]
 */
const unsigned num_states = 6;

/**
 * Actuators: [delta, a]
 */
const unsigned num_actuators = 2;

/**
 * Length from front to CoG
 */
const double Lf = 2.67;

/**
 * MPC horizon. tried 10, 20, 25
 */
const size_t N = 10;

/**
 * discrete interval to compute vehicle dynamics. tried 0,025, 0.05, 0.1, 0.2
 */
const double dt = 0.1;

/**
 * latency of the simulator
 */
const double latency = 0.1;

/**
 * reference velocity
 */
const double ref_v = 80;

class MPC {
 public:

  MPC();

  virtual ~MPC();

  /**
   * Solve the model given an initial state and polynomial coefficients.
   * Return control inputs and the x,y state
   */
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
