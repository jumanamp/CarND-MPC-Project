#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;


class MPC {
 public:

   /*
   * Cost Function weight vector
   */
  std::vector<double> c_weights;
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  // Predict new state using Kinematic Motion model
  void ApplyMotionModel(double &x, double &y, double &psi, double &v, double a, double delta, double dt);

  /*
  * Initialize MPC Cost Function Weights.
  */
  void InitCostWeights(double c_cte, double c_epsi, double c_v, double c_delta, double c_a, double c_s_delta, double c_s_a);
};

#endif /* MPC_H */
