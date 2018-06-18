#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

size_t N = 10;
double dt = 0.2;

// Cost weights
double c_cte = 2000;
double c_epsi = 2000;
double c_v = 10;
double c_delta = 100;
double c_a = 5;
double c_s_delta = 50;
double c_s_a = 50;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Reference values we want our car to follow
double ref_v = 20;
double ref_cet = 0;
double ref_epsi = 0;

//Time step for every state
// mpc solver takes state variables and actuator variables in a single vector.
// So the starting point for every state varaible is determined here.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

// Cost function Evaluation Class
class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    //Cost function is set to zero in the beginning of every iteration
    fg[0] = 0;

    //Update Cost function

    // Reference State Cost
    // Define the cost related the reference state and
    // any anything you think may be beneficial.

    // cte, epsi and velocity control
    for (int i = 0; i < N; i++) {
      fg[0] += c_cte * CppAD::pow(vars[cte_start + i], 2); // cte, highest weight
      fg[0] += c_epsi * CppAD::pow(vars[epsi_start + i], 2); // epsi, highest weight

      // Cost function for maintaining velocity and avoid stopping.
      // Note reference velocity is added to form equation
      fg[0] += c_v * CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    // Initial acceleration and delta_t
    for (int i = 0; i < N - 1; i++) {
      fg[0] += c_delta * CppAD::pow(vars[delta_start + i], 2);
      fg[0] += c_a * CppAD::pow(vars[a_start + i], 2);
    }

    // Actuator inputs
    for (int i = 0; i < N - 2; i++) {
      fg[0] += c_s_delta * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += c_s_a * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }

    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
   for (int t = 1; t < N; t++) {
     // Values at time t+1
     AD<double> x1 = vars[x_start + t];
     AD<double> y1 = vars[y_start + t];
     AD<double> psi1 = vars[psi_start + t];
     AD<double> v1 = vars[v_start + t];
     AD<double> cte1 = vars[cte_start + t];
     AD<double> epsi1 = vars[epsi_start + t];

     // Values at time t
     AD<double> x0 = vars[x_start + t - 1];
     AD<double> y0 = vars[y_start + t - 1];
     AD<double> psi0 = vars[psi_start + t - 1];
     AD<double> v0 = vars[v_start + t - 1];
     AD<double> cte0 = vars[cte_start + t - 1];
     AD<double> epsi0 = vars[epsi_start + t - 1];

     // Actuator values at time t
     AD<double> a0 = vars[a_start + t - 1];
     AD<double> delta0 = vars[delta_start + t - 1];

     // Calculate f0 and psides0 since polynomial if fit to 3 degree
     AD<double> f0 = coeffs[0] + (coeffs[1] * x0) + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
     AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

     // Here's `x` to get you started.
     // The idea here is to constraint this value to be 0.
     //
     // NOTE: The use of `AD<double>` and use of `CppAD`!
     // This is also CppAD can compute derivatives and pass
     // these to the solver.

     // Setup the rest of the model constraints
     fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
     fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
     fg[1 + psi_start + t] = psi1 - (psi0 + v0/Lf * delta0 * dt);
     fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
     fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
     fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0/Lf * delta0 * dt);

   }

  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Define state elements
  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];


  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332 ; // 25 degree in radian
    vars_upperbound[i] = 0.436332;  // 25 degree in radian
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }


  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> result;

  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  for (int i = 0; i < N-1; i++) {
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }

  return result;
  return {};
}

void MPC::ApplyMotionModel(double &x, double &y, double &psi, double &v,
    double a, double delta, double latency) {
    double v_n = v + (a * latency);
    double psi_n = psi - (v * delta * latency / Lf);
    double x_n = x + (v * cos(psi) * latency);
    double y_n = y + (v * sin(psi) * latency);

    x = x_n;
    y = y_n;
    v = v_n;
    psi = psi_n;
}
