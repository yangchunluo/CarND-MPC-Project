#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
const size_t N = 10;
const double dt = 0.1;

// Both the reference cross track and orientation errors are 0.
// The reference velocity is set here.
const double ref_velocity = 100;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
public:
  // Fitted polynomial coefficients, used in compute the desired psi (psides).
  Eigen::VectorXd coeffs;
  double Lf;
  FG_eval(Eigen::VectorXd coeffs, double Lf): coeffs(coeffs), Lf(Lf) {}

  // ADvector is mandated by the CppAD::ipopt interface.
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  /*
   * The overloading operation () is mandated by the CppAD::ipopt interface.
   *
   * `fg` is a vector containing the cost and constraints.
   * `vars` is a vector containing the variable values (state & actuators).
   */
  void operator()(ADvector& fg, const ADvector& vars) {
    /*
     * Setup cost function.
     * The cost is stored is the first element of fg.
     */
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += 2000 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 2000 * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_velocity, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += 5 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 5 * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      // Add more penality to steering angle change.
      fg[0] += 200 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 10 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    /*
     * Setup Constraints.
     * There is an offset index of 1 since fg[0] is used for the cost function.
     */

    // Initial constraints.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints.
    for (int t = 1; t < N; t++) {
      // Vehicle states at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Actuations at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      // Evalute the polynomial at time t: f(x0) and f'(x0).
      // However, we should abstain from using poly_eval() and poly_deriv_1() because we
      // need to rely on x0 being AD<double> for auto-differentiation.
      AD<double> poly_val = coeffs[0];
      AD<double> poly_der = 0;
      AD<double> curr_pow = 1;
      for (int i = 1; i < coeffs.size(); i++) {
        AD<double> next_pow = curr_pow * x0;
        poly_val += coeffs[i] * next_pow;
        poly_der += i * coeffs[i] * curr_pow;
        curr_pow = next_pow;
      }
      AD<double> f0 = poly_val;
      AD<double> psides0 = CppAD::atan(poly_der);

      // Vehicle states at time t+1.
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // Constrain the difference to be 0 (see the solver function).
      // Global kinematic model state transition.
      //   x[t+1]    = x[t] + v[t] * cos(psi[t]) * dt
      //   y[t+1]    = y[t] + v[t] * sin(psi[t]) * dt
      //   psi[t+1]  = psi[t] + v[t] / Lf * delta[t] * dt
      //   v[t+1]    = v[t] + a[t] * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      // For error turns, always use (actual - reference).
      //   cte[t+1]  = (y[t] - f(x[t])) + v[t] * sin(epsi[t]) * dt
      //   epsi[t+1] = (psi[t] - psides[t]) + v[t] * delta[t] / Lf * dt
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

SolverResult MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs,
                        double max_steering, double Lf) {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Set the number of model variables (includes both states and inputs):
  //   Vehicle states in N time steps, actuators in N-1 time steps.
  size_t n_vars = 6 * N + 2 * (N - 1);
  // Set the number of constraints: vehicle states in N time steps
  size_t n_constraints = 6 * N;

  // Set initial value for variables: 0 exepct for the initial state variables.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Set lower and upper limits for variables.
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  // Limits of steering angle are set to -25 and 25 in degrees.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -max_steering * Lf;
    vars_upperbound[i] = max_steering * Lf;
  }
  // Limits of throttle/brake.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints: 0 except for the initial state variables.
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

  // Object that computes objective (cost function) and constraints
  FG_eval fg_eval(coeffs, Lf);

  // Options for IPOPT solver
  std::string options;
  // Comment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // FIXME: I suspect that max_cpu_time is internally cached like a static variable.
  //        We must find an API to reset it before each call to the solver.
  // options += "Numeric max_cpu_time          0.5\n";

  // Solve the problem
  CppAD::ipopt::solve_result<Dvector> solution;
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check result status.
  if (solution.status != CppAD::ipopt::solve_result<Dvector>::success) {
    std::cerr << "IPOPT error" << endl;
  }

  // Cost
  // std::cout << "Cost " << solution.obj_value << std::endl;

  // Return the first actuator values.
  // Also sent back are (x, y) coordinates for display.
  SolverResult res;
  res.steering = solution.x[delta_start];
  res.throttle = solution.x[a_start];
  res.cost = solution.obj_value;
  for (int i = 1; i < N; i++) {
    res.xpos.emplace_back(solution.x[x_start + i]);
    res.ypos.emplace_back(solution.x[y_start + i]);
  }
  return res;
}
