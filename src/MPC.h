#pragma once

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

struct SolverResult {
  double steering;
  double throttle;
  vector<double> xpos;
  vector<double> ypos;
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  SolverResult Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs,
                     double max_steering, double Lf);
};