#pragma once

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// Evaluate a polynomial at value X.
extern double poly_eval(Eigen::VectorXd coeffs, double x);

// Evalute the first derivative of a polynomial at value X.
extern double poly_deriv_1(Eigen::VectorXd coeffs, double x);

// Fit a polynomial.
extern Eigen::VectorXd poly_fit(std::vector<double> xvals, std::vector<double> yvals, int order);
extern Eigen::VectorXd poly_fit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);