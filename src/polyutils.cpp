#include "polyutils.h"

// Evaluate a polynomial at value X.
double poly_eval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Evalute the first derivative at value X.
double poly_deriv_1(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += coeffs(i) * i * pow(x, i - 1);
  }
  return result;
}

// Fit a polynomial.
// Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd poly_fit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

Eigen::VectorXd poly_fit(std::vector<double> xvals, std::vector<double> yvals, int order) {
    Eigen::VectorXd xe(xvals.size());
    Eigen::VectorXd ye(yvals.size());
    for (int i = 0; i < xvals.size(); i++) {
        xe(i) = xvals[i];
        ye(i) = yvals[i];
    }
    return poly_fit(xe, ye, order);
}
