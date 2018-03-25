#include <vector>
#include <iostream>
#include <cmath>
#include "polyutils.h"
#include "cassert"

using namespace std;

static void assert_double(double actual, double expect) {
    const double EPSILON = 1e-6;
    assert(abs(expect - actual) < EPSILON);
}

int main() {
    {
        vector<double> xvals = {0, 1, 2};
        vector<double> yvals = {1, 3, 5};
        auto coeffs = poly_fit(xvals, yvals, 1);
        assert_double(coeffs(0), 1);
        assert_double(coeffs(1), 2);

        assert_double(poly_eval(coeffs, 5), 5 * 2 + 1);
        assert_double(poly_deriv_1(coeffs, 5), 2);
    }

    {
        vector<double> xvals = {1, 2, 3, 4};
        vector<double> yvals = {10, 49, 142, 313};
        auto coeffs = poly_fit(xvals, yvals, 3);
        assert_double(coeffs(0), 1);
        assert_double(coeffs(1), 2);
        assert_double(coeffs(2), 3);
        assert_double(coeffs(3), 4);

        assert_double(poly_eval(coeffs, -1), -2);
        assert_double(poly_eval(coeffs, 5), 586);
        assert_double(poly_deriv_1(coeffs, -1), 8);
        assert_double(poly_deriv_1(coeffs, 5), 332);
    }
    return 0;
}