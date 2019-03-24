#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include "Eigen-3.3/Eigen/Core"

using Eigen::VectorXd;
using std::string;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions to fit and evaluate polynomials.
//

// Evaluate a polynomial.
double polyeval(const VectorXd &coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); ++i) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from:
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);

  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); ++i) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); ++j) {
    for (int i = 0; i < order; ++i) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);

  return result;
}

//vehicle model calculation
VectorXd globalKinematic(const VectorXd &state, 
                         const VectorXd &actuators, 
                         const double dt, 
                         const double Lf, 
                         const VectorXd &coeffs) {
  // Create a new vector for the next state.
  VectorXd next_state(state.size());

  // NOTE: state is [x, y, psi, v, cte, epsi]
  auto x = state(0);
  auto y = state(1);
  auto psi = state(2);
  auto v = state(3);
  auto cte = state(4);
  auto epsi = state(5);

  // NOTE: actuators is [delta, a]
  auto delta = actuators(0);
  auto a = actuators(1);

  // Recall the equations for the model:
  // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
  // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
  // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
  // v_[t+1] = v[t] + a[t] * dt
  // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
  // epsi[t+1] = psi[t] - arctan(f'(x[t])) + v[t] * psi[t] / Lf * dt
  next_state(0) = x + v * cos(psi) * dt;
  next_state(1) = y + v * sin(psi) * dt;
  next_state(2) = psi + v / Lf * (-delta) * dt;
  next_state(3) = v + a * dt;
  next_state(4) = cte + v * sin(epsi) * dt;
  next_state(5) = epsi + v / Lf * (-delta) * dt; 
  return next_state;
}

#endif  // HELPERS_H