#include "Eigen-3.3/Eigen/Dense"
#include "jmt.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void JMT::set_boundaries(const vector<double>& start, const vector<double>& end, double T)
{
  /*
  Calculate the Jerk Minimizing Trajectory that connects the initial state
  to the final state in time T.

  INPUTS

  start - the vehicles start location given as a length three array
  corresponding to initial values of [s, s_dot, s_double_dot]

  end   - the desired end state for vehicle. Like "start" this is a
  length three array.

  T     - The duration, in seconds, over which this maneuver should occur.

  OUTPUT
  an array of length 6, each value corresponding to a coefficent in the polynomial
  s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

  EXAMPLE

  > JMT( [0, 10, 0], [10, 10, 0], 1)
  [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
  */

  auto A = MatrixXd(3, 3);
  A <<  T*T*T, T*T*T*T, T*T*T*T*T,
        3*T*T, 4*T*T*T, 5*T*T*T*T,
        6*T,   12*T*T,  20*T*T*T;

  auto B = MatrixXd(3, 1);
  B <<  end[0] - (start[0] + start[1] * T + .5*start[2] * T*T),
        end[1] - (start[1] + start[2] * T),
        end[2] - start[2];

  MatrixXd Ai = A.inverse();

  MatrixXd C = Ai*B;

  // Overwrite previous coefficients.
  _coeffs = { start[0], start[1], .5*start[2] };

  for (unsigned int i = 0; i < C.size(); i++)
  {
    _coeffs.push_back(C.data()[i]);
  }

  assert(_coeffs.size() == 6);
}

double JMT::operator() (double x) const
{
  // Computes the polynomial with the previously computed coefficients
  return _coeffs[0] + _coeffs[1] * x + _coeffs[2] * x*x + _coeffs[3] * x*x*x + _coeffs[4] * x*x*x*x + _coeffs[5] * x*x*x*x*x;
}

vector<double> JMT::operator()(const vector<double>& start, const vector<double>& end, double T)
{
  set_boundaries(start, end, T);
  return _coeffs;
}
