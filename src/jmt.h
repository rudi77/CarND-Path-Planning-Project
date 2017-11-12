#pragma once
#include <vector>
#include <string>
#include <sstream>

class JMT
{
public:
  // Set boundary conditons and keeps the computed coefficients internally.
  void set_boundaries(const std::vector< double>& start, const std::vector <double>& end, double T);

  double operator() (double x) const;

  std::vector<double> operator() (const std::vector<double>& start, const std::vector<double>& end, double T);

  std::string to_string() const
  {
    std::stringstream ss;
    ss << _coeffs[0] << "," << _coeffs[1] << "," << _coeffs[2] << "," << _coeffs[4] << "," << _coeffs[5];
    return ss.str();
  }
private:
  std::vector<double> _coeffs;
};
