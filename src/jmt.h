#pragma once
#include <vector>
#include <string>
#include <sstream>

class JMT
{
public:
  void set_boundaries(std::vector< double> start, std::vector <double> end, double T);

  double operator() (double x) const;

  std::string to_string() const
  {
    std::stringstream ss;
    ss << _coeffs[0] << "," << _coeffs[1] << "," << _coeffs[2] << "," << _coeffs[4] << "," << _coeffs[5];
    return ss.str();
  }
private:
  std::vector<double> _coeffs;
};
