#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "sdf2d.hpp"

using namespace std;

vector<vector<double>> convert2sdf(
  const vector<array<double, 2>>& V,
  const vector<array<unsigned int, 2>>& E,
  const array<double, 2>& b_min,
  const array<double, 2>& b_max,
  const array<unsigned int, 2>& N)
{
  vector<vector<double>> sdf(N[0], vector<double>(N[1]));
  construct_signed_distance_field(V, E, b_min, b_max, N, sdf);
  return sdf;
}

PYBIND11_MODULE(sdf2d, m) {
    m.doc() = "convert 2dim mesh to sdf"; // optional module docstring
    m.def("convert2sdf", &convert2sdf);
}
