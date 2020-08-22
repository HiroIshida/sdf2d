#include <time.h>
#include <nlohmann/json.hpp>
#include "sdf2d.hpp"

struct ContourData
{
  array<double, 2> b_min;
  array<double, 2> b_max;
  array<uint, 2> N;
  vector<array<uint, 2>> E;
  vector<array<double, 2>> V;
};

ContourData load_testdata(string filename="../test.json")
{
  ifstream my_file(filename);
  nlohmann::json js;
  js << my_file;
  array<double, 2> b_min = js["b_min"];
  array<double, 2> b_max = js["b_max"];
  array<uint, 2> N = js["N"];
  vector<array<uint, 2>> E = js["E"];
  vector<array<double, 2>> V = js["V"];
  ContourData cdata = {b_min, b_max, N, E, V};
  return cdata;
}


int main(){
  auto cdata = load_testdata();
  auto& b_min = cdata.b_min;
  auto& b_max = cdata.b_max;
  auto& N = cdata.N;
  auto& E = cdata.E;
  auto& V = cdata.V; 

  vector<vector<double>> sdf(N[0], vector<double>(N[1]));
  construct_signed_distance_field(V, E, b_min, b_max, N, sdf);

  nlohmann::json j;
  j["data"] = sdf;

  ofstream outputfile("../data.json");
  outputfile << j.dump();
  outputfile.close();
}
