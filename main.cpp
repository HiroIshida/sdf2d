#include <array>
#include <vector>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

#include <time.h>

using namespace std;
using uint = unsigned int;
using array2d = std::array<double, 2>;

array2d subt(const array2d& x, const array2d& y){
  array2d out = {x[0] - y[0], x[1] - y[1]};
  return out;
}

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

void construct_check_inside_map(
    const vector<array<double, 2>>& V,
    const vector<array<uint, 2>>& E,
    vector<vector<bool>>& check_inside_map)
{
  for(auto& e : E)
  {
    auto& v0 = V[e[0]];
    auto& v1 = V[e[1]];
    array2d p, q;
    if(v0[0] < v1[0]){ // p is always lower than q in x axis
      p = v0; q = v1;
    }else{
      p = v1; q = v0;
    }

    array2d diff = subt(q, p);
    double inc = diff[1]/diff[0];

    auto xint_min = uint(std::ceil(p[0]));
    auto xint_max = uint(std::floor(q[0]));
    for(int xint=xint_min; xint <= xint_max; xint++){
      int y_intersect = std::ceil(inc*(xint - p[0]) + p[1]);
      check_inside_map[xint][y_intersect] = true;
    }
  }
  for(auto& yline : check_inside_map){
    for(int i=1; i<yline.size(); i++){
      yline[i] = (yline[i] != yline[i-1]);
    }
  }
}

double compute_unsigned_distance(unsigned int xint, unsigned int yint,
    const vector<array<double, 2>>& V,
    const vector<array<uint, 2>>& E, 
    const vector<array<uint, 2>>& V2E)
{
  double sqdist_min = 9999999.0 ; // some random value
  unsigned int vert_idx_closest;
  for(int i=0; i<V.size(); i++){
    auto& vert = V[i];
    double sqdist = pow(vert[0] - (double)xint, 2) + pow(vert[1] - (double)yint, 2);
    if(sqdist < sqdist_min){
      sqdist_min = sqdist;
      vert_idx_closest = i;
    }
  }
  return sqdist_min;
}

int main(){

  auto cdata = load_testdata();
  auto& b_min = cdata.b_min;
  auto& b_max = cdata.b_max;
  auto& N = cdata.N;

  array2d w = {double(N[0])/(b_max[0] - b_min[0]), double(N[1])/(b_max[1] - b_min[1])};

  auto& E = cdata.E;
  auto V = cdata.V; 
  for(auto& v : V){
    for(int i=0; i<2; i++){v[i] = (v[i] - b_min[i]) * w[i];}
  }

  vector<array<uint, 2>> V2E(V.size());
  vector<unsigned int> counters(V.size());
  for(int i=0; i<E.size(); i++){
    for(int j=0; j<2; j++){
      auto idx_v = E[i][j];
      V2E[idx_v][counters[idx_v]] = i;
      counters[idx_v] += 1;
    }
  }

  // TODO for a large data, hashtable like data structure would be prefarable
  vector<vector<bool>> isInside(N[0], vector<bool>(N[1], false));
  construct_check_inside_map(V, E, isInside);

  vector<vector<double>> sdf(N[0], vector<double>(N[1]));

  for(int i=0; i<N[0]; i++){
    for(int j=0; j<N[1]; j++){
      double dist = compute_unsigned_distance(i, j, V, E, V2E);
      sdf[i][j] = (isInside[i][j] ? -dist : dist);
    }
  }

  /*
  for(auto& yline : sdf){
    string hoge;
    for(auto y : yline){ cout << y;}
    cout << endl;
  }
  */
}
