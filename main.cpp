#include <array>
#include <vector>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <math.h>
#include <time.h>

using namespace std;
using uint = unsigned int;
using array2d = std::array<double, 2>;

array2d add(const array2d& x, const array2d& y){
  array2d out = {x[0] + y[0], x[1] + y[1]};
  return out;
}

array2d subt(const array2d& x, const array2d& y){
  array2d out = {x[0] - y[0], x[1] - y[1]};
  return out;
}

array2d mult(double x, const array2d& y){
  array2d out = {x*y[0], x*y[1]};
  return out;
}

double dot(const array2d& x, const array2d& y){
  return x[0]*y[0] + x[1]*y[1];
}

double sqnorm(const array2d& x){
  return dot(x, x);
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

  array2d p = {xint, yint};
  auto& adj_edge_idxes = V2E[vert_idx_closest];


  double sqdist_to_edge_min = 999999.0;
  for(auto edge_idx : adj_edge_idxes){
    auto& e = E[edge_idx];
    auto& v0 = V[e[0]];
    auto& v1 = V[e[1]];
    auto d = subt(v1, v0);
    double s = dot(subt(p, v0), d)/sqnorm(d); // like highschool math

    double sqdist;
    if(s < 0.0){
      sqdist = sqnorm(subt(v0, p));
    }else if(s > 1.0){
      sqdist = sqnorm(subt(v1, p));
    }else{
      sqdist = sqnorm(subt(p, add(v0, mult(s, d)))); // lisp??
    }
    if(sqdist < sqdist_to_edge_min){
      sqdist_to_edge_min = sqdist;
    }
  }
  return sqrt(sqdist_to_edge_min);
}

int main(){

  auto cdata = load_testdata();
  auto& b_min = cdata.b_min;
  auto& b_max = cdata.b_max;
  auto& N = cdata.N;

  auto& E = cdata.E;
  auto& V = cdata.V; 

  // scaling from original coordinate to interger-based coordinates starting from (0, 0)
  array2d w_grid = {(b_max[0] - b_min[0])/((double)N[0]), (b_max[1] - b_min[1])/((double)N[1])};
  auto V_scaled = V; 
  for(auto& v : V_scaled){
    for(int i=0; i<2; i++){v[i] = (v[i] - b_min[i]) / w_grid[i];}
  }

  // TODO for a large data, hashtable like data structure would be prefarable
  vector<vector<bool>> isInside(N[0], vector<bool>(N[1], false));
  construct_check_inside_map(V_scaled, E, isInside);

  vector<array<uint, 2>> V2E(V.size());
  vector<unsigned int> counters(V.size());
  for(int i=0; i<E.size(); i++){
    for(int j=0; j<2; j++){
      auto idx_v = E[i][j];
      V2E[idx_v][counters[idx_v]] = i;
      counters[idx_v] += 1;
    }
  }

  vector<vector<double>> sdf(N[0], vector<double>(N[1]));
  for(int i=0; i<N[0]; i++){
    for(int j=0; j<N[1]; j++){
      double dist = compute_unsigned_distance(i, j, V_scaled, E, V2E);
      sdf[i][j] = (isInside[i][j] ? -dist : dist);
    }
  }

  nlohmann::json j;
  j["data"] = sdf;

  ofstream outputfile("../data.json");
  outputfile << j.dump();
  outputfile.close();

  /*
  for(auto& yline : sdf){
    string hoge;
    for(auto y : yline){ cout << y;}
    cout << endl;
  }
  */
}
