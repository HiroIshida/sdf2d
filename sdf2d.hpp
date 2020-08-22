#include <array>
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>

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

void construct_check_inside_map(
    const vector<array<double, 2>>& V,
    const vector<array<uint, 2>>& E,
    const array<double, 2>& w_grid,
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
    double eps = 1e-3;
    bool isEqualVertex = (abs(diff[0]) < w_grid[0] * eps || abs(diff[1]) < w_grid[1] * eps);
    if(!isEqualVertex){
      double inc = diff[1]/diff[0];

      auto xint_min = uint(std::ceil(p[0]));
      auto xint_max = uint(std::floor(q[0]));
      for(int xint=xint_min; xint <= xint_max; xint++){
        int y_intersect = std::ceil(inc*(xint - p[0]) + p[1]);
        // tempting to set just "true". but we must handle the case when even number intersections exist at the grid point.
        // Thus, if already check_inside_map must be reversed like below. (true->false, false->true)
        check_inside_map[xint][y_intersect] = (check_inside_map[xint][y_intersect] == false); 
      }
    }
  }

  for(auto& yline : check_inside_map){
    for(int i=1; i<yline.size(); i++){
      yline[i] = (yline[i] != yline[i-1]);
    }
  }
}

double compute_unsigned_distance(double x, double y,
    const vector<array<double, 2>>& V,
    const vector<array<uint, 2>>& E, 
    const vector<array<uint, 2>>& V2E)
{

  double sqdist_min = 9999999.0 ; // some random value
  unsigned int vert_idx_closest;
  for(int i=0; i<V.size(); i++){
    auto& vert = V[i];
    double sqdist = pow(vert[0] - x, 2) + pow(vert[1] - y, 2);
    if(sqdist < sqdist_min){
      sqdist_min = sqdist;
      vert_idx_closest = i;
    }
  }

  array2d p = {x, y};
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

void construct_signed_distance_field(
  const vector<array<double, 2>>& V,
  const vector<array<uint, 2>>& E,
  const array<double, 2>& b_min,
  const array<double, 2>& b_max,
  const array<uint, 2>& N,
  vector<vector<double>>& out_sdf)
{
  // scaling from original coordinate to interger-based coordinates starting from (0, 0)
  array2d w_grid = {(b_max[0] - b_min[0])/(N[0] - 1.0), (b_max[1] - b_min[1])/(N[1] - 1.0)};
  auto V_scaled = V; 
  double eps_adhoc = 1e-5; // to avoid a vertex is exactly on the interger grid
  for(auto& v : V_scaled){
    for(int i=0; i<2; i++){v[i] = ((v[i] - b_min[i]) / w_grid[i]) - eps_adhoc;}
  }

  vector<vector<bool>> isInside(N[0], vector<bool>(N[1], false));
  clock_t start = clock();
  construct_check_inside_map(V_scaled, E, w_grid, isInside);
  cout << "better: " << clock() - start << endl;

  vector<array<uint, 2>> V2E(V.size());
  vector<unsigned int> counters(V.size());
  for(int i=0; i<E.size(); i++){
    for(int j=0; j<2; j++){
      auto idx_v = E[i][j];
      V2E[idx_v][counters[idx_v]] = i;
      counters[idx_v] += 1;
    }
  }

  for(int i=0; i<N[0]; i++){
    double x = b_min[0] + i * w_grid[0];
    for(int j=0; j<N[1]; j++){
      double y = b_min[1] + j * w_grid[1];
      double dist = compute_unsigned_distance(x, y, V, E, V2E);
      out_sdf[i][j] = (isInside[i][j] ? -dist : dist);
    }
  }
  /*
  for(auto& yline : isInside){
    string hoge;
    for(auto y : yline){ cout << y;}
    cout << endl;
  }
  */
}
