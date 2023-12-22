#pragma once

#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "hybrid_astar_searcher/node3d.h"
#include "math.h"

namespace planning
{

struct ReedSheppPath {
  std::vector<double> segs_lengths;
  std::vector<char> segs_types;
  double total_length = 0.0;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  // true for driving forward and false for driving backward
  std::vector<bool> gear;
};

struct RSPParam {
  bool flag = false;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
};

class ReedShepp {
 public:
  ReedShepp(const double max_kappa, const double step_size);
  // Pick the shortest path from all possible combination of movement primitives
  // by Reed Shepp
  bool ShortestRSP(const std::shared_ptr<Node3d> start_node,
                   const std::shared_ptr<Node3d> end_node,
                   ReedSheppPath& optimal_path);

 protected:
  // Generate all possible combination of movement primitives by Reed Shepp and
  // interpolate them
  bool GenerateRSPs(const std::shared_ptr<Node3d> start_node,
                    const std::shared_ptr<Node3d> end_node,
                    std::vector<ReedSheppPath>* all_possible_paths);
  // Set the general profile of the movement primitives
  bool GenerateRSP(const std::shared_ptr<Node3d> start_node,
                   const std::shared_ptr<Node3d> end_node,
                   std::vector<ReedSheppPath>* all_possible_paths);
  // Set the general profile of the movement primitives, parallel implementation
  bool GenerateRSPPar(const std::shared_ptr<Node3d> start_node,
                      const std::shared_ptr<Node3d> end_node,
                      std::vector<ReedSheppPath>* all_possible_paths);
  // Set local exact configurations profile of each movement primitive
  bool GenerateLocalConfigurations(const std::shared_ptr<Node3d> start_node,
                                   const std::shared_ptr<Node3d> end_node,
                                   ReedSheppPath* shortest_path);
  // Interpolation usde in GenetateLocalConfiguration
  void Interpolation(const int index, const double pd, const char m,
                     const double ox, const double oy, const double ophi,
                     std::vector<double>* px, std::vector<double>* py,
                     std::vector<double>* pphi, std::vector<bool>* pgear);
  // motion primitives combination setup function
  bool SetRSP(const int size, const double* lengths, const char* types,
              std::vector<ReedSheppPath>* all_possible_paths);
  // setRSP parallel version
  bool SetRSPPar(const int size, const double* lengths,
                 const std::string& types,
                 std::vector<ReedSheppPath>* all_possible_paths, const int idx);
  // Six different combination of motion primitive in Reed Shepp path used in
  // GenerateRSP()
  bool SCS(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CSC(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CCC(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CCCC(const double x, const double y, const double phi,
            std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSC(const double x, const double y, const double phi,
            std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSCC(const double x, const double y, const double phi,
             std::vector<ReedSheppPath>* all_possible_paths);
  // different options for different combination of motion primitives
  void LSL(const double x, const double y, const double phi, RSPParam* param);
  void LSR(const double x, const double y, const double phi, RSPParam* param);
  void LRL(const double x, const double y, const double phi, RSPParam* param);
  void SLS(const double x, const double y, const double phi, RSPParam* param);
  void LRLRn(const double x, const double y, const double phi, RSPParam* param);
  void LRLRp(const double x, const double y, const double phi, RSPParam* param);
  void LRSR(const double x, const double y, const double phi, RSPParam* param);
  void LRSL(const double x, const double y, const double phi, RSPParam* param);
  void LRSLR(const double x, const double y, const double phi, RSPParam* param);
  std::pair<double, double> calc_tau_omega(const double u, const double v,
                                           const double xi, const double eta,
                                           const double phi);

  double max_kappa_;
  double step_size_;
};
}