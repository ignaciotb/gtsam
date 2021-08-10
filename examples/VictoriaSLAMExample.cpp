/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PlanarSLAMExample.cpp
 * @brief Simple robotics example using odometry measurements and bearing-range (laser) measurements
 * @author Alex Cunningham
 */

/**
 * A simple 2D planar slam example with landmarks
 *  - The robot and landmarks are on a 2 meter grid
 *  - Robot poses are facing along the X axis (horizontal, to the right in 2D)
 *  - The robot moves 2 meters each step
 *  - We have full odometry between poses
 *  - We have bearing and range information for measurements
 *  - Landmarks are 2 meters away from the robot trajectory
 */

// As this is a planar SLAM example, we will use Pose2 variables (x, y, theta) to represent
// the robot positions and Point2 variables (x, y) to represent the landmark coordinates.
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use a RangeBearing factor for the range-bearing measurements to identified
// landmarks, and Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// common Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

#include <gtsam/slam/dataset.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
  
  // Load Victoria dataset
  const string filename = findExampleDataFile("victoria_park_test.txt");
  // Create a factor graph
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initialEstimate;
  // Load all
  boost::tie(graph, initialEstimate) = load2D(filename);
  
  // Print
  graph->print("Factor Graph:\n");

  // Print
  initialEstimate->print("Initial Estimate:\n");
  
  // Save for testing
//   std::cout << "Saving graph" << std::endl;
//   save2D(*graph, *initialEstimate, nullptr, "victoria_test_out.txt");
//   std::cout << "Graph saved" << std::endl;

  // Optimize
  LevenbergMarquardtOptimizer optimizer(*graph, *initialEstimate);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  
  return 0;
} 
