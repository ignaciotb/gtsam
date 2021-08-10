/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>

#include <gtsam/inference/Symbol.h>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/slam/dataset.h>

using namespace std;
using namespace gtsam;

void saveResults(const Values& result, const string& outfilename)
{
    fstream stream(outfilename.c_str(), fstream::out);

    auto index = [](gtsam::Key key){ return Symbol(key).index(); };

    // save 2D poses
    for (const auto key_value : result)
    {
        auto p = dynamic_cast<const GenericValue<Pose2> *>(&key_value.value);
        if (!p)
            continue;
        const Pose2 &pose = p->value();
        stream << "Pose " << index(key_value.key) << " " << pose.x() << " "
               << pose.y() << " " << pose.theta() << endl;
    }

    // save 2D landmarks
    for (const auto key_value : result)
    {
        auto p = dynamic_cast<const GenericValue<Point2> *>(&key_value.value);
        if (!p)
            continue;
        const Point2 &point = p->value();
        stream << "Landmark " << index(key_value.key) << " " << point.x() << " "
               << point.y() << endl;
    }

    stream.close();
    std::cout << "Results saved" << std::endl;
}

int main(int argc, char** argv) {
  
  // Load Victoria dataset
  const string filename = findExampleDataFile("victoria_park_test.txt");
  const string outfilename = "victoria_test_out.txt";
  
  // Create a factor graph
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initialEstimate;
  // Load all
  boost::tie(graph, initialEstimate) = load2D(filename);
  
  // Print
  graph->print("Factor Graph:\n");

  // Print
  initialEstimate->print("Initial Estimate:\n");

  // Optimize
  LevenbergMarquardtOptimizer optimizer(*graph, *initialEstimate);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  saveResults(result, outfilename);

//   std::cout << "First pose" << std::endl;
//   Pose2 pose = result.at(0).cast<Pose2>();
//   std::cout << pose.x() << " " << pose.y() << " " << pose.theta() << std::endl;

      return 0;
} 
