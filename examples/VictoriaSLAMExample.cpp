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
  const string filename = findExampleDataFile("victoria_park.txt");
  const string testfilename = "victoria_test_in.txt";
  const string outfilename = "victoria_test_out.txt";

  // Create a factor graph
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initialEstimate;
  // Load all
  boost::tie(graph, initialEstimate) = load2D(filename);

  // Add a prior on pose x1 at the origin. A prior factor consists of a mean and
  // a noise model (covariance matrix)
  Pose2 prior(0.0, 0.0, 0.0); // prior mean is at origin
  auto priorNoise = noiseModel::Diagonal::Sigmas(
      Vector3(0.0, 0.0, 0.0));           // 30cm std on x,y, 0.1 rad on theta
  graph->addPrior(0, prior, priorNoise); // add directly to graph

  saveResults(*initialEstimate, testfilename);

  // Optimize
  LevenbergMarquardtParams paramsLM;
  paramsLM.linearSolverType = LevenbergMarquardtParams::MULTIFRONTAL_QR;
  LevenbergMarquardtOptimizer optimizer(*graph, *initialEstimate, paramsLM);
  Values result = optimizer.optimize();

  saveResults(result, outfilename);

//   std::cout << "First pose" << std::endl;
//   Pose2 pose = result.at(0).cast<Pose2>();
//   std::cout << pose.x() << " " << pose.y() << " " << pose.theta() << std::endl;

      return 0;
} 
