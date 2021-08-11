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

#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/ISAM2.h>

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

Values solverLM(NonlinearFactorGraph &graph, Values &initialEstimate)
{
    // Optimize
    LevenbergMarquardtParams paramsLM;
    //   paramsLM.linearSolverType = LevenbergMarquardtParams::MULTIFRONTAL_QR;
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);

    return optimizer.optimize();
}

Values solverISAM2(NonlinearFactorGraph& graph, Values& initialEstimate){
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.0001;
    // parameters.relinearizeSkip = 1;
    ISAM2 isam(parameters);

    isam.update(graph, initialEstimate);

    return isam.calculateEstimate();
}

Values solverISAM(NonlinearFactorGraph &graph, Values &initialEstimate)
{
    int relinearizeInterval = 1;
    NonlinearISAM isam(relinearizeInterval);

    isam.update(graph, initialEstimate);

    return isam.estimate();
}

int main(int argc, char** argv) {
  
  // Load Victoria dataset
  const string filename = findExampleDataFile("victoria_park.txt");
  const string testfilename = "victoria_test_in.txt";
  const string outfilename = "victoria_test_out.txt";

  // Create a factor graph and load dataset
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initialEstimate;
  boost::tie(graph, initialEstimate) = load2D(filename);

  // Add a prior on pose x1 at the origin. A prior factor consists of a mean and
  auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.0, 0.0, 0.0));
  graph->addPrior(0, initialEstimate->at(0).cast<Pose2>(), priorNoise); // add directly to graph

  // Save input estimate for plotting
  // saveResults(*initialEstimate, testfilename);

  // Test marginals
  KeyVector keys{Symbol('l', 6222), Symbol('l', 6253), Symbol('l', 6218)};
  Marginals marginals(*graph, *initialEstimate);
  JointMarginal joint = marginals.jointMarginalCovariance(keys);
  std::cout << joint.fullMatrix() << std::endl;

  // Optimize
//   Values result = solverLM(*graph, *initialEstimate);
  Values result = solverISAM(*graph, *initialEstimate);

  // Save trajectory and map
  saveResults(result, outfilename);

//   result.print("Current estimate: ");

  Marginals marginals_results(*graph, result);
  JointMarginal joint_results = marginals_results.jointMarginalCovariance(keys);
  std::cout << joint_results.fullMatrix() << std::endl;
  
  return 0;
} 
