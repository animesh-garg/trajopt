#include "needle_steering.hpp"

using namespace Needle;

int main(int argc, char** argv) {

  NeedleProblemPlannerPtr planner(new NeedleProblemPlanner(argc, argv));

  vector<VectorXd> sol;
  vector< vector<Vector6d> > states;
  states.push_back(planner->starts);
  bool sim_plotting = false;

  {
    Config config;
    config.add(new Parameter<bool>("sim_plotting", &sim_plotting, "sim_plotting"));
    CommandParser parser(config);
    parser.read(argc, argv, true);
  }

  boost::shared_ptr<NeedleSimPlotter> sim_plotter;

  if (sim_plotting) {
    sim_plotter.reset(new NeedleSimPlotter());
  }

  while (!planner->Finished()) {
    sol = planner->GetSolutionsWithoutFirstTimestep(planner->Solve(sol));
    planner->SimulateExecution();
    if (sim_plotting) {
      sim_plotter->Plot(planner);
    }
  }

  return 0;

}
