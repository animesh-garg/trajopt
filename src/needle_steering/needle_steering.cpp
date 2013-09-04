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
    //cout << planner->Ts[0] << endl;
    sol = planner->GetSolutionsWithoutFirstTimestep(planner->Solve(sol));
    states.push_back(planner->SimulateExecution(states.back()));
    if (sim_plotting) {
      sim_plotter->Plot(states, planner);
    }
    cout << "simulate one step" << endl;
  }

  return 0;

}
