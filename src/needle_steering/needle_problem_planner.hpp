#pragma once

#include "common.hpp"
#include "fwd.hpp"
#include "needle_problem_helper.hpp"

namespace Needle {
  struct NeedleProblemPlanner {
    int argc;
    char **argv;
    int n_needles;
    vector<int> Ts;
    NeedleProblemHelperPtr helper;

    bool stage_plotting;
    bool plot_final_result;
    bool verbose;
    bool is_first_needle_run;
    double env_transparency;
    double deviation;
    string data_dir;
    string env_file_path;
    string robot_file_path;

    EnvironmentBasePtr env;
    boost::shared_ptr<TrajPlotter> plotter;

    vector<Vector6d> starts;
    vector<Vector6d> goals;
    DblVec x;
    vector<Vector3d> start_position_error_relax;
    vector<double> start_orientation_error_relax;
    vector<double> goal_distance_error_relax;
    vector<KinBodyPtr> managed_kinbodies;
    vector<LocalConfigurationPtr> managed_configs;

    vector< vector<Vector6d> > simulated_needle_trajectories;

    //boost::shared_ptr<OptimizerT> opt;
    //OptProbPtr prob;

    NeedleProblemPlanner(int argc, char **argv);
    ~NeedleProblemPlanner();
    Vector6d PerturbState(const Vector6d& state);
    vector<VectorXd> Solve(const vector<VectorXd>& initial=vector<VectorXd>());
    vector<VectorXd> GetSolutionsWithoutFirstTimestep(const vector<VectorXd>& sol);
    DblVec Solve(const DblVec& x);
    void SimulateExecution();
    void AddSimulatedNeedleToBullet(const vector<Vector6d>& states);
    bool Finished() const;
  };
}
