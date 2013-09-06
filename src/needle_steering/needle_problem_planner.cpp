#include "needle_problem_planner.hpp"
#include "utils.hpp"
#include "traj_plotter.hpp"
#include "needle_collision_hash.hpp"

namespace Needle {

  inline double rndnum() {
	  return ((double) random()) / RAND_MAX;
  }

  inline double normal() {
    double u_1 = 0;
    while (u_1 == 0) {
      u_1 = rndnum();
    }
    double u_2 = 0;
    while (u_2 == 0) {
      u_2 = rndnum();
    }
    return sqrt(-2*log(u_1)) * sin(2*PI*u_2);
  }

  NeedleProblemPlanner::NeedleProblemPlanner(int argc, char **argv) :
    argc(argc),
    argv(argv),
    helper(new NeedleProblemHelper()),
    stage_plotting(false),
    verbose(false),
    plot_final_result(false),
    env_transparency(0.1),
    is_first_needle_run(true),
    deviation(INFINITY),
    data_dir(get_current_directory() + "/../data") {

    vector<string> start_string_vec;
    vector<string> goal_string_vec;
    start_string_vec.push_back("-11.67067,5.54934,0,0,0.78,0");
    goal_string_vec.push_back("-2.71912,8.00334,-1.12736,0,0.78,0");
    start_string_vec.push_back("-11.17067,5.04934,0,0,0.78,0");
    goal_string_vec.push_back("-3.2396,6.46645,0.301649,0,0.78,0");

    this->start_position_error_relax.push_back(Vector3d(0.05, 1.25, 1.25));
    this->start_orientation_error_relax.push_back(0.0873);
    this->goal_distance_error_relax.push_back(0.0025);
    this->start_position_error_relax.push_back(Vector3d(0.05, 1.25, 1.25));
    this->start_orientation_error_relax.push_back(0.0873);
    this->goal_distance_error_relax.push_back(0.25);

    vector<double> start_position_error_relax_x;
    vector<double> start_position_error_relax_y;
    vector<double> start_position_error_relax_z;

    int T = 25;
    
    Config config;
    config.add(new Parameter<bool>("stage_plotting", &this->stage_plotting, "stage_plotting"));
    config.add(new Parameter<bool>("plot_final_result", &this->plot_final_result, "plot_final_result"));
    config.add(new Parameter<bool>("verbose", &this->verbose, "verbose"));
    config.add(new Parameter<double>("env_transparency", &this->env_transparency, "env_transparency"));
    config.add(new Parameter<string>("data_dir", &this->data_dir, "data_dir"));
    config.add(new Parameter<string>("env_file_path", &this->env_file_path, "env_file_path"));
    config.add(new Parameter<string>("robot_file_path", &this->robot_file_path, "robot_file_path"));
    config.add(new Parameter< vector<string> >("start_vec", &start_string_vec, "s"));
    config.add(new Parameter< vector<string> >("goal_vec", &goal_string_vec, "g"));
    config.add(new Parameter< vector<double> >("start_position_error_relax_x", &start_position_error_relax_x, "start_position_error_relax_x"));
    config.add(new Parameter< vector<double> >("start_position_error_relax_y", &start_position_error_relax_y, "start_position_error_relax_y"));
    config.add(new Parameter< vector<double> >("start_position_error_relax_z", &start_position_error_relax_z, "start_position_error_relax_z"));
    config.add(new Parameter< vector<double> >("start_orientation_error_relax", &this->start_orientation_error_relax, "start_orientation_error_relax"));
    config.add(new Parameter< vector<double> >("goal_distance_error_relax", &this->goal_distance_error_relax, "goal_distance_error_relax"));
    config.add(new Parameter<int>("T", &T, "T"));
    CommandParser parser(config);
    parser.read(argc, argv, true);

    if (this->env_file_path.length() == 0) {
      this->env_file_path = data_dir + "/prostate.env.xml";
    }
    if (this->robot_file_path.length() == 0) {
      this->robot_file_path = data_dir + "/needlebot.xml";
    }

    if (start_string_vec.size() != goal_string_vec.size()) {
      throw std::runtime_error("The number of start and goal vectors must be the same!");
    }

    if (start_string_vec.size() == 0) {
      throw std::runtime_error("You must provide at least 1 start and 1 goal vector.");
    }

    RaveInitialize(false, verbose ? Level_Debug : Level_Info);
    this->env = RaveCreateEnvironment();
    this->env->StopSimulation();

    OSGViewerPtr viewer;
    if (this->stage_plotting || this->plot_final_result) {
      viewer = OSGViewer::GetOrCreate(env);
      assert(viewer);
    }

    this->env->Load(this->env_file_path);

    if (this->stage_plotting || this->plot_final_result) {
      viewer->SetAllTransparency(this->env_transparency);
    }


    this->n_needles = start_string_vec.size();
    this->starts.clear();
    this->goals.clear();

    for (int i = 0; i < n_needles; ++i) {
      DblVec start_vec;
      DblVec goal_vec;
      strtk::parse(start_string_vec[i], ",", start_vec);
      strtk::parse(goal_string_vec[i], ",", goal_vec);
      Vector6d start = toVectorXd(start_vec), goal = toVectorXd(goal_vec);
      //Matrix4d start_pose, goal_pose;
      //start_pose.topLeftCorner<3, 3>() = rotMat(start.tail<3>());
      //start_pose.topRightCorner<3, 1>() = start.head<3>();
      //start_pose.bottomLeftCorner<1, 3>() = Vector3d::Zero();
      //start_pose(3, 3) = 1;
      //goal_pose.topLeftCorner<3, 3>() = rotMat(goal.tail<3>());
      //goal_pose.topRightCorner<3, 1>() = goal.head<3>();
      //goal_pose.bottomLeftCorner<1, 3>() = Vector3d::Zero();
      //goal_pose(3, 3) = 1;
      starts.push_back(logDown(se4Up(start)));//start_pose));
      goals.push_back(logDown(se4Up(goal)));//goal_pose));
    }

    for (int i = 0; i < n_needles; ++i) {
      this->Ts.push_back(T);
    }

    if (!(start_position_error_relax_x.size() == start_position_error_relax_y.size() && start_position_error_relax_y.size() == start_position_error_relax_z.size())) {
      throw std::runtime_error("start position error relaxes must have the same size.");
    }

    if (start_position_error_relax_x.size() > 0) {
      this->start_position_error_relax.clear();
      for (int i = 0; i < start_position_error_relax_x.size(); ++i) {
        this->start_position_error_relax.push_back(Vector3d(start_position_error_relax_x[i],
                                                            start_position_error_relax_y[i],
                                                            start_position_error_relax_z[i]));
      }
    }

    trajopt::SetUserData(*env, "trajopt_cc_hash", CollisionHashPtr(new NeedleCollisionHash(helper)));
  }

  vector<VectorXd> NeedleProblemPlanner::Solve(const vector<VectorXd>& initial) {
    trajopt::SetUserData(*this->env, "trajopt_cc", OpenRAVE::UserDataPtr());
    helper->InitParametersFromConsole(this->argc, this->argv);
    helper->n_needles = this->n_needles;
    helper->starts = this->starts;
    helper->goals = this->goals;
    helper->start_position_error_relax = this->start_position_error_relax;
    helper->start_orientation_error_relax = this->start_orientation_error_relax;
    helper->goal_distance_error_relax = this->goal_distance_error_relax;
    helper->Ts = this->Ts;

    if (helper->Ts.front() == 1) {
      helper->trust_box_size = .01;//merit_error_coeff = 10000;
      //helper->max_merit_coeff_increases -= 6;
    }
    /*} else */if (this->deviation > 0.01) {
      helper->merit_error_coeff = 10;
    } else {
      helper->merit_error_coeff = 100;
      helper->max_merit_coeff_increases -= 2;
    }

    if (!this->is_first_needle_run) { // fix start position if not first run
      helper->start_position_error_relax.front() = Vector3d::Zero();
      helper->start_orientation_error_relax.front() = 0;
    }

    for (int i = 0; i < n_needles; ++i) {
      helper->robots.push_back(this->env->ReadRobotURI(RobotBasePtr(), this->robot_file_path));
      this->env->Add(helper->robots.back(), true);
    }

    OptProbPtr prob(new OptProb());
    //prob.reset(new OptProb());
    helper->ConfigureProblem(*prob);
    OptimizerT opt(prob);//.reset(new OptimizerT(prob));
    //OptimizerT opt(prob);
    helper->ConfigureOptimizer(opt);

    if (initial.size() == helper->n_needles) {
      helper->SetSolutions(initial, opt);
    }

    if (this->stage_plotting || this->plot_final_result) {
      this->plotter.reset(new Needle::TrajPlotter());
    }
    if (this->stage_plotting) {
      opt.addCallback(boost::bind(&Needle::TrajPlotter::OptimizerCallback, boost::ref(this->plotter), _1, _2, helper));
    }

    opt.optimize();

    this->x = opt.x();
    
    return helper->GetSolutions(opt);
  }

  Vector6d NeedleProblemPlanner::PerturbState(const Vector6d& state) {
    Vector6d ret = state;
    for (int i = 0; i < 3; ++i) {
      ret(i) += normal() * 0.05;
    }
    for (int i = 3; i < 6; ++i) {
      ret(i) += normal() * 0.025;
    }
    return ret;
  }

  void NeedleProblemPlanner::SimulateExecution() {

    double phi = helper->GetPhi(this->x, 0, helper->pis.front());
    double Delta = helper->GetDelta(this->x, 0, helper->pis.front());
    double curvature_or_radius = helper->GetCurvatureOrRadius(this->x, 0, helper->pis.front());

    if (this->is_first_needle_run) {
      simulated_needle_trajectories.push_back(vector<Vector6d>());
      simulated_needle_trajectories.back().push_back(logDown(helper->pis.front()->local_configs.front()->pose));
    }
    Vector6d state_to_change = simulated_needle_trajectories.back().back();

    Vector6d new_state_without_noise = logDown(helper->TransformPose(expUp(state_to_change), phi, Delta, curvature_or_radius));
    Vector6d new_state = PerturbState(new_state_without_noise);
    this->deviation = (new_state_without_noise - new_state).norm();

    //if (this->is_first_needle_run) {
    //  simulated_needle_trajectories.push_back(vector<Vector6d>());
    //}

    simulated_needle_trajectories.back().push_back(new_state);
    
    this->starts.front() = new_state;

    if (Ts.front() > 1) {
      --Ts.front();
      this->is_first_needle_run = false;
    } else {
      // get rid of first time step
      this->Ts.erase(this->Ts.begin());
      this->starts.erase(this->starts.begin());
      this->goals.erase(this->goals.begin());
      this->start_position_error_relax.erase(this->start_position_error_relax.begin());
      this->start_orientation_error_relax.erase(this->start_orientation_error_relax.begin());
      this->goal_distance_error_relax.erase(this->goal_distance_error_relax.begin());
      this->is_first_needle_run = true;
      this->AddSimulatedNeedleToBullet(this->simulated_needle_trajectories.back());
      --n_needles;
    }

  }

  vector<VectorXd> NeedleProblemPlanner::GetSolutionsWithoutFirstTimestep(const vector<VectorXd>& sol) {
    if (sol.size() > 0) {
      return helper->GetSolutionsWithoutFirstTimestep(sol);
    } else {
      return sol;
    }
  }

  bool NeedleProblemPlanner::Finished() const {
    return this->Ts.size() == 0;
  }

  NeedleProblemPlanner::~NeedleProblemPlanner() {
    RaveDestroy();
  }

  void NeedleProblemPlanner::AddSimulatedNeedleToBullet(const vector<Vector6d>& states) {
    KinBodyPtr robot = this->env->ReadRobotURI(RobotBasePtr(), this->robot_file_path);
    this->managed_kinbodies.push_back(robot);
    this->env->Add(robot, true);
    boost::shared_ptr<BulletCollisionChecker> cc = boost::dynamic_pointer_cast<BulletCollisionChecker>(CollisionChecker::GetOrCreate(*this->env));

    vector<LocalConfigurationPtr> local_configs;
    for (int i = 0; i < states.size(); ++i) {
      local_configs.push_back(LocalConfigurationPtr(new LocalConfiguration(robot)));
      local_configs.back()->pose = expUp(states[i]);
      this->managed_configs.push_back(local_configs.back());
    }
    for (int i = 0; i < (int) local_configs.size() - 1; ++i) {
      vector<KinBody::LinkPtr> links;
      vector<int> inds;
      local_configs[i]->GetAffectedLinks(links, true, inds);
      cc->AddCastHullShape(*local_configs[i], *local_configs[i+1], links, toDblVec(Vector6d::Zero()), toDblVec(Vector6d::Zero()));
    }
    this->env->Remove(robot);
  }
}
