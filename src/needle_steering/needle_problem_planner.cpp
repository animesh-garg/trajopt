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
    stage_result_plotting(false),
    verbose(false),
    env_transparency(0.1),
    is_first_needle_run(true),
    deviation(INFINITY),
    data_dir(get_current_directory() + "/../data") {

    vector<string> start_string_vec;
    vector<string> goal_string_vec;
    //start_string_vec.push_back("-11.67067,5.54934,0,0,0.78,0");
    //goal_string_vec.push_back("-2.71912,8.00334,-1.12736,0,0.78,0");
    //start_string_vec.push_back("-11.17067,5.04934,0,0,0.78,0");
    //goal_string_vec.push_back("-3.2396,6.46645,0.301649,0,0.78,0");

    start_string_vec.push_back("0,0,0,0,0,0");
    goal_string_vec.push_back("-0.875,0.0,7,-1.5707963267948966,-0.0,-0.0");
    start_string_vec.push_back("0,0,0,0,0,0");
    goal_string_vec.push_back("0.875,0.0,7,1.5707963267948966,0.0,0.0");

    #ifdef CHANNEL
    this->start_position_error_relax.push_back(Vector3d(1.75, 1.75, 0.05));
    this->start_orientation_error_relax.push_back(0.1745);
    this->goal_distance_error_relax.push_back(0);
    this->start_position_error_relax.push_back(Vector3d(1.75, 1.75, 0.05));
    this->start_orientation_error_relax.push_back(0.1745);
    this->goal_distance_error_relax.push_back(0);
    #else
    this->start_position_error_relax.push_back(Vector3d(0.05, 1.25, 1.25));
    this->start_orientation_error_relax.push_back(0.0873);
    this->goal_distance_error_relax.push_back(0.25);
    this->start_position_error_relax.push_back(Vector3d(0.05, 1.25, 1.25));
    this->start_orientation_error_relax.push_back(0.0873);
    this->goal_distance_error_relax.push_back(0.25);
    #endif

    vector<double> start_position_error_relax_x;
    vector<double> start_position_error_relax_y;
    vector<double> start_position_error_relax_z;

    int T = 25;
    
    Config config;
    config.add(new Parameter<bool>("stage_plotting", &this->stage_plotting, "stage_plotting"));
    config.add(new Parameter<bool>("stage_result_plotting", &this->stage_result_plotting, "stage_result_plotting"));
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
    if (this->stage_plotting || this->stage_result_plotting ) {
      viewer = OSGViewer::GetOrCreate(env);
      assert(viewer);
    }

    this->env->Load(this->env_file_path);

    if (this->stage_plotting || this->stage_result_plotting ) {
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
      starts.push_back(logDown(se4Up(start)));
      goals.push_back(logDown(se4Up(goal)));
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
    vector<KinBodyPtr> robots;
    vector<VectorXd> sol;
    for (int i = 0; i < n_needles; ++i) {

      helper->InitParametersFromConsole(this->argc, this->argv);
      helper->n_needles = 1;
      helper->starts.push_back(this->starts[i]);
      helper->goals.push_back(this->goals[i]);
      helper->start_position_error_relax.push_back(this->start_position_error_relax[i]);
      helper->start_orientation_error_relax.push_back(this->start_orientation_error_relax[i]);
      helper->goal_distance_error_relax.push_back(this->goal_distance_error_relax[i]);
      helper->Ts.push_back(this->Ts[i]);
      helper->robots.push_back(this->env->ReadRobotURI(RobotBasePtr(), this->robot_file_path));
      this->env->Add(helper->robots.back(), true);
      if (!this->is_first_needle_run && i == 0) { // fix start position if not first run
        helper->start_position_error_relax.front() = Vector3d::Zero();
        helper->start_orientation_error_relax.front() = 0;
      }

      OptProbPtr prob(new OptProb());
      helper->ConfigureProblem(*prob);
      OptimizerT opt(prob);
      helper->ConfigureOptimizer(opt);
      if (initial.size() == helper->n_needles) {
        vector<VectorXd> subinitial;
        subinitial.push_back(initial[i]);
        helper->SetSolutions(subinitial, opt);
      }
      if (!this->is_first_needle_run && i == 0) {
        helper->IntegrateControls(opt.x());
      }
      if (this->stage_plotting || this->stage_result_plotting) {
        this->plotter.reset(new Needle::TrajPlotter());
      }
      if (this->stage_plotting) {
        opt.addCallback(boost::bind(&Needle::TrajPlotter::OptimizerCallback, boost::ref(this->plotter), _1, _2, helper));
      }
      opt.optimize();
      for (int j = 0; j < helper->robots.size(); ++j) {
        robots.push_back(helper->robots[j]);
      }
      sol.push_back(helper->GetSolutions(opt).front());
    }
    trajopt::SetUserData(*env, "trajopt_cc", OpenRAVE::UserDataPtr());
    helper->InitParametersFromConsole(this->argc, this->argv);
    helper->n_needles = this->n_needles;
    helper->starts = this->starts;
    helper->goals = this->goals;
    helper->start_position_error_relax = this->start_position_error_relax;
    helper->start_orientation_error_relax = this->start_orientation_error_relax;
    helper->goal_distance_error_relax = this->goal_distance_error_relax;
    helper->Ts = this->Ts;

    if (helper->Ts.front() == 1) {
      helper->trust_box_size = .01;
    }
    if (this->deviation > 0.01) {
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
    helper->ConfigureProblem(*prob);
    OptimizerT opt(prob);
    helper->ConfigureOptimizer(opt);

    helper->SetSolutions(sol, opt);
    //if (initial.size() == helper->n_needles) {
    //  helper->SetSolutions(initial, opt);
    //}

    //if (!this->is_first_needle_run) {
    //  helper->IntegrateControls(opt.x());
    //}

    if (this->stage_plotting || this->stage_result_plotting) {
      this->plotter.reset(new Needle::TrajPlotter());
    }
    if (this->stage_plotting) {
      opt.addCallback(boost::bind(&Needle::TrajPlotter::OptimizerCallback, boost::ref(this->plotter), _1, _2, helper));
    }

    opt.optimize();

    this->x = opt.x();

    if (this->stage_result_plotting) {
      this->plotter->OptimizerCallback(prob.get(), this->x, this->helper);
    }
    
    return helper->GetSolutions(opt);
  }

  Vector6d NeedleProblemPlanner::PerturbState(const Vector6d& state) {
    Vector6d ret = state;//se4Down(expUp(state));
    for (int i = 0; i < 3; ++i) {
      ret(i) += normal() * 0.05;
    }
    for (int i = 3; i < 6; ++i) {
      ret(i) += normal() * 0.025;
    }
    return ret;
    //cout << "transformed matrix: " << endl << se4Up(se4Down(expUp(state))) << endl;
    //cout << "transformed matrix should be: " << endl << expUp(state) << endl;
    //cout << "transformed 2: " << se4Down(se4Up(state)).transpose() << endl;
    //cout << "transformed 1: " << logDown(expUp(state)).transpose() << endl;
    //cout << "transformed: " << logDown(se4Up(se4Down(expUp(state)))).transpose() << endl;
    //cout << "original: " << state.transpose() << endl;
    //return logDown(se4Up(ret));
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

    simulated_needle_trajectories.back().push_back(new_state);
    
    this->starts.front() = new_state;

    TrajArray traj(2, 6); traj.row(0) = state_to_change; traj.row(1) = new_state;
    vector<ConfigurationPtr> rads; rads.push_back(helper->pis.front()->local_configs[0]); rads.push_back(helper->pis.front()->local_configs[1]); 
    vector<Collision> collisions;

    CollisionChecker::GetOrCreate(*this->env)->ContinuousCheckTrajectory(traj, rads, collisions);
    BOOST_FOREACH(const Collision& collision, collisions) {
      cout << "distance: " << collision.distance << endl;
    }
    //boost::shared_ptr<BulletCollisionChecker> cc = boost::dynamic_pointer_cast<BulletCollisionChecker>();
    //cc->ContinuousCheckShape(

    if (Ts.front() > 1) {
      --Ts.front();
      this->is_first_needle_run = false;
    } else {
      // get rid of first needle
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
