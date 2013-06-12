#include "o3.hpp"
#include "sco/expr_ops.hpp"
#include "sco/modeling_utils.hpp"
#include "sco/modeling.hpp"
#include "osgviewer/osgviewer.hpp"
#include "trajopt/collision_checker.hpp"
#include "trajopt/collision_terms.hpp"
#include "trajopt/common.hpp"
#include "trajopt/plot_callback.hpp"
#include "trajopt/problem_description.hpp"
#include "trajopt/rave_utils.hpp"
#include "trajopt/trajectory_costs.hpp"
#include "utils/clock.hpp"
#include "utils/config.hpp"
#include "utils/eigen_conversions.hpp"
#include "utils/stl_to_string.hpp"
#include <boost/assign.hpp>
#include <boost/foreach.hpp>
#include <ctime>
#include <openrave-core.h>
#include <openrave/openrave.h>

using namespace trajopt;
using namespace std;
using namespace OpenRAVE;
using namespace util;
using namespace boost::assign;
using namespace Eigen;


namespace Needle {

  template<typename T, size_t N>
  T* end(T (&ra)[N]) {
    return ra + N;
  }


  void AddVarArrays(OptProb& prob, int rows, const vector<int>& cols, const vector<double>& lbs, const vector<double>& ubs, const vector<string>& name_prefix, const vector<VarArray*>& newvars) {
    int n_arr = name_prefix.size();
    assert(n_arr == newvars.size());

    vector<MatrixXi> index(n_arr);
    for (int i=0; i < n_arr; ++i) {
      newvars[i]->resize(rows, cols[i]);
      index[i].resize(rows, cols[i]);
    }

    vector<string> names;
    vector<double> all_lbs;
    vector<double> all_ubs;
    int var_idx = prob.getNumVars();
    for (int i=0; i < rows; ++i) {
      for (int k=0; k < n_arr; ++k) {
        for (int j=0; j < cols[k]; ++j) {
          index[k](i,j) = var_idx;
          names.push_back( (boost::format("%s_%i_%i")%name_prefix[k]%i%j).str() );
          all_lbs.push_back(lbs[k]);
          all_ubs.push_back(ubs[k]);
          ++var_idx;
        }
      }
    }
    prob.createVariables(names, all_lbs, all_ubs); // note that w,r, are both unbounded

    const vector<Var>& vars = prob.getVars();
    for (int k=0; k < n_arr; ++k) {
      for (int i=0; i < rows; ++i) {
        for (int j=0; j < cols[k]; ++j) {
          (*newvars[k])(i,j) = vars[index[k](i,j)];
        }
      }
    }
  }

  void AddVarArrays(OptProb& prob, int rows, const vector<int>& cols, const vector<string>& name_prefix, const vector<VarArray*>& newvars) {
    vector<double> lbs(newvars.size(), -INFINITY);
    vector<double> ubs(newvars.size(), INFINITY);
    AddVarArrays(prob, rows, cols, lbs, ubs, name_prefix, newvars);
  }

  void AddVarArray(OptProb& prob, int rows, int cols, double lb, double ub, const string& name_prefix, VarArray& newvars) {
    vector<VarArray*> arrs(1, &newvars);
    vector<string> prefixes(1, name_prefix);
    vector<int> colss(1, cols);
    vector<double> lbs(1, lb);
    vector<double> ubs(1, ub);
    AddVarArrays(prob, rows, colss, lbs, ubs, prefixes, arrs);
  }

  void AddVarArray(OptProb& prob, int rows, int cols, const string& name_prefix, VarArray& newvars) {
    AddVarArray(prob, rows, cols, -INFINITY, INFINITY, name_prefix, newvars);
  }

  Matrix3d rotMat(const Vector3d& x) {
    Matrix3d out;
    out << 0, -x(2), x(1),
           x(2), 0, -x(0),
           -x(1), x(0), 0;
    return out;
  }

  Matrix3d expRot(const Vector3d& x) {
    double rr = x.squaredNorm();
    if (fabs(rr) < 1e-10) {
      return Matrix3d::Identity();
    } else {
      double r = sqrt(rr);
      return rotMat(x * (sin(r) / r)) + Matrix3d::Identity() * cos(r) + (x*x.transpose()) * ((1 - cos(r)) / rr);
    }
  }

  Vector3d logRot(const Matrix3d& X) {
    Vector3d x;
    x << X(2, 1) - X(1, 2),
         X(0, 2) - X(2, 0),
         X(1, 0) - X(0, 1);
    double r = x.norm();
    double t = X(0, 0) + X(1, 1) + X(2, 2) - 1;

    if (r == 0) {
      return Vector3d::Zero();
    } else {
      return x * (atan2(r, t) / r);
    }
  }

  Matrix4d expUp(const VectorXd& x) {
    assert(x.size() == 6);
    Matrix4d X = Matrix4d::Identity();
    X.block<3, 3>(0, 0) = expRot(x.tail<3>());
    X.block<3, 1>(0, 3) = x.head<3>();
    return X;
  }

  VectorXd logDown(const Matrix4d& X) {
    VectorXd x(6);
    x.head<3>() = X.block<3, 1>(0, 3);
    x.tail<3>() = logRot(X.block<3, 3>(0, 0));
    return x;
  }

  OpenRAVE::Transform matrixToTransform(const Matrix4d& X) {
    OpenRAVE::TransformMatrix M;
    M.trans.x = X(0, 3);
    M.trans.y = X(1, 3);
    M.trans.z = X(2, 3);
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        M.m[row*4+col] = X(row, col);
      }
    }
    return OpenRAVE::Transform(M);
  }

  OpenRAVE::Transform vecToTransform(const VectorXd& x) {
    OpenRAVE::Transform T;
    OpenRAVE::Vector trans(x[0], x[1], x[2]);
    OpenRAVE::Vector rot(x[3], x[4], x[5]);
    T.trans = trans;
    T.rot = OpenRAVE::geometry::quatFromAxisAngle(rot);
    return T;
  }

  class SpeedCost : public Cost {
  public:
    SpeedCost(const Var& var, double coeff) : Cost("Speed"), var_(var), coeff_(coeff) {
      exprInc(expr_, exprMult(var, coeff));
    }
    virtual double value(const vector<double>& xvec) {
      double speed = getVec(xvec, singleton<Var>(var_))[0];
      return speed * coeff_;
    }
    virtual ConvexObjectivePtr convex(const vector<double>& xvec, Model* model) {
      ConvexObjectivePtr out(new ConvexObjective(model));
      out->addAffExpr(expr_);
      return out;
    }
  private:
    Var var_;
    double coeff_;
    AffExpr expr_;
  };

  class RotationCost : public Cost {
  public:
    RotationCost(const VarVector& vars, double coeff) : Cost("Rotation"), vars_(vars), coeff_(coeff) {
      for (int i = 0; i < vars.size(); ++i) {
        exprInc(expr_, exprMult(exprSquare(vars[i]), coeff));
      }
    }
    virtual double value(const vector<double>& xvec) {
      VectorXd vals = getVec(xvec, vars_);
      return vals.array().square().sum() * coeff_;
    }
    virtual ConvexObjectivePtr convex(const vector<double>& xvec, Model* model) {
      ConvexObjectivePtr out(new ConvexObjective(model));
      out->addQuadExpr(expr_);
      return out;
    }
  private:
    VarVector vars_;
    double coeff_;
    QuadExpr expr_;
  };

  struct LocalConfiguration : public Configuration {
    KinBodyPtr body_;
    Matrix4d pose_;

    LocalConfiguration(KinBodyPtr body, const Matrix4d& pose) :
      body_(body), pose_(pose) {}
      
    LocalConfiguration(KinBodyPtr body) :
      body_(body) {}

    virtual void SetDOFValues(const DblVec& dofs) {
      VectorXd x(dofs.size());
      for (int i = 0; i < dofs.size(); ++i) {
        x[i] = dofs[i];
      }
      OpenRAVE::Transform T = matrixToTransform(pose_ * expUp(x));
      body_->SetTransform(T);
    }

    virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const {
      lower = DblVec(6, -INFINITY);
      upper = DblVec(6, INFINITY);
    }

    virtual DblVec GetDOFValues() {
      DblVec out(6);
      OpenRAVE::Transform T = body_->GetTransform();
      out[0] = T.trans.x;
      out[1] = T.trans.y;
      out[2] = T.trans.z;
      OpenRAVE::Vector rot = OpenRAVE::geometry::axisAngleFromQuat(T.rot);
      out[3] = rot.x;
      out[4] = rot.y;
      out[5] = rot.z;
      return out;
    }

    virtual int GetDOF() const {
      return 6;
    }
    virtual OpenRAVE::EnvironmentBasePtr GetEnv() {
      return body_->GetEnv();
    }

    virtual DblMatrix PositionJacobian(int link_ind, const OpenRAVE::Vector& pt) const {
      PRINT_AND_THROW("not implemented");
    }
    virtual DblMatrix RotationJacobian(int link_ind) const {
      PRINT_AND_THROW("not implemented");
    }
    virtual bool DoesAffect(const KinBody::Link& link) {
      const vector<KinBody::LinkPtr>& links = body_->GetLinks();
      for (int i=0; i < links.size(); ++i) {
        if (links[i].get() == &link) return true;
      }
      return false;
    }

    virtual std::vector<KinBody::LinkPtr> GetAffectedLinks() {
      return body_->GetLinks();
    }

    virtual void GetAffectedLinks(std::vector<KinBody::LinkPtr>& links,
        bool only_with_geom, vector<int>& link_inds) {
      links = GetAffectedLinks();
      link_inds.resize(links.size());
      for (int i = 0; i < links.size(); ++i)
        link_inds.push_back(links[i]->GetIndex());
    }

    virtual DblVec RandomDOFValues() {
      return toDblVec(VectorXd::Random(6));
    }
    virtual vector<OpenRAVE::KinBodyPtr> GetBodies() {
      return singleton(body_);
    }
  };

  typedef boost::shared_ptr<LocalConfiguration> LocalConfigurationPtr;

  struct PositionError : public VectorOfVector {
    LocalConfigurationPtr cfg;
    KinBodyPtr body;
    VectorXd target_pos;
    PositionError(LocalConfigurationPtr cfg, const VectorXd& target_pos) : cfg(cfg), target_pos(target_pos), body(cfg->GetBodies()[0]) {}
    VectorXd operator()(const VectorXd& a) const {
      return logDown(cfg->pose_ * expUp(a)) - target_pos;
    }
  };

  struct ControlError : public VectorOfVector {

    LocalConfigurationPtr cfg0, cfg1;
    double r_min;
    KinBodyPtr body;
    ControlError(LocalConfigurationPtr cfg0, LocalConfigurationPtr cfg1, double r_min) : cfg0(cfg0), cfg1(cfg1), r_min(r_min), body(cfg0->GetBodies()[0]) {}
    VectorXd operator()(const VectorXd& a) const {
      Matrix4d pose1 = cfg0->pose_ * expUp(a.topRows(6));
      Matrix4d pose2 = cfg1->pose_ * expUp(a.middleRows(6,6));
      double phi = a(12), Delta = a(13);
      VectorXd w(6); w << 0, 0, 0, 0, 0, phi;
      VectorXd v(6); v << 0, 0, Delta, Delta / r_min, 0, 0;
      //VectorXd w(6); w << 0, 0, 0, phi, 0, 0;
      //VectorXd v(6); v << Delta, 0, 0, 0, Delta / r_min, 0;
      return logDown((pose1 * expUp(w) * expUp(v)).inverse() * pose2);
    }
  };

  struct NeedleProblemHelper {
    // Config parameters
    VectorXd start;
    VectorXd goal;
    double coeff_rotation;
    double coeff_speed;
    int T;
    int n_dof;
    double r_min;
    vector<string> ignored_kinbody_names;
    double collision_dist_pen;
    double collision_coeff;
    double Delta_lb;
    // Variables
    VarArray twistvars;
    VarArray phivars;
    Var Delta;
    // Local configurations
    vector<LocalConfigurationPtr> local_configs;

    void ConfigureProblem(const KinBodyPtr robot, OptProb& prob) {
      CreateVariables(prob);
      InitLocalConfigurations(robot, prob);
      InitTrajectory(prob);
      prob.addCost(CostPtr(new RotationCost(phivars.col(0), coeff_rotation)));
      prob.addCost(CostPtr(new SpeedCost(Delta, coeff_speed)));
      AddStartConstraint(prob);
      AddGoalConstraint(prob);
      AddControlConstraint(prob);
      AddCollisionConstraint(prob);
    }

    void InitOptimizeVariables(BasicTrustRegionSQP& opt) {
      DblVec initVec;
      // Initialize twistvars
      for (int i = 0; i <= T; ++i) {
        for (int j = 0; j < n_dof; ++j) {
          initVec.push_back(0.);
        }
      }
      // Initialize phivars
      for (int i = 0; i < T; ++i) {
        initVec.push_back(0.);
      }
      // Initialize Delta
      initVec.push_back(Delta_lb);
      opt.initialize(initVec);
    }

    void OptimizerCallback(OptProb*, DblVec& x) {
      MatrixXd twistvals = getTraj(x, twistvars);
      for (int i = 0; i < local_configs.size(); ++i) {
        local_configs[i]->pose_ = local_configs[i]->pose_ * expUp(twistvals.row(i));
      }
      setVec(x, twistvars.m_data, DblVec(twistvars.size(), 0));
    }

    void ConfigureOptimizer(BasicTrustRegionSQP& opt) {
      InitOptimizeVariables(opt);
      opt.addCallback(boost::bind(&Needle::NeedleProblemHelper::OptimizerCallback, this, _1, _2));
    }

    void CreateVariables(OptProb& prob) {
      // Time frame varies from 0 to T instead of from 0 to T-1
      AddVarArray(prob, T+1, n_dof, "twist", twistvars);
      AddVarArray(prob, T, 1, -PI, PI, "phi", phivars);
      Delta_lb = (goal.topRows(3) - start.topRows(3)).norm() / T / r_min;
      Delta = prob.createVariables(singleton<string>("Delta"), singleton<double>(Delta_lb),singleton<double>(INFINITY))[0];
      // Only the twist variables are incremental (i.e. their trust regions should be around zero)
      prob.setIncremental(twistvars.flatten());
    }

    void InitLocalConfigurations(const KinBodyPtr robot, OptProb& prob) {
      for (int i = 0; i <= T; ++i) {
        local_configs.push_back(LocalConfigurationPtr(new LocalConfiguration(robot)));
      }
    }

    void InitTrajectory(OptProb& prob) {
      MatrixXd initTraj(T+1, n_dof);
      for (int idof = 0; idof < n_dof; ++idof) {
        initTraj.col(idof) = VectorXd::LinSpaced(T+1, start[idof], goal[idof]);
      }
      for (int i = 0; i <= T; ++i) {
        local_configs[i]->pose_ = expUp(initTraj.row(i));
      }
    }

    void AddStartConstraint(OptProb& prob) {
      VarVector vars = twistvars.row(0);
      VectorOfVectorPtr f(new Needle::PositionError(local_configs[0], start));
      VectorXd coeffs = VectorXd::Ones(6);
      prob.addConstraint(ConstraintPtr(new ConstraintFromFunc(f, vars, coeffs, EQ, "entry")));
    }

    void AddGoalConstraint(OptProb& prob) {
      VarVector vars = twistvars.row(T);
      VectorOfVectorPtr f(new Needle::PositionError(local_configs[T], goal));
      VectorXd coeffs(n_dof); coeffs << 1., 1., 1., 0., 0., 0.;// = VectorXd::Ones(6);
      prob.addConstraint(ConstraintPtr(new ConstraintFromFunc(f, vars, coeffs, EQ, "goal")));
    }

    void AddControlConstraint(OptProb& prob) {
      for (int i = 0; i < T; ++i) {
        VarVector vars = concat(concat(twistvars.row(i), twistvars.row(i+1)), phivars.row(i));
        vars.push_back(Delta);
        VectorOfVectorPtr f(new Needle::ControlError(local_configs[i], local_configs[i+1], r_min));
        VectorXd coeffs = VectorXd::Ones(6);
        prob.addConstraint(ConstraintPtr(new ConstraintFromFunc(f, vars, coeffs, EQ, (boost::format("control%i")%i).str())));
      }
    }

    void AddCollisionConstraint(OptProb& prob) {
      // TODO

    }

  };

  struct TrajPlotter {
    vector<LocalConfigurationPtr> local_configs;
    VarArray vars;
    OSGViewerPtr viewer;
    TrajPlotter(const vector<LocalConfigurationPtr>& local_configs, const VarArray& vars) : local_configs(local_configs), vars(vars) {
      viewer = OSGViewer::GetOrCreate(local_configs[0]->GetEnv());
    }
    void OptimizerCallback(OptProb*, DblVec& x) {
      vector<GraphHandlePtr> handles;
      vector<KinBodyPtr> bodies = local_configs[0]->GetBodies();
      MatrixXd vals = getTraj(x, vars);
      for (int i=0; i < vals.rows(); ++i) {
        local_configs[i]->SetDOFValues(toDblVec(vals.row(i)));
        BOOST_FOREACH(const KinBodyPtr& body, bodies) {
          handles.push_back(viewer->PlotKinBody(body));
          SetTransparency(handles.back(), .35);
        }
      }
      viewer->Idle();
    }
  };
}

int main(int argc, char** argv)
{
  bool plotting=true, verbose=false;
  double env_transparency = 0.5;

  int T = 10;
  double r_min = 2;
  int n_dof = 6;

  double improve_ratio_threshold = 0.25;
  double trust_shrink_ratio = 0.7;
  double trust_expand_ratio = 1.2;
  
  double start_vec_array[] = {0, 0, 0, 0, 0, 0};//-12.82092, 6.80976, 0.06844, 0, 0, 0};
  double goal_vec_array[] = {9, 0, 0, 0, 0, 0};//-3.21932, 6.87362, -1.21877, 0, 0, 0};

  vector<double> start_vec(start_vec_array, start_vec_array + n_dof);
  vector<double> goal_vec(goal_vec_array, goal_vec_array + n_dof);

  {
    Config config;
    config.add(new Parameter<bool>("plotting", &plotting, "plotting"));
    config.add(new Parameter<bool>("verbose", &verbose, "verbose"));
    config.add(new Parameter<double>("env_transparency", &env_transparency, "env_transparency"));
    config.add(new Parameter<int>("T", &T, "T"));
    config.add(new Parameter<double>("r_min", &r_min, "r_min"));
    config.add(new Parameter<double>("improve_ratio_threshold", &improve_ratio_threshold, "improve_ratio_threshold"));
    config.add(new Parameter<double>("trust_shrink_ratio", &trust_shrink_ratio, "trust_shrink_ratio"));
    config.add(new Parameter<double>("trust_expand_ratio", &trust_expand_ratio, "trust_expand_ratio"));
    config.add(new Parameter< vector<double> >("s", &start_vec, "s"));
    config.add(new Parameter< vector<double> >("g", &goal_vec, "g"));
    CommandParser parser(config);
    parser.read(argc, argv);
  }

  

  double coeff_rotation = 1.;
  double coeff_speed = 1.;

  RaveInitialize(false, verbose ? Level_Debug : Level_Info);
  EnvironmentBasePtr env = RaveCreateEnvironment();
  env->StopSimulation();
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(env);
  assert(viewer);

  env->Load(string(DATA_DIR) + "/prostate.env.xml");//needleprob.env.xml");
  viewer->SetAllTransparency(env_transparency);
  RobotBasePtr robot = GetRobot(*env);

  VectorXd start(n_dof); for (int i = 0; i < n_dof; ++i) start[i] = start_vec[i];
  VectorXd goal(n_dof); for (int i = 0; i < n_dof; ++i) goal[i] = goal_vec[i];

  const char *ignored_kinbody_c_strs[] = { "KinBodyProstate", "KinBodyDermis", "KinBodyEpidermis", "KinBodyHypodermis" };
  vector<string> ignored_kinbody_names(ignored_kinbody_c_strs, Needle::end(ignored_kinbody_c_strs));

  OptProbPtr prob(new OptProb());

  Needle::NeedleProblemHelper helper;
  helper.start = start;
  helper.goal = goal;
  helper.coeff_rotation = coeff_rotation;
  helper.coeff_speed = coeff_speed;
  helper.T = T;
  helper.r_min = r_min;
  helper.n_dof = n_dof;
  helper.ignored_kinbody_names = ignored_kinbody_names;
  helper.collision_dist_pen = 0.025;
  helper.collision_coeff = 20;
  helper.ConfigureProblem(robot, *prob);

  BasicTrustRegionSQP opt(prob);
  opt.max_iter_ = 500;    
  opt.improve_ratio_threshold_ = improve_ratio_threshold;
  opt.trust_shrink_ratio_ = trust_shrink_ratio;
  opt.trust_expand_ratio_ = trust_expand_ratio;

  helper.ConfigureOptimizer(opt);

  boost::shared_ptr<Needle::TrajPlotter> plotter;
  if (plotting) {
    plotter.reset(new Needle::TrajPlotter(helper.local_configs, helper.twistvars));
    opt.addCallback(boost::bind(&Needle::TrajPlotter::OptimizerCallback, boost::ref(plotter), _1, _2));
  }

  opt.optimize();

  RaveDestroy();


}
