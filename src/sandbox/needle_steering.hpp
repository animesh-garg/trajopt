#pragma once

#include "trajopt/common.hpp"
#include "sco/modeling_utils.hpp"
#include "sco/modeling.hpp"
#include <openrave-core.h>
#include <openrave/openrave.h>
#include "osgviewer/osgviewer.hpp"

using namespace trajopt;
using namespace std;
using namespace OpenRAVE;
using namespace util;
using namespace Eigen;

namespace Needle {

  class SpeedCost : public Cost {
  public:
    SpeedCost(const Var& var, double coeff);
    virtual double value(const vector<double>& xvec);
    virtual ConvexObjectivePtr convex(const vector<double>& xvec, Model* model);
  private:
    Var var_;
    double coeff_;
    AffExpr expr_;
  };

  class RotationCost : public Cost {
  public:
    RotationCost(const VarVector& vars, double coeff);
    virtual double value(const vector<double>& xvec);
    virtual ConvexObjectivePtr convex(const vector<double>& xvec, Model* model);
  private:
    VarVector vars_;
    double coeff_;
    QuadExpr expr_;
  };

  struct LocalConfiguration : public Configuration {
    KinBodyPtr body_;
    Matrix4d pose_;
    LocalConfiguration(KinBodyPtr body, const Matrix4d& pose);
    LocalConfiguration(KinBodyPtr body);
    virtual void SetDOFValues(const DblVec& dofs);
    virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const;
    virtual DblVec GetDOFValues();
    virtual int GetDOF() const;
    virtual OpenRAVE::EnvironmentBasePtr GetEnv();
    virtual DblMatrix PositionJacobian(int link_ind, const OpenRAVE::Vector& pt) const;
    virtual DblMatrix RotationJacobian(int link_ind) const;
    virtual bool DoesAffect(const KinBody::Link& link);
    virtual std::vector<KinBody::LinkPtr> GetAffectedLinks();
    virtual void GetAffectedLinks(std::vector<KinBody::LinkPtr>& links,
        bool only_with_geom, vector<int>& link_inds);
    virtual DblVec RandomDOFValues();
    virtual vector<OpenRAVE::KinBodyPtr> GetBodies();
  };

  typedef boost::shared_ptr<LocalConfiguration> LocalConfigurationPtr;

  struct PositionError : public VectorOfVector {
    LocalConfigurationPtr cfg;
    KinBodyPtr body;
    VectorXd target_pos;
    PositionError(LocalConfigurationPtr cfg, const VectorXd& target_pos);
    VectorXd operator()(const VectorXd& a) const;
  };

  struct ControlError : public VectorOfVector {
    LocalConfigurationPtr cfg0, cfg1;
    double r_min;
    KinBodyPtr body;
    int formulation;
    int strategy;
    int curvature_constraint;
    ControlError(LocalConfigurationPtr cfg0, LocalConfigurationPtr cfg1, double r_min, int formulation, int strategy, int curvature_constraint);
    VectorXd operator()(const VectorXd& a) const;
    int outputSize() const;
  };

  struct NeedleProblemHelper {
    // Formulation flag
    enum Formulation { Form1 = 1, Form2 = 2 };
    enum Strategy { StopAndTurn = 1, ConstantTwist = 2 };
    enum CurvatureConstraint { ConstantRadius = 1, BoundedRadius = 2 };
    // Config parameters
    VectorXd start;
    VectorXd goal;
    double coeff_rotation;
    double coeff_speed;
    int T;
    int n_dof;
    int formulation;
    int strategy;
    int curvature_constraint;
    double r_min;
    vector<string> ignored_kinbody_names;
    double collision_dist_pen;
    double collision_coeff;
    double Delta_lb;
    // Variables
    VarArray twistvars;
    VarArray phivars;
    VarArray radiusvars;
    Var Delta;
    // Local configurations
    vector<LocalConfigurationPtr> local_configs;

    void ConfigureProblem(const KinBodyPtr robot, OptProb& prob);
    void InitOptimizeVariables(BasicTrustRegionSQP& opt);
    void OptimizerCallback(OptProb*, DblVec& x);
    void ConfigureOptimizer(BasicTrustRegionSQP& opt);
    void CreateVariables(OptProb& prob);
    void InitLocalConfigurations(const KinBodyPtr robot, OptProb& prob);
    void InitTrajectory(OptProb& prob);
    void AddStartConstraint(OptProb& prob);
    void AddGoalConstraint(OptProb& prob);
    void AddControlConstraint(OptProb& prob);
    void AddCollisionConstraint(OptProb& prob);
  };

  struct TrajPlotter {
    vector<LocalConfigurationPtr> local_configs;
    VarArray vars;
    OSGViewerPtr viewer;
    TrajPlotter(const vector<LocalConfigurationPtr>& local_configs, const VarArray& vars);
    void OptimizerCallback(OptProb*, DblVec& x);
  };
}
