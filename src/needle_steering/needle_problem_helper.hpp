#pragma once

#include "common.hpp"
#include "fwd.hpp"
#include "needle_problem_instance.hpp"

namespace Needle {
  struct NeedleProblemHelper : public boost::enable_shared_from_this<NeedleProblemHelper> {
    // Formulation flag
    enum Formulation { Form1 = 1, Form2 = 2 };
    enum CurvatureConstraint { ConstantRadius = 1, BoundedRadius = 2 };
    enum Method { Colocation = 1, Shooting = 2 };
    enum CurvatureFormulation { UseRadius = 1, UseCurvature = 2 };
    enum SpeedFormulation { ConstantSpeed = 1, VariableSpeed = 2 };
    enum RotationCost { UseRotationQuadraticCost = 1, UseRotationL1Cost = 2 };
    // Config parameters
    vector<Vector6d> starts;
    vector<Vector6d> goals;
    int n_needles;
    double coeff_rotation;
    double coeff_rotation_regularization;
    double coeff_speed;
    double coeff_orientation_error;
    double improve_ratio_threshold;
    double trust_shrink_ratio;
    double trust_expand_ratio;
    double merit_error_coeff;
    int max_merit_coeff_increases;
    double trust_box_size;
    bool record_trust_region_history;
    vector<int> Ts;
    int n_dof;
    int formulation;
    int curvature_constraint;
    int speed_formulation;
    int method;
    int curvature_formulation;
    int rotation_cost;
    bool use_speed_deviation_constraint;
    bool use_speed_deviation_cost;
    bool verbose;
    bool explicit_controls;
    bool continuous_collision;
    bool control_constraints;
    bool goal_orientation_constraint;
    double env_transparency;
    double r_min;
    vector<string> ignored_kinbody_names;
    double collision_dist_pen;
    double collision_coeff;
    double collision_clearance_coeff;
    double collision_clearance_threshold;
    vector<KinBodyPtr> robots;

    vector<NeedleProblemInstancePtr> pis;
    vector<ConstraintPtr> self_collision_constraints;

    vector<Vector3d> start_position_error_relax;
    vector<double> start_orientation_error_relax;
    vector<double> goal_distance_error_relax;

    void ConfigureProblem(OptProb& prob);
    void InitOptimizeVariables(OptimizerT& opt);
    void OptimizerCallback(OptProb*, DblVec& x);
    void ConfigureOptimizer(OptimizerT& opt);
    vector<VectorXd> GetSolutionsWithoutFirstTimestep(const vector<VectorXd>& sol);



    void InitParameters();
    void InitParametersFromConsole(int argc, char** argv);
    void Clear();

    void CreateVariables(OptProb& prob, NeedleProblemInstancePtr pi);
    void InitLocalConfigurations(const KinBodyPtr robot, OptProb& prob, NeedleProblemInstancePtr pi);
    void InitTrajectory(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddRotationCost(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddSpeedCost(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddSpeedConstraint(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddStartConstraint(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddGoalConstraint(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddControlConstraint(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddPoseConstraint(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddCollisionConstraint(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddSelfCollisionConstraint(OptProb& prob, NeedleProblemInstancePtr piA, NeedleProblemInstancePtr piB);
    void AddCollisionClearanceCost(OptProb& prob);
    //void AddCollisionCost(OptProb& prob, NeedleProblemInstancePtr pi);
    void InitializeCollisionEnvironment();

    Matrix4d TransformPose(const Matrix4d& pose, double phi, double Delta, double radius) const;
    double GetPhi(const DblVec& x, int i, NeedleProblemInstancePtr pi) const;
    double GetDelta(const DblVec& x, int i, NeedleProblemInstancePtr pi) const;
    double GetCurvatureOrRadius(const DblVec& x, int i, NeedleProblemInstancePtr pi) const;
    double GetCurvature(const DblVec& x, int i, NeedleProblemInstancePtr pi) const;
    double GetRadius(const DblVec& x, int i, NeedleProblemInstancePtr pi) const;

    vector<VectorXd> GetSolutions(OptimizerT& opt);
    void SetSolutions(const vector<VectorXd>& sol, OptimizerT& opt);

    void AddNeedlesToBullet(OptimizerT& prob);
    void AddNeedleToBullet(NeedleProblemInstancePtr pi, OptimizerT& prob);

    #ifdef NEEDLE_TEST
    void checkAlignment(DblVec& x);
    #endif
  };

}
