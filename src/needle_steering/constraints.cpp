#include "needle_steering.hpp"
#include <math.h>

namespace Needle {

  InsertionRegionPositionError::InsertionRegionPositionError(LocalConfigurationPtr cfg, const Vector6d& target_pos, const Vector3d& position_error_relax, double orientation_error_relax, NeedleProblemHelperPtr helper) : cfg(cfg), target_pose(expUp(target_pos)), position_error_relax(position_error_relax), orientation_error_relax(orientation_error_relax), body(cfg->GetBodies()[0]), helper(helper) {}

  ExactPositionError::ExactPositionError(LocalConfigurationPtr cfg, const Vector6d& target_pos, NeedleProblemHelperPtr helper) : cfg(cfg), target_pose(expUp(target_pos)), body(cfg->GetBodies()[0]), helper(helper) {}

  BallPositionError::BallPositionError(LocalConfigurationPtr cfg, const Vector6d& target_pos, double distance_error_relax, NeedleProblemHelperPtr helper) : cfg(cfg), target_pose(expUp(target_pos)), body(cfg->GetBodies()[0]), distance_error_relax(distance_error_relax), helper(helper) {}

  double cosangle(VectorXd a, VectorXd b) {
    if (a.norm() < 1e-6 || b.norm() < 1e-6) {
      return 1;
    } else {
      return a.dot(b) / (a.norm() * b.norm());
    }
  }

  VectorXd InsertionRegionPositionError::operator()(const VectorXd& a) const {
    assert(a.size() == 6);
    
    Matrix4d current_pose = cfg->pose * expUp(a);
    Vector3d current_rot = rotVec(current_pose.topLeftCorner<3, 3>());
    Vector3d target_rot = rotVec(target_pose.topLeftCorner<3, 3>());

    double orientation_error = fmax(cosangle(current_rot, target_rot) - cos(this->orientation_error_relax), 0);
    Vector3d position_error = (current_pose.block<3, 1>(0, 3) - target_pose.block<3, 1>(0, 3)).array().abs();
    position_error = (position_error - this->position_error_relax).cwiseMax(Vector3d::Zero());

    Vector4d err;
    err.head<3>() = position_error;
    err(3) = orientation_error;
    return err;
  }

  VectorXd ExactPositionError::operator()(const VectorXd& a) const {
    assert(a.size() == 6);
    Matrix4d current_pose = cfg->pose * expUp(a);
    return logDown(current_pose.inverse() * target_pose);
  }

  VectorXd BallPositionError::operator()(const VectorXd& a) const {
    assert(a.size() == 6);
    Matrix4d current_pose = cfg->pose * expUp(a);
    Vector3d orientation_error = logDown(current_pose.inverse() * target_pose).tail<3>();
    double distance_error = (current_pose.block<3, 1>(0, 3) - target_pose.block<3, 1>(0, 3)).norm();
    distance_error = fmax(distance_error - this->distance_error_relax, 0);
    //cout << "distance error: " << distance_error << endl;

    Vector4d err;
    err(0) = distance_error;
    err.tail<3>() = orientation_error;
    return err;
  }

  PoseError::PoseError(LocalConfigurationPtr cfg0, LocalConfigurationPtr cfg1, NeedleProblemHelperPtr helper) : cfg0(cfg0), cfg1(cfg1), helper(helper) {}

  VectorXd PoseError::operator()(const VectorXd& a) const {
    Matrix4d pose1 = cfg0->pose * expUp(a.topRows(6));
    Matrix4d pose2 = cfg1->pose * expUp(a.middleRows(6,6));
    double Delta = a(12);
    double curvature_or_radius;
    switch (helper->curvature_constraint) {
      case NeedleProblemHelper::ConstantRadius:
        curvature_or_radius = 1.0 / helper->r_min;
        break;
      case NeedleProblemHelper::BoundedRadius:
        curvature_or_radius = a(13);
        break;
      SWITCH_DEFAULT;
    }
    double theta;
    switch (helper->curvature_formulation) {
      case NeedleProblemHelper::UseCurvature:
        theta = Delta * curvature_or_radius;
        break;
      case NeedleProblemHelper::UseRadius:
        theta = Delta / curvature_or_radius;
        break;
      SWITCH_DEFAULT;
    }
    Vector6d v; v << 0, 0, Delta, theta, 0, 0;
    return logDown((pose1 * expUp(v)).inverse() * pose2);
  }


  SpeedDeviationError::SpeedDeviationError(double deviation, NeedleProblemHelperPtr helper) : deviation(deviation), helper(helper) {}

  VectorXd SpeedDeviationError::operator()(const VectorXd& a) const {
    Vector1d x;
    x[0] = sqrt((a.array() - deviation).square().sum()) - deviation * 0.5;
    return x;
  }

  ControlError::ControlError(LocalConfigurationPtr cfg0, LocalConfigurationPtr cfg1, NeedleProblemHelperPtr helper) : cfg0(cfg0), cfg1(cfg1), body(cfg0->GetBodies()[0]), helper(helper) {}

  VectorXd ControlError::operator()(const VectorXd& a) const {
    Matrix4d pose1 = cfg0->pose * expUp(a.topRows(6));
    Matrix4d pose2 = cfg1->pose * expUp(a.middleRows(6,6));
    double phi = a(12), Delta = a(13);
    double curvature_or_radius;
    switch (helper->curvature_formulation) {
      case NeedleProblemHelper::UseCurvature:
        switch (helper->curvature_constraint) {
          case NeedleProblemHelper::ConstantRadius:
            curvature_or_radius = 1.0 / helper->r_min;
            break;
          case NeedleProblemHelper::BoundedRadius:
            curvature_or_radius = a(14);
            break;
          SWITCH_DEFAULT;
        }
        break;
      case NeedleProblemHelper::UseRadius:
        switch (helper->curvature_constraint) {
          case NeedleProblemHelper::ConstantRadius:
            curvature_or_radius = helper->r_min;
            break;
          case NeedleProblemHelper::BoundedRadius:
            curvature_or_radius = a(14);
            break;
          SWITCH_DEFAULT;
        }
        break;
      SWITCH_DEFAULT;
    }
    switch (helper->formulation) {
      case NeedleProblemHelper::Form1:
      case NeedleProblemHelper::Form2: {
        return logDown(helper->TransformPose(pose1, phi, Delta, curvature_or_radius).inverse() * pose2);
      }
      SWITCH_DEFAULT;
    }
  }

  int ControlError::outputSize() const {
    switch (helper->formulation) {
      case NeedleProblemHelper::Form1:
      case NeedleProblemHelper::Form2:
        return 6;
      SWITCH_DEFAULT;
    }
  }

}
