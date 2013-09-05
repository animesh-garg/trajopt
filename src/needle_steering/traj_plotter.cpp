#include "needle_steering.hpp"

namespace Needle {

  TrajPlotter::TrajPlotter() {}

  void TrajPlotter::OptimizerCallback(OptProb* prob, DblVec& x, NeedleProblemHelperPtr helper) {
    vector<GraphHandlePtr> handles;
    OSGViewerPtr viewer = OSGViewer::GetOrCreate(helper->pis[0]->local_configs[0]->GetEnv());
    //BOOST_FOREACH(CostPtr& cost, prob->getCosts()) {
    //  if (Plotter* plotter = dynamic_cast<Plotter*>(cost.get())) {
    //    plotter->Plot(x, *(helper->local_configs[0]->GetEnv()), handles);
    //  }
    //}
    //vector<ConstraintPtr> constraints = prob->getConstraints();
    //BOOST_FOREACH(ConstraintPtr& cnt, constraints) {
    //  if (Plotter* plotter = dynamic_cast<Plotter*>(cnt.get())) {
    //    plotter->Plot(x, *(helper->local_configs[0]->GetEnv()), handles);
    //  }
    //}
    EnvironmentBasePtr env = helper->pis[0]->local_configs[0]->GetEnv();
    CollisionChecker::GetOrCreate(*env)->PlotCollisionGeometry(handles);//SetContactDistance(collision_dist_pen + 0.05);
    for (int k = 0; k < helper->pis.size(); ++k) {
      vector<KinBodyPtr> bodies = helper->pis[k]->local_configs[0]->GetBodies();
      MatrixXd vals = getTraj(x, helper->pis[k]->twistvars);
      for (int i=0; i < vals.rows(); ++i) {
        helper->pis[k]->local_configs[i]->SetDOFValues(toDblVec(vals.row(i)));
        BOOST_FOREACH(const KinBodyPtr& body, bodies) {
          handles.push_back(viewer->PlotKinBody(body));
          SetTransparency(handles.back(), .35);
        }
      }
    }
    viewer->Idle();
  }

  NeedleSimPlotter::NeedleSimPlotter() {}

  void NeedleSimPlotter::Plot(NeedleProblemPlannerPtr planner) {
    KinBodyPtr robot = planner->env->ReadRobotURI(RobotBasePtr(), planner->robot_file_path);
    planner->env->Add(robot, true);
    vector<GraphHandlePtr> handles;
    OSGViewerPtr viewer = OSGViewer::GetOrCreate(planner->env);
    viewer->UpdateSceneData();
    viewer->SetAllTransparency(planner->env_transparency);
    for (int i = 0; i < planner->simulated_needle_trajectories.size(); ++i) {
      for (int j = 0; j < planner->simulated_needle_trajectories[i].size(); ++j) {
        robot->SetTransform(matrixToTransform(expUp(planner->simulated_needle_trajectories[i][j])));
        handles.push_back(viewer->PlotKinBody(robot));
        SetTransparency(handles.back(), 1);
      }
    }
    planner->env->Remove(robot);
    viewer->Idle();
  }
}
