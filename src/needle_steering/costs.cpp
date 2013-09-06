#include "costs.hpp"
#include "utils.hpp"
#include "needle_problem_instance.hpp"
#include "needle_problem_helper.hpp"
#include "trajopt/collision_terms.hpp"
#include "trajopt/collision_checker.hpp"

namespace Needle {

  ConstantSpeedCost::ConstantSpeedCost(const Var& var, double coeff, NeedleProblemHelperPtr helper, NeedleProblemInstancePtr pi) : Cost("Speed"), var(var), coeff(coeff), helper(helper), pi(pi) {
    assert (helper->speed_formulation == NeedleProblemHelper::ConstantSpeed);
    exprInc(expr, exprMult(var, coeff * pi->T));
  }

  double ConstantSpeedCost::value(const vector<double>& xvec, Model* model) {
    double speed = getVec(xvec, singleton<Var>(var))[0];
    return speed * coeff * pi->T;
  }

  ConvexObjectivePtr ConstantSpeedCost::convex(const vector<double>& xvec) {
    ConvexObjectivePtr out(new ConvexObjective());
    out->addAffExpr(expr);
    return out;
  }

  VariableSpeedCost::VariableSpeedCost(const VarVector& vars, double coeff, NeedleProblemHelperPtr helper) : Cost("Speed"), vars(vars), coeff(coeff), helper(helper) {
    assert (helper->speed_formulation == NeedleProblemHelper::VariableSpeed);
    for (int i = 0; i < vars.size(); ++i) {
      exprInc(expr, exprMult(vars[i], coeff));
    }
  }

  double VariableSpeedCost::value(const vector<double>& xvec, Model* model) {
    VectorXd speeds = getVec(xvec, vars);
    return coeff * speeds.array().sum();
  }

  ConvexObjectivePtr VariableSpeedCost::convex(const vector<double>& xvec) {
    ConvexObjectivePtr out(new ConvexObjective());
    out->addAffExpr(expr);
    return out;
  }

  SpeedDeviationCost::SpeedDeviationCost(const VarVector& vars, double deviation, double coeff, NeedleProblemHelperPtr helper) : Cost("Speed"), vars(vars), deviation(deviation), coeff(coeff), helper(helper) {
    assert (helper->speed_formulation == NeedleProblemHelper::VariableSpeed);
    for (int i = 0; i < vars.size(); ++i) {
      exprInc(expr, exprMult(exprSquare(exprAdd(AffExpr(vars[i]), -deviation)), coeff));
    }
  }

  double SpeedDeviationCost::value(const vector<double>& xvec, Model* model) {
    VectorXd speeds = getVec(xvec, vars);
    return coeff * (speeds.array() - deviation).square().sum();
  }

  ConvexObjectivePtr SpeedDeviationCost::convex(const vector<double>& xvec) {
    ConvexObjectivePtr out(new ConvexObjective());
    out->addQuadExpr(expr);
    return out;
  }

  RotationQuadraticCost::RotationQuadraticCost(const VarVector& vars, double coeff, NeedleProblemHelperPtr helper) : Cost("Rotation"), vars(vars), coeff(coeff), helper(helper) {
    for (int i = 0; i < vars.size(); ++i) {
      exprInc(expr, exprMult(exprSquare(vars[i]), coeff));
    }
  }

  double RotationQuadraticCost::value(const vector<double>& xvec, Model* model) {
    VectorXd vals = getVec(xvec, vars);
    return vals.array().square().sum() * coeff;
  }

  ConvexObjectivePtr RotationQuadraticCost::convex(const vector<double>& xvec) {
    ConvexObjectivePtr out(new ConvexObjective());
    out->addQuadExpr(expr);
    return out;
  }

  RotationL1Cost::RotationL1Cost(const VarVector& vars, double coeff, NeedleProblemHelperPtr helper) : Cost("Rotation"), vars(vars), coeff(coeff), helper(helper) {}

  double RotationL1Cost::value(const vector<double>& xvec, Model* model) {
    VectorXd vals = getVec(xvec, vars);
    return vals.array().abs().sum() * coeff;
  }

  ConvexObjectivePtr RotationL1Cost::convex(const vector<double>& xvec) {
    ConvexObjectivePtr out(new ConvexObjective());
    for (int i = 0; i < vars.size(); ++i) {
      out->addAbs(AffExpr(vars[i]), coeff);
    }
    return out;
  }

  NeedleCollisionClearanceCost::NeedleCollisionClearanceCost(NeedleProblemHelperPtr helper, double coeff) :
    Cost("needle_collision_clearance"),
    helper(helper),
    coeff(coeff) {
    this->ccs.clear();
    for (int i = 0; i < helper->pis.size(); ++i) {
      NeedleProblemInstancePtr pi = helper->pis[i];
      for (int j = 0; j < (int) pi->local_configs.size() - 1; ++j) {
        ccs.push_back(CollisionEvaluatorPtr(new CastCollisionEvaluator(pi->local_configs[j], pi->local_configs[j+1], pi->twistvars.row(j), pi->twistvars.row(j+1))));
      }
    }
    for (int i = 0; i < helper->pis.size(); ++i) {
      for (int j = i + 1; j < helper->pis.size(); ++j) {
        NeedleProblemInstancePtr pi0 = helper->pis[i];
        NeedleProblemInstancePtr pi1 = helper->pis[j];
        for (int t0 = 0; t0 < (int) pi0->local_configs.size() - 1; ++t0) {
          for (int t1 = 0; t1 < (int) pi1->local_configs.size() - 1; ++t1) {
            ccs.push_back(CollisionEvaluatorPtr(new CastSelfCollisionEvaluator(pi0->local_configs[t0], pi0->local_configs[t0+1], pi1->local_configs[t1], pi1->local_configs[t1+1],
                                                                               pi0->twistvars.row(t0), pi0->twistvars.row(t0+1), pi1->twistvars.row(t1), pi1->twistvars.row(t1+1))));
          }
        }
      }
    }
  }

  ConvexObjectivePtr NeedleCollisionClearanceCost::convex(const vector<double>& x) {
    ConvexObjectivePtr out(new ConvexObjective());
    vector<AffExpr> exprs;
    EnvironmentBasePtr env = helper->pis[0]->local_configs[0]->GetEnv();
    double old_contact_distance = CollisionChecker::GetOrCreate(*env)->GetContactDistance();
    CollisionChecker::GetOrCreate(*env)->SetContactDistance(helper->collision_clearance_threshold);
    BOOST_FOREACH(const CollisionEvaluatorPtr& cc, this->ccs) {
      vector<AffExpr> tmp_exprs;
      cc->CalcDistExpressions(x, tmp_exprs);
      for (int i = 0; i < tmp_exprs.size(); ++i) {
        exprs.push_back(exprMult(tmp_exprs[i], -1));
      }
    }
    CollisionChecker::GetOrCreate(*env)->SetContactDistance(old_contact_distance);
    //if (exprs.size() == 0) {
    //  throw std::runtime_error("should always have at least one result.");
    //}
    if (exprs.size() > 0) {
      out->addMax(exprs, this->coeff);
    } else {
      out->addAffExpr(AffExpr(-helper->collision_clearance_threshold * this->coeff));
    }
    return out;
  }

  double NeedleCollisionClearanceCost::value(const vector<double>& x, Model* model) {
    DblVec dists;
    EnvironmentBasePtr env = helper->pis[0]->local_configs[0]->GetEnv();
    double old_contact_distance = CollisionChecker::GetOrCreate(*env)->GetContactDistance();
    CollisionChecker::GetOrCreate(*env)->SetContactDistance(helper->collision_clearance_threshold);
    BOOST_FOREACH(const CollisionEvaluatorPtr& cc, this->ccs) {
      DblVec tmp_dists;
      cc->CalcDists(x, tmp_dists);
      for (int i = 0; i < tmp_dists.size(); ++i) {
        dists.push_back(-tmp_dists[i]);
      }
    }
    CollisionChecker::GetOrCreate(*env)->SetContactDistance(old_contact_distance);
    if (dists.size() > 0) {
      return vecMax(dists) * this->coeff;
    } else {
      return - helper->collision_clearance_threshold * this->coeff;
    }
  }
}
