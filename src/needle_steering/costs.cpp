#include "needle_steering.hpp"

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
      exprInc(expr, exprMult(vars[i], coeff));//exprSquare(exprAdd(AffExpr(vars[i]), -deviation)), coeff));
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
    helper(helper),
    coeff(coeff) {
    this->cast_ccs.clear();
    this->cast_self_ccs.clear();
    for (int i = 0; i < helper->pis.size(); ++i) {
      NeedleProblemInstancePtr pi = helper->pis[i];
      for (int j = 0; j < (int) pi->local_configs.size() - 1; ++j) {
        cast_ccs.push_back(new CastCollisionEvaluator(pi->local_configs[j], pi->local_configs[j+1], pi->twistvars.row(j), pi->twistvars.row(j+1)));
      }
    }
    for (int i = 0; i < helper->pis.size(); ++i) {
      for (int j = i + 1; j < helper->pis.size(); ++j) {
        NeedleProblemInstancePtr pi0 = helper->pis[i];
        NeedleProblemInstancePtr pi1 = helper->pis[j];
        for (int t0 = 0; t0 < (int) pi0->local_configs.size() - 1; ++t0) {
          for (int t1 = 0; t1 < (int) pi1->local_configs.size() - 1; ++t1) {
            cast_self_ccs.push_back(new CastSelfCollisionEvaluator(pi0->local_configs[t0], pi0->local_configs[t0+1], pi1->local_configs[t1], pi1->local_configs[t1+1]));
          }
        }
      }
    }
  }

  ConvexObjectivePtr NeedleCollisionClearanceCost::convex(const vector<double>& x) {
    ConvexObjectivePtr out(new ConvexObjective());
    vector<AffExpr> exprs;
    BOOST_FOREACH(const CastCollisionEvaluatorPtr& cc, this->cast_ccs) {
      vector<AffExpr> tmp_exprs;
      cc->CalcDistExpressions(x, tmp_exprs);
      exprs.insert(exprs.end(), tmp_exprs.begin(), tmp_exprs.end());
    }
    BOOST_FOREACH(const CastSelfCollisionEvaluatorPtr& cc, this->cast_ccs) {
      vector<AffExpr> tmp_exprs;
      cc->CalcDistExpressions(x, tmp_exprs);
      exprs.insert(exprs.end(), tmp_exprs.begin(), tmp_exprs.end());
    }
    out->addMax(exprs);
  }
}
