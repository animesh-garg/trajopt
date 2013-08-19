#pragma once
#include <string>
#include "modeling.hpp"
#include <boost/function.hpp>
/*
 * Algorithms for non-convex, constrained optimization
 */

namespace sco {

using std::string;
using std::vector;


enum OptStatus {
  OPT_CONVERGED,
  OPT_SCO_ITERATION_LIMIT, // hit iteration limit before convergence
  OPT_PENALTY_ITERATION_LIMIT,
  OPT_FAILED,
  OPT_ALPHA_ITERATION_LIMIT,
  OPT_INFEASIBLE,
  INVALID
};
static const char* OptStatus_strings[]  = {
  "CONVERGED",
  "SCO_ITERATION_LIMIT",
  "PENALTY_ITERATION_LIMIT",
  "FAILED",
  "ALPHA_ITERATION_LIMIT",
  "INFEASIBLE",
  "INVALID"
};
inline string statusToString(OptStatus status) {
  return OptStatus_strings[status];
}


struct OptResults {
  DblVec x; // solution estimate
  OptStatus status;
  double total_cost;
  vector<double> cost_vals;
  DblVec cnt_viols;
  int n_func_evals, n_qp_solves, n_lp_solves;
  int n_iters;
  int n_merit_increases;
  void clear() {
    x.clear();
    status = INVALID;
    cost_vals.clear();
    cnt_viols.clear();
    n_func_evals = 0;
    n_qp_solves = 0;
    n_lp_solves = 0;
    n_merit_increases = 0;
  }
  OptResults() {clear();}
};
std::ostream& operator<<(std::ostream& o, const OptResults& r);

class Optimizer {
  /*
   * Solves an optimization problem
   */
public:
  virtual OptStatus optimize() = 0;
  virtual ~Optimizer() {}
  virtual void setProblem(OptProbPtr prob) {prob_ = prob;}
  void initialize(const vector<double>& x);
  vector<double>& x() {return results_.x;}
  OptResults& results() {return results_;}

  typedef boost::function<void(OptProb*, DblVec&)> Callback;
  void addCallback(const Callback& f); // called before each iteration
protected:
  vector<Callback> callbacks_;
  void callCallbacks(DblVec& x);
  OptProbPtr prob_;
  OptResults results_;
};

class BasicTrustRegionSQP : public Optimizer {
  /*
   * Alternates between convexifying objectives and constraints and then solving convex subproblem
   * Uses a merit function to decide whether or not to accept the step
   * merit function = objective + merit_err_coeff * | constraint_error |
   * Note: sometimes the convexified objectives and constraints lead to an infeasible subproblem
   * In that case, you should turn them into penalties and solve that problem
   * (todo: implement penalty-based sqp that gracefully handles infeasible constraints)
   */
public:
  double improve_ratio_threshold_, // minimum ratio true_improve/approx_improve to accept step
         min_trust_box_size_, // if trust region gets any smaller, exit and report convergence
         min_approx_improve_, // if model improves less than this, exit and report convergence
         min_approx_improve_frac_, // if model improves less than this, exit and report convergence
         max_iter_,
         trust_shrink_ratio_, // if improvement is less than improve_ratio_threshold, shrink trust region by this ratio
         trust_expand_ratio_, // see above
         cnt_tolerance_, // after convergence of penalty subproblem, if constraint violation is less than this, we're done
         merit_coeff_increase_ratio_, // ratio that we increate coeff each time
         max_time_ // not yet implemented
         ;
  int    max_merit_coeff_increases_; // number of times that we jack up penalty coefficient
  double merit_error_coeff_, // initial penalty coefficient
         trust_box_size_ // current size of trust region (component-wise)
         ;
  bool record_trust_region_history_;
  vector< vector<double> > trust_region_size_history;
  vector< vector<double> > log_trust_region_size_history;

  BasicTrustRegionSQP();
  BasicTrustRegionSQP(OptProbPtr prob);
  void setProblem(OptProbPtr prob);
  bool hasViolation(const DblVec& cnt_viols);
  OptStatus optimize();
  ModelPtr getModel() { return model_; }
protected:
  void adjustTrustRegion(double ratio);
  void setTrustBoxConstraints(const vector<double>& x, Model* model);
  void initParameters();
  ModelPtr model_;
};

class LineSearchSQP : public BasicTrustRegionSQP {
public:

  double trust_shrink_ratio_;
  double trust_expand_ratio_;
  double cnt_tolerance_;
  double merit_coeff_increase_ratio_;
  double merit_error_coeff_;
  double trust_box_size_;

  int max_per_merit_iter_;
  int max_merit_coeff_increases_;

  double min_cnt_improve_ratio;
  double min_model_merit_improve_ratio_;
  double line_search_shrink_ratio_;
  double min_line_search_size_;
  double min_merit_improve_ratio;
  double trust_region_shrink_threshold_;
  double trust_region_expand_threshold_;
  double min_trust_box_size_;
  double max_trust_box_size_;
  double opt_eps;

	LineSearchSQP();
	LineSearchSQP(OptProbPtr prob);
  bool hasViolation(const DblVec& cnt_viols);
  void unsetTrustBoxConstraints(Model* model);
  void initParameters();
	OptStatus optimize();
};

DblVec evaluateCosts(vector<CostPtr>& costs, const DblVec& x, Model* model);
DblVec evaluateConstraintViols(vector<ConstraintPtr>& constraints, const DblVec& x, Model* model);
vector<ConvexObjectivePtr> convexifyCosts(vector<CostPtr>& costs, const DblVec& x);
vector<ConvexConstraintsPtr> convexifyConstraints(vector<ConstraintPtr>& cnts, const DblVec& x);
DblVec evaluateModelCosts(vector<ConvexObjectivePtr>& costs, const DblVec& x, Model* model);
DblVec evaluateModelCntViols(vector<ConvexConstraintsPtr>& cnts, const DblVec& x, Model* model);
vector<string> getCostNames(const vector<CostPtr>& costs);
vector<string> getCntNames(const vector<ConstraintPtr>& cnts);
void printCostInfo(const vector<double>& old_cost_vals, const vector<double>& model_cost_vals, const vector<double>& new_cost_vals,
                  const vector<double>& old_cnt_vals, const vector<double>& model_cnt_vals, const vector<double>& new_cnt_vals,
    const vector<string>& cost_names, const vector<string>& cnt_names, double merit_coeff);
vector<ConvexObjectivePtr> cntsToCosts(const vector<ConvexConstraintsPtr>& cnts, double err_coeff);
}
