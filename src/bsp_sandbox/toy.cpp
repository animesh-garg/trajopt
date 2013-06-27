#include "bsp/bsp.hpp"
#include "toy.hpp"
#include <QApplication>
#include <QtCore>

using namespace BSP;

namespace ToyBSP {

  ToyBSPProblemHelper::ToyBSPProblemHelper() : BSPProblemHelper<ToyBeliefFunc>() {
    input_dt = 1.0;
    set_state_dim(2);
    set_sigma_dof(3);
    set_observe_dim(2);
    set_control_dim(2);
    set_state_bounds(DblVec(2, -10), DblVec(2, 10));
    set_control_bounds(DblVec(2, -0.9), DblVec(2, 0.9));
    set_variance_cost(VarianceT::Identity(state_dim, state_dim));
    set_final_variance_cost(VarianceT::Identity(state_dim, state_dim) * 40);
    set_control_cost(ControlCostT::Identity(control_dim, control_dim));
  }

  void ToyBSPProblemHelper::init_control_values(vector<ControlT>* output_init_controls) const {
    assert (output_init_controls != NULL);
    ControlT control_step;
    control_step.resize(control_dim);
    control_step(0) = (goal(0) - start(0)) / T;
    control_step(1) = (goal(1) - start(1)) / T;
    output_init_controls->clear();
    for (int i = 0; i < T; ++i) {
      output_init_controls->push_back(control_step);
    }
  }

  ToyStateFunc::ToyStateFunc() : StateFunc<StateT, ControlT, StateNoiseT>() {}

  ToyStateFunc::ToyStateFunc(BSPProblemHelperBasePtr helper) :
    StateFunc<StateT, ControlT, StateNoiseT>(helper), toy_helper(boost::static_pointer_cast<ToyBSPProblemHelper>(helper)) {}

  StateT ToyStateFunc::operator()(const StateT& x, const ControlT& u, const StateNoiseT& m) const {
    StateT new_x(state_dim);
    new_x(0) = x(0) + u(0)*toy_helper->input_dt + sqrt(0.01*u(0)*u(0)*toy_helper->input_dt + 0.001) * m(0);
    new_x(1) = x(1) + u(1)*toy_helper->input_dt + sqrt(0.01*u(1)*u(1)*toy_helper->input_dt + 0.001) * m(1);
    return new_x;
  }

  ToyObserveFunc::ToyObserveFunc() : ObserveFunc<StateT, ObserveT, ObserveNoiseT>() {}

  ToyObserveFunc::ToyObserveFunc(BSPProblemHelperBasePtr helper) :
    ObserveFunc<StateT, ObserveT, ObserveNoiseT>(helper), toy_helper(boost::static_pointer_cast<ToyBSPProblemHelper>(helper)) {}

  ObserveT ToyObserveFunc::operator()(const StateT& x, const ObserveNoiseT& n) const {
    return compute_gamma(x, -1) * x + 0.1 * n;
  }

  ObserveT ToyObserveFunc::operator()(const StateT& x, const ObserveNoiseT& n, double approx_factor) const {
    return compute_gamma(x, approx_factor) * x + 0.1 * n;
  }

  bool ToyObserveFunc::sgndist(const Vector2d& x, Vector2d* dists) const {
    Vector2d p1; p1 << 0, 2;
    Vector2d p2; p2 << 0, 0;
    (*dists)(0) = (x - p1).norm() - 0.5;
    (*dists)(1) = (x - p2).norm() - 0.5;
    return (*dists)(0) < 0 || (*dists)(1) < 0;
  }

  ObserveMatT ToyObserveFunc::compute_gamma(const StateT& x, double approx_factor) const {
    double tol = 0.1;
    Vector2d dists;
    sgndist(x.head<2>(), &dists);
    double gamma1, gamma2;
    if (approx_factor < 0) {
      gamma1 = dists(0) <= 0 ? 1 : 0;
      gamma2 = dists(1) <= 0 ? 1 : 0;
    } else {
      gamma1 = 1. - (1./(1.+exp(-approx_factor*(dists(0)+tol))));
      gamma2 = 1. - (1./(1.+exp(-approx_factor*(dists(1)+tol))));
    }
    ObserveMatT gamma(observe_dim, observe_dim);
    gamma << gamma1, 0,
             0, gamma2;
    return gamma;
  }

  ToyBeliefFunc::ToyBeliefFunc() : BeliefFunc<ToyStateFunc, ToyObserveFunc, BeliefT>() {
    set_approx_factor(0.5);
  }

  ToyBeliefFunc::ToyBeliefFunc(BSPProblemHelperBasePtr helper, StateFuncPtr f, ObserveFuncPtr h) :
    BeliefFunc<ToyStateFunc, ToyObserveFunc, BeliefT>(helper, f, h), toy_helper(boost::static_pointer_cast<ToyBSPProblemHelper>(helper)) {
    set_approx_factor(0.5);
  }

  ToyPlotter::ToyPlotter(double x_min, double x_max, double y_min, double y_max, BSPProblemHelperBasePtr helper, QWidget* parent) :
   BSPQtPlotter(x_min, x_max, y_min, y_max, helper, parent),
   ProblemState(helper),
   toy_helper(boost::static_pointer_cast<ToyBSPProblemHelper>(helper)),
   old_approx_factor(-1), cur_approx_factor(-1), distmap(400, 400, QImage::Format_RGB32) {}

  void ToyPlotter::paintEvent(QPaintEvent* ) {
    QPainter painter(this);
    if (cur_approx_factor != old_approx_factor || distmap.height() != height() || distmap.width() != width()) {
      // replot distmap
      distmap = QImage(width(), height(), QImage::Format_RGB32);
      for (int j = 0; j < height(); ++j) {
        QRgb *line = (QRgb*) distmap.scanLine(j);
        for (int i = 0; i < width(); ++i) {
          double x = unscale_x(i),
                 y = unscale_y(j);
          Vector2d dists;
          Vector2d pos; pos << x, y;
          toy_helper->belief_func->h->sgndist(pos, &dists);
          double grayscale = fmax(1./(1. + exp(toy_helper->belief_func->approx_factor*dists(0))),
                                  1./(1. + exp(toy_helper->belief_func->approx_factor*dists(1))));
          line[i] = qRgb(grayscale*255, grayscale*255, grayscale*255);
        }
      }
    }
    painter.drawImage(0, 0, distmap);
    QPen cvx_cov_pen(Qt::red, 2, Qt::SolidLine);
    QPen path_pen(Qt::red, 2, Qt::SolidLine);
    QPen pos_pen(Qt::red, 8, Qt::SolidLine);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::HighQualityAntialiasing);
    painter.setPen(cvx_cov_pen);
    for (int i = 0; i < states.size(); ++i) {
      draw_ellipse(states[i], sigmas[i], painter, 0.5);
    }
    painter.setPen(path_pen);
    for (int i = 0; i < states.size() - 1; ++i) {
      draw_line(states[i](0), states[i](1), states[i+1](0), states[i+1](1), painter);
    }
    painter.setPen(pos_pen);
    for (int i = 0; i < states.size(); ++i) {
      draw_point(states[i](0), states[i](1), painter);
    }
  }

  void ToyPlotter::update_plot_data(OptProb*, DblVec& xvec) {
    vector<VectorXd> new_states;
    vector<MatrixXd> new_sigmas;
    old_approx_factor = cur_approx_factor;
    cur_approx_factor = toy_helper->belief_func->approx_factor;
    BeliefT cur_belief;
    toy_helper->belief_func->compose_belief(toy_helper->start, toy_helper->start_sigma, &cur_belief);
    for (int i = 0; i <= T; ++i) {
      StateT cur_state;
      VarianceT cur_sigma;
      toy_helper->belief_func->extract_state(cur_belief, &cur_state);
      toy_helper->belief_func->extract_sigma(cur_belief, &cur_sigma);
      new_states.push_back(cur_state);
      new_sigmas.push_back(cur_sigma);
      if (i < T) cur_belief = toy_helper->belief_func->call(cur_belief, (ControlT) getVec(xvec, toy_helper->control_vars.row(i)));
    }
    states = new_states;
    sigmas = new_sigmas;
    this->repaint();
  }

  ToyOptimizerTask::ToyOptimizerTask(QObject* parent) : BSPOptimizerTask(parent) {}

  ToyOptimizerTask::ToyOptimizerTask(int argc, char **argv, QObject* parent) : BSPOptimizerTask(argc, argv, parent) {}

  template< class BSPProblemHelperT >
  class BSPPlanner {
  public:
    typedef BSPProblemHelperT T;
    //typedef typename BSPProblemHelperT::StateT StateT;
    //typedef typename BSPProblemHelperT::ControlT ControlT;
    //typedef typename BSPProblemHelperT::StateNoiseT StateNoiseT;
    //typedef typename BSPProblemHelperT::StateGradT StateGradT;
    //typedef typename BSPProblemHelperT::ControlGradT ControlGradT;
    //typedef typename BSPProblemHelperT::StateNoiseGradT StateNoiseGradT;
    //typedef typename BSPProblemHelperT::ObserveStateT ObserveStateT;
    //typedef typename BSPProblemHelperT::ObserveT ObserveT;
    //typedef typename BSPProblemHelperT::ObserveNoiseT ObserveNoiseT;
    //typedef typename BSPProblemHelperT::ObserveStateGradT ObserveStateGradT;
    //typedef typename BSPProblemHelperT::ObserveNoiseGradT ObserveNoiseGradT;
    //typedef typename BSPProblemHelperT::BeliefT BeliefT;
    //typedef typename BSPProblemHelperT::BeliefGradT BeliefGradT;
    //typedef typename BSPProblemHelperT::BeliefControlGradT BeliefControlGradT;
    //typedef typename BSPProblemHelperT::VarianceT VarianceT;
    //typedef typename BSPProblemHelperT::StateFuncT StateFuncT;
    //typedef typename BSPProblemHelperT::ObserveFuncT ObserveFuncT;
    //typedef typename BSPProblemHelperT::StateFuncPtr StateFuncPtr;
    //typedef typename BSPProblemHelperT::ObserveFuncPtr ObserveFuncPtr;
    //typedef typename BSPProblemHelperT::BeliefFuncPtr BeliefFuncPtr;
    T::StateT start;
    T::VarianceT start_sigma;
    T::StateT goal;
    int T;

    BSPPlanne

    void initialize() {

    }
  }
  class BSPPlanner

  class ToyBSPPlanner : public BSPPlanner<ToyBSPProblemHelper> {
  public:
    
  };

  typedef boost::shared_ptr<ToyBSPPlanner> ToyBSPPlannerPtr;

  void ToyOptimizerTask::emit_plot_message(OptProb* prob, DblVec& xvec) {
    BSPOptimizerTask::emit_plot_message(prob, xvec);
  }

  void ToyOptimizerTask::run() {
    int T = 20;
    bool plotting = false;
    double start_vec_array[] = {-5, 2};
    double goal_vec_array[] = {-5, 0};
    vector<double> start_vec(start_vec_array, end(start_vec_array));
    vector<double> goal_vec(goal_vec_array, end(goal_vec_array));
    {
      Config config;
      config.add(new Parameter<bool>("plotting", &plotting, "plotting"));
      config.add(new Parameter< vector<double> >("s", &start_vec, "s"));
      config.add(new Parameter< vector<double> >("g", &goal_vec, "g"));
      CommandParser parser(config);
      parser.read(argc, argv, true);
    }
    Vector2d start = toVectorXd(start_vec);
    Vector2d goal = toVectorXd(goal_vec);
    Matrix2d start_sigma = Matrix2d::Identity();

    //OptProbPtr prob(new OptProb());

    //ToyBSPProblemHelperPtr helper(new ToyBSPProblemHelper());
    //helper->start = start;
    //helper->goal = goal;
    //helper->start_sigma = start_sigma;
    //helper->T = T;
    //helper->set_initial_controls()

    ToyBSPPlannerPtr planner(new ToyBSPPlanner());

    planner->start = start;
    planner->goal = goal;
    planner->start_sigma = start_sigma;
    planner->T = T;
    planner->initialize();

    boost::shared_ptr<ToyPlotter> plotter;
    if (plotting) {
      double x_min = -7, x_max = 2, y_min = -3, y_max = 5;
      plotter.reset(planner->create_plotter(x_min, x_max, y_min, y_max));
    }

    while (!planner.finished()) {
      planner.solve();
      // simulate one step
      planner.simulate_execution(1);
      if (plotting) {
        planner.plot_simulation(plotter);
      }
    }

    if (!plotting) {
      emit finished_signal();
    }
      
    start = ...
    goal = ...
    start_sigma = ...
    T = ...
    while (!replanner.finished()) {
      replanner.solve 
      replanner.simulate_execution
      replanner.plot_simulation

    Replanner.solve
    Replanner.simulate_execution(1)

    helper->configure_problem(*prob);

    BSPTrustRegionSQP opt(prob);
    opt.max_iter_ = 500;
    opt.merit_error_coeff_ = 500;
    opt.trust_shrink_ratio_ = 0.5;
    opt.trust_expand_ratio_ = 1.25;
    opt.min_trust_box_size_ = 1e-3;
    opt.min_approx_improve_ = 1e-2;
    opt.min_approx_improve_frac_ = 1e-4;
    opt.improve_ratio_threshold_ = 0.2;
    opt.trust_box_size_ = 1;

    helper->configure_optimizer(*prob, opt);

    boost::shared_ptr<ToyPlotter> plotter;
    if (plotting) {
      double x_min = -7, x_max = 2, y_min = -3, y_max = 5;
      plotter.reset(create_plotter<ToyPlotter>(x_min, x_max, y_min, y_max, helper));
      plotter->show();
      opt.addCallback(boost::bind(&ToyOptimizerTask::emit_plot_message, boost::ref(this), _1, _2));
    }
    opt.optimize();
    if (!plotting) {
      emit finished_signal();
    }
  }
}

using namespace ToyBSP;

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  ToyOptimizerTask* task = new ToyOptimizerTask(argc, argv, &app);
  QTimer::singleShot(0, task, SLOT(run_slot()));
  QObject::connect(task, SIGNAL(finished_signal()), &app, SLOT(quit()));
  return app.exec();
}
