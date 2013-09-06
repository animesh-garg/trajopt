#pragma once

#include "common.hpp"
#include "fwd.hpp"
#include "needle_problem_planner.hpp"

namespace Needle {
  struct TrajPlotter {
    TrajPlotter();
    void OptimizerCallback(OptProb*, DblVec& x, NeedleProblemHelperPtr helper);
  };

  struct NeedleSimPlotter {
    NeedleSimPlotter();
    void Plot(NeedleProblemPlannerPtr planner);
  };
}
