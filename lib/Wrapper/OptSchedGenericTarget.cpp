//===- OptSchedGenericTarget.cpp - Generic Target -------------------------===//
//
// Implements a generic target stub.
//
//===----------------------------------------------------------------------===//
#include "opt-sched/Scheduler/OptSchedTarget.h"
#include "opt-sched/Scheduler/config.h"
#include "opt-sched/Scheduler/defines.h"
#include "opt-sched/Scheduler/machine_model.h"
#include "OptSchedDDGWrapperBasic.h"
#include "OptSchedMachineWrapper.h"
#include "OptSchedGenericTarget.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/ScheduleDAGInstrs.h"
#include <memory>

using namespace llvm;
using namespace llvm::opt_sched;


InstCount OptSchedGenericTarget::getCost(
    const llvm::SmallVectorImpl<unsigned> &PRP) const {
  InstCount TotalPRP = 0;
  for (int16_t T = 0; T < MM->GetRegTypeCnt(); ++T)
    TotalPRP += PRP[T];
  return TotalPRP;
}


