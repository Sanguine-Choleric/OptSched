#ifndef GenericTarget
#define GenericTarget

#include "opt-sched/Scheduler/OptSchedTarget.h"
#include "opt-sched/Scheduler/config.h"
#include "opt-sched/Scheduler/defines.h"
#include "opt-sched/Scheduler/machine_model.h"
#include "OptSchedDDGWrapperBasic.h"
#include "OptSchedMachineWrapper.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/ScheduleDAGInstrs.h"
#include <memory>

namespace llvm {
namespace opt_sched {

class OptSchedGenericTarget : public OptSchedTarget {
public:
  std::unique_ptr<OptSchedMachineModel>
  createMachineModel(const char *ConfigPath) override {
    return std::make_unique<OptSchedMachineModel>(ConfigPath);
  }

  std::unique_ptr<OptSchedDDGWrapperBase>
  createDDGWrapper(llvm::MachineSchedContext *Context, ScheduleDAGOptSched *DAG,
                   OptSchedMachineModel *MM, LATENCY_PRECISION LatencyPrecision,
                   const std::string &RegionID, const int NumSolvers) override {
    return std::make_unique<OptSchedDDGWrapperBasic>(
        Context, DAG, MM, LatencyPrecision, RegionID, NumSolvers);
  }

  void initRegion(llvm::ScheduleDAGInstrs *DAG, MachineModel *MM_,
   	          Config &OccFile) override {
    MM = MM_;
  }
  void finalizeRegion(const InstSchedule *Schedule) override {}
  // For generic target find total PRP.
  InstCount getCost(const llvm::SmallVectorImpl<unsigned> &PRP) const override;
};

}
}


#endif // GenericTarget