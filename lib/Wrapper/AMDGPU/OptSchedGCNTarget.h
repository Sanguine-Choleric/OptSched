#ifndef GCNTarget
#define GCNTarget

#include "Wrapper/OptSchedMachineWrapper.h"
#include "opt-sched/Scheduler/OptSchedTarget.h"
#include "opt-sched/Scheduler/config.h"
#include "opt-sched/Scheduler/data_dep.h"
#include "opt-sched/Scheduler/defines.h"
#include "opt-sched/Scheduler/machine_model.h"
#include "OptSchedDDGWrapperGCN.h"
#include "SIMachineFunctionInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/ScheduleDAGInstrs.h"
#include <algorithm>
#include <memory>


namespace llvm {
namespace opt_sched {
class OptSchedGCNTarget : public OptSchedTarget {
public:
  std::unique_ptr<OptSchedMachineModel>
  createMachineModel(const char *ConfigPath) override {
    return std::make_unique<OptSchedMachineModel>(ConfigPath);
  }

  std::unique_ptr<OptSchedDDGWrapperBase>
  createDDGWrapper(llvm::MachineSchedContext *Context, ScheduleDAGOptSched *DAG,
                   OptSchedMachineModel *MM, LATENCY_PRECISION LatencyPrecision,
                   const std::string &RegionID, const int NumSolvers) override {
    return std::make_unique<OptSchedDDGWrapperGCN>(Context, DAG, MM,
                                                    LatencyPrecision, RegionID, NumSolvers);
  }

  void initRegion(llvm::ScheduleDAGInstrs *DAG, MachineModel *MM_,
                  Config &OccFile) override;

  void finalizeRegion(const InstSchedule *Schedule) override;

  // Returns occupancy cost with number of VGPRs and SGPRs from PRP for
  // a partial or complete schedule.
  InstCount getCost(const llvm::SmallVectorImpl<unsigned> &PRP) const override;

  void dumpOccupancyInfo(const InstSchedule *Schedule) const;

  // Revert scheduing if we decrease occupancy.
  bool shouldKeepSchedule() override;

    void SetOccupancyLimit(int OccupancyLimitParam) override {
    OccupancyLimit = OccupancyLimitParam;
  }
  void SetShouldLimitOcc(bool ShouldLimitOccParam) override {
    ShouldLimitOcc = ShouldLimitOccParam;
  }
  void SetOccLimitSource(OCC_LIMIT_TYPE LimitTypeParam) override {
    LimitType = LimitTypeParam;
  }

  int getOccupancyLimit(Config &OccFile) const;

private:
  const llvm::MachineFunction *MF;
  SIMachineFunctionInfo *MFI;
  ScheduleDAGOptSched *DAG;
  const GCNSubtarget *ST;

  unsigned RegionStartingOccupancy;
  unsigned RegionEndingOccupancy;
  unsigned TargetOccupancy;

  // Limiting occupancy has shown to greatly increase the performance of some
  // kernels
  int OccupancyLimit;
  bool ShouldLimitOcc;
  OCC_LIMIT_TYPE LimitType;

  // Max occupancy with local memory size;
  unsigned MaxOccLDS;

  // In RP only (max occupancy) scheduling mode we should try to find
  // a min-RP schedule without considering perf hints which suggest limiting
  // occupancy. Returns true if we should consider perf hints.
  bool shouldLimitWaves(llvm::SIMachineFunctionInfo *MFI) const;

  // Find occupancy with spill cost.
  unsigned getOccupancyWithCost(const InstCount Cost) const;
};


}
}
#endif // GCNTarget