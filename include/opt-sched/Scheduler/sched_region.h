/*******************************************************************************
Description:  Implements an abstract base class for representing scheduling
              regions.
Author:       Ghassan Shobaki
Created:      Apr. 2005
Updated By:   Ciprian Elies and Vang Thao
Last Update:  Jan. 2020
*******************************************************************************/

#ifndef OPTSCHED_SCHED_REGION_SCHED_REGION_H
#define OPTSCHED_SCHED_REGION_SCHED_REGION_H

#include "opt-sched/Scheduler/defines.h"
#include "opt-sched/Scheduler/lnkd_lst.h"
#include "opt-sched/Scheduler/sched_basic_data.h"
// For DataDepGraph, LB_ALG.
#include "opt-sched/Scheduler/data_dep.h"
// For Enumerator, LengthCostEnumerator, EnumTreeNode and Pruning.
#include "opt-sched/Scheduler/enumerator.h"
#include "opt-sched/Scheduler/gen_sched.h"
#include "llvm/CodeGen/MachineFunction.h"


namespace llvm {
namespace opt_sched {

// How to compare cost.
// TODO(max): Elaborate.
enum COST_COMP_MODE {
  // Dynamically.
  CCM_DYNMC,
  // Statically.
  CCM_STTC
};

// (Chris)
enum class BLOCKS_TO_KEEP {
  ZERO_COST,
  IMPROVED,
  OPTIMAL,
  IMPROVED_OR_OPTIMAL,
  ALL
};

class ListScheduler;

class SchedRegion {
public:
  // TODO(max): Document.
  SchedRegion(MachineModel *machMdl, DataDepGraph *dataDepGraph, long rgnNum,
              int16_t sigHashSize, LB_ALG lbAlg, SchedPriorities hurstcPrirts,
              SchedPriorities enumPrirts, bool vrfySched,
              Pruning PruningStrategy, SchedulerType HeurSchedType,
              SmallVector<MemAlloc<EnumTreeNode> *, 16> &EnumNodeAllocs,
              SmallVector<MemAlloc<CostHistEnumTreeNode> *, 16> &HistNodeAllocs, 
               SmallVector<MemAlloc<BinHashTblEntry<HistEnumTreeNode>> *, 16> &HashTablAllocs,
              SPILL_COST_FUNCTION spillCostFunc = SCF_PERP);
  // Destroys the region. Must be overriden by child classes.
  virtual ~SchedRegion() {delete OptimalSolverID_;}

  // Returns the dependence graph of this region.
  inline DataDepGraph *GetDepGraph() { return dataDepGraph_; }
  // Returns the heuristic cost for this region.
  inline InstCount GetHeuristicCost() { return hurstcCost_; }
  // Returns a pointer to the list scheduler heurisitcs.
  inline SchedPriorities GetHeuristicPriorities() { return hurstcPrirts_; }
  // Get the number of simulated spills code added for this block.
  inline int GetSimSpills() { return totalSimSpills_; }

  // TODO(max): Document.
  virtual FUNC_RESULT
  FindOptimalSchedule(Milliseconds rgnTimeout, Milliseconds lngthTimeout,
                      bool &isHurstcOptml, InstCount &bestCost,
                      InstCount &bestSchedLngth, InstCount &hurstcCost,
                      InstCount &hurstcSchedLngth, InstSchedule *&bestSched,
                      bool filterByPerp, const BLOCKS_TO_KEEP blocksToKeep, 
                      bool isParallelBB);

  // External abstract functions.

  // TODO(max): Document.
  virtual int cmputCostLwrBound() = 0;
  // TODO(max): Document.
  virtual InstCount UpdtOptmlSched(InstSchedule *crntSched,
                                   LengthCostEnumerator *enumrtr) = 0;
  // TODO(max): Document.
  //virtual bool chkCostFsblty(InstCount trgtLngth, EnumTreeNode *treeNode) = 0;
  // TODO(max): Document.
  virtual void SchdulInst(SchedInstruction *inst, InstCount cycleNum,
                          InstCount slotNum, bool trackCnflcts) = 0;
  // TODO(max): Document.
  virtual void UnschdulInst(SchedInstruction *inst, InstCount cycleNum,
                            InstCount slotNum, EnumTreeNode *trgtNode) = 0;
  // TODO(max): Document.
  //virtual void SetSttcLwrBounds(EnumTreeNode *node) = 0;

  // Do region-specific checking for the legality of scheduling the
  // given instruction in the current issue slot
  virtual bool ChkInstLglty(SchedInstruction *inst) = 0;

  virtual void InitForSchdulng() = 0;

  virtual bool ChkSchedule_(InstSchedule *bestSched,
                            InstSchedule *lstSched) = 0;

  // TODO(max): Document.
  InstSchedule *AllocNewSched_();

  SPILL_COST_FUNCTION GetSpillCostFunc();

  inline void setSpillCostFunc(SPILL_COST_FUNCTION scf) {spillCostFunc_ = scf;}

  // Initialize variables for the second pass of the two-pass-optsched
  void InitSecondPass();

  // (Chris): The cost function. Defaults to PERP.
  SPILL_COST_FUNCTION spillCostFunc_ = SCF_PERP;


private:
  // The algorithm to use for calculated lower bounds.
  LB_ALG lbAlg_;
  // The number of this region.
  long rgnNum_;
  // Whether to verify the schedule after calculating it.
  bool vrfySched_;

  // The normal heuristic scheduling results.
  InstCount hurstcCost_;

  // total simulated spills.
  int totalSimSpills_;

  // What list scheduler should be used to find an initial feasible schedule.
  SchedulerType HeurSchedType_;

  


  // list scheduling heuristics
  SchedPriorities hurstcPrirts_;
  // Scheduling heuristics to use when enumerating
  SchedPriorities enumPrirts_;

  // TODO(max): Document.
  int16_t sigHashSize_;

  // The pruning technique to use for this region.
  Pruning prune_;

  // The ID of the solver that solved the region optimally
  int *OptimalSolverID_;

  // Whether to dump the DDGs for the blocks we schedule
  bool DumpDDGs_;
  // Where to dump the DDGs
  std::string DDGDumpPath_;

protected:
  // Best cost so far
  InstCount bestCost_;
  // BEst Schedule Length so far.
  InstCount bestSchedLngth_;
  // The dependence graph of this region.
  DataDepGraph *dataDepGraph_;
  // The machine model used by this region.
  MachineModel *machMdl_;

  // The schedule currently used by the enumerator
  InstSchedule *enumCrntSched_;
  // The best schedule found by the enumerator so far
  InstSchedule *enumBestSched_;
  // The best schedule found so far (may be heuristic or enumerator generated)
  InstSchedule *bestSched_;

  // TODO(max): Document.
  InstCount schedLwrBound_;
  // TODO(max): Document.
  InstCount schedUprBound_;
  // TODO(max): Document.
  InstCount abslutSchedUprBound_;

  // TODO(max): Document.
  InstCount crntCycleNum_;
  // TODO(max): Document.
  InstCount crntSlotNum_;

  // Used for two-pass-optsched to enable second pass functionalies.
  bool isSecondPass_;

  bool IsTimeoutPerInst_ = true;

  // The absolute cost lower bound to be used as a ref for normalized costs.
  InstCount costLwrBound_ = 0;

  SmallVector<MemAlloc<EnumTreeNode> *, 16> EnumNodeAllocs_;
  SmallVector<MemAlloc<CostHistEnumTreeNode> *, 16> HistNodeAllocs_;
  SmallVector<MemAlloc<BinHashTblEntry<HistEnumTreeNode>> *, 16> HashTablAllocs_;

  // protected accessors:
  SchedulerType GetHeuristicSchedulerType() const { return HeurSchedType_; }

  void SetBestSchedLength(InstCount bestSchedLngth) {
    bestSchedLngth_ = bestSchedLngth;
  }

  const SchedPriorities &GetEnumPriorities() const { return enumPrirts_; }

  int16_t GetSigHashSize() const { return sigHashSize_; }

  Pruning GetPruningStrategy() const { return prune_; }

  // TODO(max): Document.
  void UseFileBounds_();

  // Top-level function for enumerative scheduling
  FUNC_RESULT Optimize_(Milliseconds startTime, Milliseconds rgnTimeout,
                        Milliseconds lngthTimeout, int *OptimalSolverID);
  // TODO(max): Document.
  void CmputLwrBounds_(bool useFileBounds, int SolverID);
  // TODO(max): Document.
  bool CmputUprBounds_(InstSchedule *schedule, bool useFileBounds);
  // Handle the enumerator's result
  void HandlEnumrtrRslt_(FUNC_RESULT rslt, InstCount trgtLngth);

  // Simulate local register allocation.
  void RegAlloc_(InstSchedule *&bestSched, InstSchedule *&lstSched);

  // TODO(max): Document.
  virtual void CmputAbslutUprBound_();

  // Internal abstract functions.

  // Compute the normalized cost.
  virtual InstCount CmputNormCost_(InstSchedule *sched, COST_COMP_MODE compMode,
                                   InstCount &execCost, bool trackCnflcts) = 0;
  // TODO(max): Document.
  // virtual InstCount CmputCost_(InstSchedule *sched, COST_COMP_MODE compMode,
                               // InstCount &execCost, bool trackCnflcts) = 0;
  // TODO(max): Document.
  virtual void CmputSchedUprBound_() = 0;
  // TODO(max): Document.
  virtual Enumerator *AllocEnumrtr_(Milliseconds timeout, SmallVector<MemAlloc<EnumTreeNode> *, 16> &EnumNodeAllocs,
             SmallVector<MemAlloc<CostHistEnumTreeNode> *, 16> &HistNodeAllocs, 
             SmallVector<MemAlloc<BinHashTblEntry<HistEnumTreeNode>> *, 16> &HashTablAllocs) = 0;
  // Wrapper for the enumerator
  virtual FUNC_RESULT Enumerate_(Milliseconds startTime,
                                 Milliseconds rgnTimeout,
                                 Milliseconds lngthTimeout,
                                 int *OptimalSolverID) = 0;
  // TODO(max): Document.
  void FinishHurstc_();
  // TODO(max): Document.
  virtual void FinishOptml_() = 0;
  // TODO(max): Document.
  ConstrainedScheduler *AllocHeuristicScheduler_();

  virtual bool EnableEnum_() = 0;

  // Prepares the region for being scheduled.
  virtual void SetupForSchdulng_() = 0;

  // Gets the number of nodes examined in enumeration
  virtual uint64_t getExaminedNodeCount() = 0;

  // (Chris) Get the SLIL for each set
  // virtual const std::vector<int> &GetSLIL_() const = 0;

  FUNC_RESULT runACO(InstSchedule *ReturnSched, InstSchedule *InitSched,
                     bool IsPostBB);

  
  
};

} // namespace opt_sched
} // namespace llvm

#endif
