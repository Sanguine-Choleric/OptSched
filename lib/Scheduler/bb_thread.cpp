#include "opt-sched/Scheduler/aco.h"
#include "opt-sched/Scheduler/config.h"
#include "opt-sched/Scheduler/data_dep.h"
#include "opt-sched/Scheduler/enumerator.h"
#include "opt-sched/Scheduler/bb_thread.h"
#include "opt-sched/Scheduler/list_sched.h"
#include "opt-sched/Scheduler/logger.h"
#include "opt-sched/Scheduler/random.h"
#include "opt-sched/Scheduler/reg_alloc.h"
#include "opt-sched/Scheduler/register.h"
#include "opt-sched/Scheduler/relaxed_sched.h"
#include "opt-sched/Scheduler/stats.h"
#include "opt-sched/Scheduler/utilities.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/ErrorHandling.h"
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <mutex>
#include <unistd.h>
#include <sys/mman.h>
#include <malloc.h>
#include <atomic>


extern bool OPTSCHED_gPrintSpills;

using namespace llvm::opt_sched;

InstPool3::InstPool3() {
  pool = new LinkedList<EnumTreeNode>();
}

InstPool3::InstPool3(int size) {
  pool = new LinkedList<EnumTreeNode>();
  maxSize_ = size;
}

InstPool3::~InstPool3() {
  delete pool;
}

void InstPool3::removeSpecificElement(SchedInstruction *inst, EnumTreeNode *parent, 
                                      EnumTreeNode *&removed) {
  // if we've reached this point of the code then there must be isnts in the pool
  assert(pool->GetElmntCnt() > 0);
  LinkedListIterator<EnumTreeNode> it = pool->begin();
  bool removeElement = false;

  EnumTreeNode *temp = it.GetEntry()->element;
#ifdef IS_CORRECT_LOCALPOOL
  Logger::Info("localPool time %d, targetNode time %d", temp->GetTime(), parent->GetTime());
  if (temp->GetParent() != parent) Logger::Info("localPool nodes parent is not the target");
#endif
  while (temp->GetParent() == parent && temp->GetInstNum() != inst->GetNum()) {
#ifdef IS_CORRECT_LOCALPOOL
    Logger::Info("iterating through localPool");
#endif
    ++it;
    temp = it.GetEntry()->element;
  }

  if (temp->GetParent() == parent && temp->GetInstNum() == inst->GetNum()) {
    removeElement = true;
#ifdef IS_CORRECT_LOCALPOOL
    Logger::Info("element hit in remove specific from local pool");
#endif
    assert(temp);

    pool->RemoveAt(it, false);
  }

  removed = removeElement ? temp : nullptr;

}

InstPool4::InstPool4() {;}

InstPool4::InstPool4(int SortMethod) {
  SortMethod_ = SortMethod;
  if (SortMethod_ == 0) {
    Logger::Info("Global Pool prioritizing SpillCost");
  }
  else {
    Logger::Info("Global Pool prioritizing heuristic");
  }
}

// TODO refactor to use comparator instead of flag
void InstPool4::sort() {
  std::queue<std::shared_ptr<HalfNode>> sortedQueue;


  if (SortMethod_ == 0) {
    //sorting by cost
	  while (!pool.empty()) {
		  std::shared_ptr<HalfNode> tempNode;
      std::shared_ptr<HalfNode> tempNode2;
		  int size = pool.size();
      bool firstIter = true;
      for (int i = 0; i < size; i++) {
        tempNode2 = pool.front();
        pool.pop();
        if (firstIter) {
          tempNode = tempNode2;
          firstIter = false;
          continue;
        }
  
  
			  else {
          if ((tempNode2->getCost() < tempNode->getCost()) || 
              (tempNode2->getCost() == tempNode->getCost() 
                  && tempNode2->getHeuristic()[0] > tempNode->getHeuristic()[0])) {
            pool.push(tempNode);
            tempNode = tempNode2;
            continue;    
          }
        }

        pool.push(tempNode2);
		  }
		  sortedQueue.push(tempNode);
	  }
  }

  else {
    while (!pool.empty()) {
		  std::shared_ptr<HalfNode> tempNode;
      std::shared_ptr<HalfNode> tempNode2;
		  int size = pool.size();
      bool firstIter = true;
      for (int i = 0; i < size; i++) {
        tempNode2 = pool.front();
        pool.pop();
        if (firstIter) {
          tempNode = tempNode2;
          firstIter = false;
          continue;
        }
  
	      else {
          int i = Depth_ - 1;
          unsigned long *thisHeur = tempNode2->getHeuristic();
          unsigned long *otherHeur = tempNode->getHeuristic();
          
          for (; i >= 0; i--) {
            if (thisHeur[i] > otherHeur[i] || thisHeur[i] < otherHeur[i]) {
              break;
            }
          }

          if (thisHeur[i] < otherHeur[i]) {
            pool.push(tempNode2);
            continue;
          }

          if ((i == 0 && thisHeur[i] == otherHeur[i] && (tempNode2->getCost() < tempNode->getCost())) || 
              (thisHeur[i] > otherHeur[i])) {
                pool.push(tempNode);
                tempNode = tempNode2;
                continue;
            }
          }
        pool.push(tempNode2);
		  }
		  sortedQueue.push(tempNode);
    }
	}

  int n = sortedQueue.size();
  for (int i = 0; i < n; i++) {
    pool.push(sortedQueue.front());
    sortedQueue.pop();
  }
}


InstPool4::~InstPool4() {
  ;
}


InstPool::InstPool() {;}

InstPool::InstPool(int SortMethod) {
  SortMethod_ = SortMethod;
  if (SortMethod_ == 0) {
    Logger::Info("Global Pool prioritizing SpillCost");
  }
  else {
    Logger::Info("Global Pool prioritizing heuristic");
  }
}

InstPool::~InstPool() {
  int size = pool.size();
  for (int i = 0; i < size; i ++) {
    std::pair<EnumTreeNode *, unsigned long *> tempNode;
    tempNode = pool.front();
    pool.pop();
    delete[] tempNode.second;
  }
}

void InstPool::sort() {
  std::queue<std::pair<EnumTreeNode *, unsigned long *>> sortedQueue;


  if (SortMethod_ == 0) {
	 while (!pool.empty()) {
		  std::pair<EnumTreeNode *, unsigned long *> tempNode;
      std::pair<EnumTreeNode *, unsigned long *> tempNode2;
		  int size = pool.size();
      bool firstIter = true;
      for (int i = 0; i < size; i++) {
        tempNode2 = pool.front();
        pool.pop();
        if (firstIter) {
          tempNode = tempNode2;
          firstIter = false;
          continue;
        }
  
  
			  else {
          if ((tempNode2.first->GetCost() < tempNode.first->GetCost()) || 
              (tempNode2.first->GetCost() == tempNode.first->GetCost() 
                  && tempNode2.second > tempNode.second)) {
            pool.push(tempNode);
            tempNode = tempNode2;
            continue;    
          }
        }

        pool.push(tempNode2);
		  }
		  sortedQueue.push(tempNode);
	  }
  }

  else {
    while (!pool.empty()) {
		  std::pair<EnumTreeNode *, unsigned long *> tempNode;
      std::pair<EnumTreeNode *, unsigned long *> tempNode2;
		  int size = pool.size();
      bool firstIter = true;
      for (int i = 0; i < size; i++) {
        tempNode2 = pool.front();
        pool.pop();
        if (firstIter) {
          tempNode = tempNode2;
          firstIter = false;
          continue;
        }
  
	      else {
          int i = Depth_ - 1;
          for (; i >= 0; i--) {
            assert(tempNode2.first);
            if (tempNode2.second[i] > tempNode.second[i] || tempNode2.second[i] < tempNode.second[i]) {
              break;
            }
          }

          if (tempNode2.second[i] < tempNode.second[i]) {
            pool.push(tempNode2);
            continue;
          }

          if ((i == 0 && tempNode2.second[i] == tempNode.second[i] && (tempNode2.first->GetCost() < tempNode.first->GetCost())) || 
              (tempNode2.second[i] > tempNode.second[i])) {
                pool.push(tempNode);
                tempNode = tempNode2;
                continue;
            }
          }

        pool.push(tempNode2);
		  }
		  sortedQueue.push(tempNode);
    }
	}

  int n = sortedQueue.size();
  for (int i = 0; i < n; i++) {
    pool.push(sortedQueue.front());
    sortedQueue.pop();
  }
	//pool = sortedQueue; 
}





// The denominator used when calculating cost weight.
static const int COST_WGHT_BASE = 100;

BBThread::BBThread(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
                         long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
                         SchedPriorities hurstcPrirts,
                         SchedPriorities enumPrirts, bool vrfySched,
                         Pruning PruningStrategy, bool SchedForRPOnly,
                         bool enblStallEnum, int SCW,
                         SPILL_COST_FUNCTION spillCostFunc,
                         SchedulerType HeurSchedType)
    : OST(OST_) {
  Enumrtr_ = NULL;
  OptmlSpillCost_ = INVALID_VALUE;

  CrntCycleNum_ = INVALID_VALUE;
  CrntSlotNum_ = INVALID_VALUE;
  CrntSpillCost_ = INVALID_VALUE;

  SchedForRPOnly_ = SchedForRPOnly;

  VrfySched_ = vrfySched;

  EnblStallEnum_ = enblStallEnum;
  SCW_ = SCW;
  SchedCostFactor_ = COST_WGHT_BASE;
  TrackLiveRangeLngths_ = true;

  SimpleMachineModel_ = OST_->MM->IsSimple();
  MaxLatency_ = dataDepGraph->GetMaxLtncy();
  NumberOfInsts_ = dataDepGraph->GetInstCnt();
  IssueRate_ = OST_->MM->GetIssueRate();

  MachMdl_ = OST_->MM;
  DataDepGraph_ = dataDepGraph;
  
  EntryInstCnt_ = dataDepGraph->GetEntryInstCnt();
  ExitInstCnt_ = dataDepGraph->GetExitInstCnt();

  SpillCostFunc_ = spillCostFunc;

  RegTypeCnt_ = OST->MM->GetRegTypeCnt();
  RegFiles_ = dataDepGraph->getRegFiles();
  LiveRegs_ = new WeightedBitVector[RegTypeCnt_];
  LivePhysRegs_ = new WeightedBitVector[RegTypeCnt_];
  SpillCosts_ = new InstCount[dataDepGraph->GetInstCnt()];
  PeakRegPressures_ = new InstCount[RegTypeCnt_];
  RegPressures_.resize(RegTypeCnt_);
  SumOfLiveIntervalLengths_.resize(RegTypeCnt_, 0);

  EntryInstCnt_ = 0;
  ExitInstCnt_ = 0;
  SchduldEntryInstCnt_ = 0;
  SchduldExitInstCnt_ = 0;
  SchduldInstCnt_ = 0;
}
/****************************************************************************/

BBThread::~BBThread() {
  if (Enumrtr_ != NULL) {
    delete Enumrtr_;
  }

  delete[] LiveRegs_;
  delete[] LivePhysRegs_;
  delete[] SpillCosts_;
  delete[] PeakRegPressures_;
}
/*****************************************************************************/

void BBThread::setupPhysRegs_() {
  int physRegCnt;
  for (int i = 0; i < RegTypeCnt_; i++) {
    physRegCnt = RegFiles_[i].FindPhysRegCnt();
    if (physRegCnt > 0)
      LivePhysRegs_[i].Construct(physRegCnt);
  }
}

/*****************************************************************************/

void BBThread::initForSchdulng() {
  initForCostCmputtn_();

  SchduldEntryInstCnt_ = 0;
  SchduldExitInstCnt_ = 0;
  SchduldInstCnt_ = 0;
}
/*****************************************************************************/

void BBThread::initForCostCmputtn_() {
  int i;

  CrntCycleNum_ = 0;
  CrntSlotNum_ = 0;
  CrntSpillCost_ = 0;
  CrntStepNum_ = -1;
  PeakSpillCost_ = 0;
  TotSpillCost_ = 0;

  for (i = 0; i < RegTypeCnt_; i++) {
    RegFiles_[i].ResetCrntUseCnts(SolverID_);
    RegFiles_[i].ResetCrntLngths();
  }

  for (i = 0; i < RegTypeCnt_; i++) {
    LiveRegs_[i].Reset();
    if (RegFiles_[i].GetPhysRegCnt() > 0)
      LivePhysRegs_[i].Reset();
    //    if (chkCnflcts_)
    //      regFiles_[i].ResetConflicts();
    PeakRegPressures_[i] = 0;
    RegPressures_[i] = 0;
  }

  for (i = 0; i < NumberOfInsts_; i++)
    SpillCosts_[i] = 0;

  for (auto &i : SumOfLiveIntervalLengths_)
    i = 0;

  DynamicSlilLowerBound_ = StaticSlilLowerBound_;
}
/*****************************************************************************/

InstCount BBThread::cmputNormCost(InstSchedule *sched,
                                      COST_COMP_MODE compMode,
                                      InstCount &execCost, bool trackCnflcts) {
  InstCount cost = CmputCost_(sched, compMode, execCost, trackCnflcts);

  cost -= getCostLwrBound();
  execCost -= getCostLwrBound();

  sched->SetCost(cost);
  sched->SetExecCost(execCost);
  return cost;
}
/*****************************************************************************/

InstCount BBThread::CmputCost_(InstSchedule *sched, COST_COMP_MODE compMode,
                                  InstCount &execCost, bool trackCnflcts) {
  /*
  if (compMode == CCM_STTC) {
    if (SpillCostFunc_ == SCF_SPILLS) {
      LocalRegAlloc regAlloc(sched, dataDepGraph_);
      regAlloc.SetupForRegAlloc();
      regAlloc.AllocRegs();
      CrntSpillCost_ = regAlloc.GetCost();
    }
  }
  */

  assert(sched->IsComplete());
  InstCount cost = sched->GetCrntLngth() * SchedCostFactor_;
  execCost = cost;
  cost += CrntSpillCost_ * SCW_;
  sched->SetSpillCosts(SpillCosts_);
  sched->SetPeakRegPressures(PeakRegPressures_);
  sched->SetSpillCost(CrntSpillCost_);
  return cost;
}
/*****************************************************************************/

void BBThread::cmputCrntSpillCost_() {
  switch (SpillCostFunc_) {
  case SCF_PERP:
  case SCF_PRP:
  case SCF_PEAK_PER_TYPE:
  case SCF_TARGET:
    CrntSpillCost_ = PeakSpillCost_;
    break;
  case SCF_SUM:
    CrntSpillCost_ = TotSpillCost_;
    break;
  case SCF_PEAK_PLUS_AVG:
    CrntSpillCost_ =
        PeakSpillCost_ + TotSpillCost_ / NumberOfInsts_;
    break;
  case SCF_SLIL:
    CrntSpillCost_ = SlilSpillCost_;
    break;
  default:
    CrntSpillCost_ = PeakSpillCost_;
    break;
  }
}
/*****************************************************************************/

void BBThread::updateSpillInfoForSchdul(SchedInstruction *inst,
                                            bool trackCnflcts) {
  int16_t regType;
  int regNum, physRegNum;
  int liveRegs;
  InstCount newSpillCost;

#ifdef IS_DEBUG_REG_PRESSURE
  Logger::Info("Updating reg pressure after scheduling Inst %d",
               inst->GetNum());
#endif

  // Update Live regs after uses
  for (llvm::opt_sched::Register *use : inst->GetUses()) {
    regType = use->GetType();
    regNum = use->GetNum();
    physRegNum = use->GetPhysicalNumber();

    if (use->IsLive(SolverID_) == false)
      llvm::report_fatal_error(
          llvm::StringRef("Reg " + std::to_string(regNum) + " of type " +
                          std::to_string(regType) + " is used without being defined"), false);

#ifdef IS_DEBUG_REG_PRESSURE
    Logger::Info("Inst %d uses reg %d of type %d and %d uses", inst->GetNum(),
                 regNum, regType, use->GetUseCnt());
#endif

    use->AddCrntUse(SolverID_);

    if (use->IsLive(SolverID_) == false) {
      // (Chris): The SLIL calculation below the def and use for-loops doesn't
      // consider the last use of a register. Thus, an additional increment must
      // happen here.
      if (SpillCostFunc_ == SCF_SLIL) {
        SumOfLiveIntervalLengths_[regType]++;
        if (!use->IsInInterval(inst) && !use->IsInPossibleInterval(inst)) {
          ++DynamicSlilLowerBound_;
        }
      }

      LiveRegs_[regType].SetBit(regNum, false, use->GetWght());

#ifdef IS_DEBUG_REG_PRESSURE
      Logger::Info("Reg type %d now has %d live regs", regType,
                   liveRegs_[regType].GetOneCnt());
#endif

      if (RegFiles_[regType].GetPhysRegCnt() > 0 && physRegNum >= 0)
        LivePhysRegs_[regType].SetBit(physRegNum, false, use->GetWght());
    }
  }

  // Update Live regs after defs
  for (llvm::opt_sched::Register *def : inst->GetDefs()) {
    regType = def->GetType();
    regNum = def->GetNum();
    physRegNum = def->GetPhysicalNumber();

#ifdef IS_DEBUG_REG_PRESSURE
    Logger::Info("Inst %d defines reg %d of type %d and %d uses",
                 inst->GetNum(), regNum, regType, def->GetUseCnt());
#endif

    // if (def->GetUseCnt() > 0) {

    if (trackCnflcts && LiveRegs_[regType].GetOneCnt() > 0)
      RegFiles_[regType].AddConflictsWithLiveRegs(
          regNum, LiveRegs_[regType].GetOneCnt(), SolverID_);

    LiveRegs_[regType].SetBit(regNum, true, def->GetWght());

#ifdef IS_DEBUG_REG_PRESSURE
    Logger::Info("Reg type %d now has %d live regs", regType,
                 liveRegs_[regType].GetOneCnt());
#endif

    if (RegFiles_[regType].GetPhysRegCnt() > 0 && physRegNum >= 0)
      LivePhysRegs_[regType].SetBit(physRegNum, true, def->GetWght());
    def->ResetCrntUseCnt(SolverID_);
    //}
  }

  newSpillCost = 0;

#ifdef IS_DEBUG_SLIL_CORRECT
  if (OPTSCHED_gPrintSpills) {
    Logger::Info(
        "Printing live range lengths for instruction BEFORE calculation.");
    for (int j = 0; j < sumOfLiveIntervalLengths_.size(); j++) {
      Logger::Info("SLIL for regType %d %s is currently %d", j,
                   sumOfLiveIntervalLengths_[j]);
    }
    Logger::Info("Now computing spill cost for instruction.");
  }
#endif

  for (int16_t i = 0; i < RegTypeCnt_; i++) {
    liveRegs = LiveRegs_[i].GetWghtedCnt();
    // Set current RP for register type "i"
    RegPressures_[i] = liveRegs;
    // Update peak RP for register type "i"
    if (liveRegs > PeakRegPressures_[i])
      PeakRegPressures_[i] = liveRegs;

    // (Chris): Compute sum of live range lengths at this point
    if (SpillCostFunc_ == SCF_SLIL) {
      SumOfLiveIntervalLengths_[i] += LiveRegs_[i].GetOneCnt();
      for (int j = 0; j < LiveRegs_[i].GetSize(); ++j) {
        if (LiveRegs_[i].GetBit(j)) {
          const llvm::opt_sched::Register *reg = RegFiles_[i].GetReg(j);
          if (!reg->IsInInterval(inst) && !reg->IsInPossibleInterval(inst)) {
            ++DynamicSlilLowerBound_;
          }
        }
      }
    }

    // FIXME: Can this be taken out of this loop?
    if (SpillCostFunc_ == SCF_SLIL) {
      SlilSpillCost_ = std::accumulate(SumOfLiveIntervalLengths_.begin(),
                                       SumOfLiveIntervalLengths_.end(), 0);
    }
  }

  if (SpillCostFunc_ == SCF_TARGET) {
    newSpillCost = OST->getCost(RegPressures_);
    SubspaceLwrBound_ = (int64_t)std::accumulate(RegPressures_.begin(), RegPressures_.end(), 0);


  } else if (SpillCostFunc_ == SCF_SLIL) {
    SlilSpillCost_ = std::accumulate(SumOfLiveIntervalLengths_.begin(),
                                     SumOfLiveIntervalLengths_.end(), 0);
    SubspaceLwrBound_ = (int64_t)SlilSpillCost_;

  } else if (SpillCostFunc_ == SCF_PRP) {
    newSpillCost =
        std::accumulate(RegPressures_.begin(), RegPressures_.end(), 0);

    SubspaceLwrBound_ = (int64_t)newSpillCost;

  } else if (SpillCostFunc_ == SCF_PEAK_PER_TYPE) {
    for (int i = 0; i < RegTypeCnt_; i++) {
      newSpillCost +=
          std::max(0, PeakRegPressures_[i] - OST->MM->GetPhysRegCnt(i));
    }
    SubspaceLwrBound_ =(int64_t)newSpillCost;

  } else {
    // Default is PERP (Some SCF like SUM rely on PERP being the default here)
    int i = 0;
    std::for_each(
        RegPressures_.begin(), RegPressures_.end(), [&](InstCount RP) {
          newSpillCost += std::max(0, RP - OST->MM->GetPhysRegCnt(i++));
        });

    //LB for work stealing is PRP if using PERP
    SubspaceLwrBound_ = (int64_t)std::accumulate(RegPressures_.begin(), RegPressures_.end(), 0);
  }

#ifdef IS_DEBUG_SLIL_CORRECT
  if (OPTSCHED_gPrintSpills) {
    Logger::Info(
        "Printing live range lengths for instruction AFTER calculation.");
    for (int j = 0; j < sumOfLiveIntervalLengths_.size(); j++) {
      Logger::Info("SLIL for regType %d is currently %d", j,
                   sumOfLiveIntervalLengths_[j]);
    }
  }
#endif

  CrntStepNum_++;
  SpillCosts_[CrntStepNum_] = newSpillCost;

#ifdef IS_DEBUG_REG_PRESSURE
  Logger::Info("Spill cost at step  %d = %d", crntStepNum_, newSpillCost);
#endif

  TotSpillCost_ += newSpillCost;

  PeakSpillCost_ = std::max(PeakSpillCost_, newSpillCost);

  cmputCrntSpillCost_();

  SchduldInstCnt_++;
  if (inst->MustBeInBBEntry())
    SchduldEntryInstCnt_++;
  if (inst->MustBeInBBExit())
    SchduldExitInstCnt_++;
}
/*****************************************************************************/

void BBThread::updateSpillInfoForUnSchdul(SchedInstruction *inst) {
  int16_t regType;
  int regNum, physRegNum;
  bool isLive;

#ifdef IS_DEBUG_REG_PRESSURE
  Logger::Info("Updating reg pressure after unscheduling Inst %d",
               inst->GetNum());
#endif

  // (Chris): Update the SLIL for all live regs at this point.
  if (SpillCostFunc_ == SCF_SLIL) {
    for (int i = 0; i < RegTypeCnt_; ++i) {
      for (int j = 0; j < LiveRegs_[i].GetSize(); ++j) {
        if (LiveRegs_[i].GetBit(j)) {
          const llvm::opt_sched::Register *reg = RegFiles_[i].GetReg(j);
          SumOfLiveIntervalLengths_[i]--;
          if (!reg->IsInInterval(inst) && !reg->IsInPossibleInterval(inst)) {
            --DynamicSlilLowerBound_;
          }
        }
      }
      //assert(sumOfLiveIntervalLengths_[i] >= 0 &&
      //       "updateSpillInfoForUnSchdul: SLIL negative!");
    }
  }

  // Update Live regs
  for (llvm::opt_sched::Register *def : inst->GetDefs()) {
    regType = def->GetType();
    regNum = def->GetNum();
    physRegNum = def->GetPhysicalNumber();

#ifdef IS_DEBUG_REG_PRESSURE
    Logger::Info("Inst %d defines reg %d of type %d and %d uses",
                 inst->GetNum(), regNum, regType, def->GetUseCnt());
#endif

    // if (def->GetUseCnt() > 0) {

    //TODO why is this assert commented
    //assert(liveRegs_[regType].GetBit(regNum));
    LiveRegs_[regType].SetBit(regNum, false, def->GetWght());

#ifdef IS_DEBUG_REG_PRESSURE
    Logger::Info("Reg type %d now has %d live regs", regType,
                 liveRegs_[regType].GetOneCnt());
#endif

    if (RegFiles_[regType].GetPhysRegCnt() > 0 && physRegNum >= 0)
      LivePhysRegs_[regType].SetBit(physRegNum, false, def->GetWght());
    def->ResetCrntUseCnt(SolverID_);
    //}
  }

  for (llvm::opt_sched::Register *use : inst->GetUses()) {
    regType = use->GetType();
    regNum = use->GetNum();
    physRegNum = use->GetPhysicalNumber();

#ifdef IS_DEBUG_REG_PRESSURE
    Logger::Info("Inst %d uses reg %d of type %d and %d uses", inst->GetNum(),
                 regNum, regType, use->GetUseCnt());
#endif

    isLive = use->IsLive(SolverID_);
    use->DelCrntUse(SolverID_);

    assert(use->IsLive(SolverID_));

    if (isLive == false) {
      // (Chris): Since this was the last use, the above SLIL calculation didn't
      // take this instruction into account.
      if (SpillCostFunc_ == SCF_SLIL) {
        SumOfLiveIntervalLengths_[regType]--;
        if (!use->IsInInterval(inst) && !use->IsInPossibleInterval(inst)) {
          --DynamicSlilLowerBound_;
        }
        //TODO why is this assert commented
        //assert(sumOfLiveIntervalLengths_[regType] >= 0 &&
        //       "updateSpillInfoForUnSchdul: SLIL negative!");
      }
      LiveRegs_[regType].SetBit(regNum, true, use->GetWght());

#ifdef IS_DEBUG_REG_PRESSURE
      Logger::Info("Reg type %d now has %d live regs", regType,
                   liveRegs_[regType].GetOneCnt());
#endif

      if (RegFiles_[regType].GetPhysRegCnt() > 0 && physRegNum >= 0)
        LivePhysRegs_[regType].SetBit(physRegNum, true, use->GetWght());
    }
  }

  SchduldInstCnt_--;
  if (inst->MustBeInBBEntry())
    SchduldEntryInstCnt_--;
  if (inst->MustBeInBBExit())
    SchduldExitInstCnt_--;

  TotSpillCost_ -= SpillCosts_[CrntStepNum_];
  CrntStepNum_--;

#ifdef IS_DEBUG_REG_PRESSURE
// Logger::Info("Spill cost at step  %d = %d", crntStepNum_, newSpillCost);
#endif
}
/*****************************************************************************/

void BBThread::schdulInst(SchedInstruction *inst, InstCount cycleNum,
                             InstCount slotNum, bool trackCnflcts) {
  CrntCycleNum_ = cycleNum;
  CrntSlotNum_ = slotNum;
  if (inst == NULL)
    return;
  assert(inst != NULL);
  updateSpillInfoForSchdul(inst, trackCnflcts);
}
/*****************************************************************************/

void BBThread::unschdulInst(SchedInstruction *inst, InstCount cycleNum,
                               InstCount slotNum, EnumTreeNode *trgtNode) {
  if (slotNum == 0) {
    CrntCycleNum_ = cycleNum - 1;
    CrntSlotNum_ = IssueRate_ - 1;
  } else {
    CrntCycleNum_ = cycleNum;
    CrntSlotNum_ = slotNum - 1;
  }

  if (inst == NULL) {
    return;
  }

  updateSpillInfoForUnSchdul(inst);
  PeakSpillCost_ = trgtNode->GetPeakSpillCost();
  cmputCrntSpillCost_();
}
/*****************************************************************************/
void BBThread::unschdulInstAndRevert(SchedInstruction *inst, InstCount cycleNum,
                               InstCount slotNum, InstCount prevPeakSpillCost) {
  if (slotNum == 0) {
    CrntCycleNum_ = cycleNum - 1;
    CrntSlotNum_ = IssueRate_ - 1;
  } else {
    CrntCycleNum_ = cycleNum;
    CrntSlotNum_ = slotNum - 1;
  }

  if (inst == NULL) {
    return;
  }

  updateSpillInfoForUnSchdul(inst);
  PeakSpillCost_ = prevPeakSpillCost;
  cmputCrntSpillCost_();
}

/*****************************************************************************/

void BBThread::FinishOptmlBBThread_() {
#ifdef IS_DEBUG_BBSPILL_COST
  stats::traceOptimalCost.Record(bestCost_);
  stats::traceOptimalScheduleLength.Record(bestSchedLngth_);
#endif
}
/*****************************************************************************/

void BBThread::setupForSchdulng() {
  for (int i = 0; i < RegTypeCnt_; i++) {
    LiveRegs_[i].Construct(RegFiles_[i].GetRegCnt());
  }

  setupPhysRegs_();

  SchduldEntryInstCnt_ = 0;
  SchduldExitInstCnt_ = 0;

  /*
  if (chkCnflcts_)
    for (int i = 0; i < regTypeCnt_; i++) {
      regFiles_[i].SetupConflicts();
    }
 */
}
/*****************************************************************************/

bool BBThread::chkCostFsblty(InstCount trgtLngth, EnumTreeNode *&node, bool isGlobalPoolNode) {
  
  bool fsbl = true;
  InstCount crntCost, dynmcCostLwrBound;
  if (SpillCostFunc_ == SCF_SLIL) {
    crntCost = DynamicSlilLowerBound_ * SCW_ + trgtLngth * SchedCostFactor_;
  } else {
    crntCost = CrntSpillCost_ * SCW_ + trgtLngth * SchedCostFactor_;
  }
  crntCost -= getCostLwrBound();
  dynmcCostLwrBound = crntCost;

  assert(dynmcCostLwrBound >= 0);
 
  fsbl = dynmcCostLwrBound < getBestCost(); 

  // FIXME: RP tracking should be limited to the current SCF. We need RP
  // tracking interface.
  if (fsbl || isGlobalPoolNode) {
    assert(node);
    node->SetCost(crntCost);
    node->SetCostLwrBound(dynmcCostLwrBound);
    node->SetTotalCost(dynmcCostLwrBound);
    node->SetPeakSpillCost(PeakSpillCost_);
    node->SetSpillCostSum(TotSpillCost_);
  }

  if (!fsbl) {
    node->SetLocalBestCost(dynmcCostLwrBound);
  }
  
  stats::costInfeasibilityHits++;
  return fsbl;
}
/*****************************************************************************/

void BBThread::setSttcLwrBounds(EnumTreeNode *) {
  // Nothing.
}

/*****************************************************************************/

bool BBThread::chkInstLgltyBBThread(SchedInstruction *inst) {
  return true;
  /*
  int16_t regType;
  int defCnt, physRegNum;
  Register **defs;
  Register *def, *liveDef;

#ifdef IS_DEBUG_CHECK
  Logger::Info("Checking inst %d %s", inst->GetNum(), inst->GetOpCode());
#endif

  if (fixLivein_) {
    if (inst->MustBeInBBEntry() == false &&
        schduldEntryInstCnt_ < entryInstCnt_)
      return false;
  }

  if (fixLiveout_) {
    if (inst->MustBeInBBExit() == true &&
        schduldInstCnt_ < (dataDepGraph_->GetInstCnt() - exitInstCnt_))
      return false;
  }

  // Update Live regs
  for (Register *def : inst->GetDefs()) {
    regType = def->GetType();
    physRegNum = def->GetPhysicalNumber();

    // If this is a physical register definition and another
    // definition of the same physical register is live, then
    // scheduling this instruction is illegal unless this
    // instruction is the last use of that physical reg definition.
    if (regFiles_[regType].GetPhysRegCnt() > 0 && physRegNum >= 0 &&
        livePhysRegs_[regType].GetBit(physRegNum) == true) {

      liveDef = regFiles_[regType].FindLiveReg(physRegNum);
      assert(liveDef != NULL);

      // If this instruction is the last use of the current live def
      if (liveDef->GetCrntUseCnt() + 1 == liveDef->GetUseCnt() &&
          inst->FindUse(liveDef) == true)
        return true;
      else
        return false;
    } // end if
  }   // end for
  return true;
  */
}

bool BBThread::ChkScheduleBBThread_(InstSchedule *bestSched,
                               InstSchedule *lstSched) {
  return true;
  /*
  if (bestSched == NULL || bestSched == lstSched)
    return true;
  if (chkSpillCostSum_) {

    InstCount i, heurLarger = 0, bestLarger = 0;
    for (i = 0; i < dataDepGraph_->GetInstCnt(); i++) {
      if (lstSched->GetSpillCost(i) > bestSched->GetSpillCost(i))
        heurLarger++;
      if (bestSched->GetSpillCost(i) > lstSched->GetSConstructd points, while best "
                 "spill cost is larger at %d points",
                 heurLarger, bestLarger);
    if (bestSched->GetTotSpillCost() > lstSched->GetTotSpillCost()) {
      // Enumerator's best schedule has a greater spill cost sum than the
      // heuristic
      // This can happen if we are using a cost function other than the spill
      // cost sum function
      Logger::Info("??? Heuristic sched has a smaller spill cost sum than best "
                   "sched, heur : %d, best : %d. ",
                   lstSched->GetTotSpillCost(), bestSched->GetTotSpillCost());
      if (lstSched->GetCrntLngth() <= bestSched->GetCrntLngth()) {
        Logger::Info("Taking heuristic schedule");
        bestSched->Copy(lstSched);
        return false;
      }
    }
  }
  if (chkCnflcts_) {
    cmputCnflcts_(lstSched);
    cmputCnflcts_(bestSched);

#ifdef IS_DEBUG_CONFLICTS
    Logger::Info("Heuristic conflicts : %d, best conflicts : %d. ",
                 lstSched->GetConflictCount(), bestSched->GetConflictCount());
#endif

    if (bestSched->GetConflictCount() > lstSched->GetConflictCount()) {
      // Enumerator's best schedule causes more conflicst than the heuristic
      // schedule.
      Logger::Info("??? Heuristic sched causes fewer conflicts than best "
                   "sched, heur : %d, best : %d. ",
                   lstSched->GetConflictCount(), bestSched->GetConflictCount());
      if (lstSched->GetCrntLngth() <= bestSched->GetCrntLngth()) {
        Logger::Info("Taking heuristic schedule");
        bestSched->Copy(lstSched);
        return false;
      }
    }
  }
  return true;
  */
}

void BBThread::cmputCnflcts_(InstSchedule *sched) {
  int cnflctCnt = 0;
  InstCount execCost;

  cmputNormCost(sched, CCM_STTC, execCost, true);
  for (int i = 0; i < RegTypeCnt_; i++) {
    cnflctCnt += RegFiles_[i].GetConflictCnt();
  }
  sched->SetConflictCount(cnflctCnt);
}

bool BBThread::EnableEnumBBThread_() {
  return true;
  /*
  if (maxSpillCost_ > 0 && hurstcCost_ > maxSpillCost_) {
    Logger::Info("Bypassing enumeration due to a large spill cost of %d",
                 hurstcCost_);
    return false;
  }
  return true;
  */
}

InstSchedule *BBThread::allocNewSched() {
  InstSchedule *newSched = new InstSchedule(MachMdl_, DataDepGraph_, VrfySched_);
  return newSched;
}


/******************************************************************/
/******************************************************************/
/******************************************************************/
/******************************************************************/
/******************************************************************/

BBInterfacer::BBInterfacer(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
              long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
              SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
              bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
              bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
              SchedulerType HeurSchedType)
              : SchedRegion(OST_->MM, dataDepGraph, rgnNum, sigHashSize, lbAlg,
                  hurstcPrirts, enumPrirts, vrfySched, PruningStrategy,
                  HeurSchedType, spillCostFunc) ,
                BBThread(OST_, dataDepGraph, rgnNum, sigHashSize, lbAlg, hurstcPrirts,
                         enumPrirts, vrfySched, PruningStrategy, SchedForRPOnly,
                         enblStallEnum, SCW, spillCostFunc, HeurSchedType)
{
  BestCost_  = &bestCost_;
}

void BBInterfacer::CmputSchedUprBound_() {
  // The maximum increase in sched length that might result in a smaller cost
  // than the known one
  int maxLngthIncrmnt = (getBestCost() - 1) / SchedCostFactor_;

  if (SimpleMachineModel_ && MaxLatency_ <= 1) {
#if defined(IS_DEBUG_DAG) || defined(IS_DEBUG_SIMPLE_DAGS)
    Logger::Info("Simple DAG with max latency of one or less.");
#endif
    maxLngthIncrmnt = 0;
  }

  assert(maxLngthIncrmnt >= 0);

  // Any schedule longer than this will have a cost that is greater than or
  // equal to that of the list schedule
  schedUprBound_ = schedLwrBound_ + maxLngthIncrmnt;

  if (abslutSchedUprBound_ < schedUprBound_) {
    schedUprBound_ = abslutSchedUprBound_;
  }
}

void BBInterfacer::CmputAbslutUprBound_() {
  abslutSchedUprBound_ = dataDepGraph_->GetAbslutSchedUprBound();
  dataDepGraph_->SetAbslutSchedUprBound(abslutSchedUprBound_);
}


InstCount BBInterfacer::cmputCostLwrBound() {
  InstCount spillCostLwrBound = 0;

  if (GetSpillCostFunc() == SCF_SLIL) {
    spillCostLwrBound =
        ComputeSLILStaticLowerBound(RegTypeCnt_, RegFiles_, dataDepGraph_);
    DynamicSlilLowerBound_ = spillCostLwrBound;
    StaticSlilLowerBound_ = spillCostLwrBound;
  }

  StaticLowerBound_ =
      schedLwrBound_ * SchedCostFactor_ + spillCostLwrBound * SCW_;
  
#if defined(IS_DEBUG_STATIC_LOWER_BOUND)
  Logger::Event("StaticLowerBoundDebugInfo", "name", dataDepGraph_->GetDagID(),
                "spill_cost_lb", spillCostLwrBound, "sc_factor", SCW_,       //
                "length_lb", schedLwrBound_, "len_factor", schedCostFactor_, //
                "static_lb", staticLowerBound);
#endif

  return StaticLowerBound_;
}

InstCount BBInterfacer::ComputeSLILStaticLowerBound(int64_t regTypeCnt_,
                                             RegisterFile *regFiles_,
                                             DataDepGraph *dataDepGraph_) {
  // (Chris): To calculate a naive lower bound of the SLIL, count all the defs
  // and uses for each register.
  int naiveLowerBound = 0;
  for (int i = 0; i < regTypeCnt_; ++i) {
    for (int j = 0; j < regFiles_[i].GetRegCnt(); ++j) {
      const auto &reg = regFiles_[i].GetReg(j);
      for (const auto &instruction : reg->GetDefList()) {
        if (reg->AddToInterval(instruction)) {
          ++naiveLowerBound;
        }
      }
      for (const auto &instruction : reg->GetUseList()) {
        if (reg->AddToInterval(instruction)) {
          ++naiveLowerBound;
        }
      }
    }
  }

#if defined(IS_DEBUG_SLIL_COST_LOWER_BOUND)
  Logger::Info("SLIL Naive Static Lower Bound Cost  is %llu for Dag %s",
               naiveLowerBound, dataDepGraph_->GetDagID());
#endif

  // (Chris): Another improvement to the lower bound calculation takes advantage
  // of the transitive closure of the DAG. Suppose instruction X must happen
  // between A and B, where A defines a register that B uses. Then, the live
  // range length of A increases by 1.
  auto closureLowerBound = naiveLowerBound;
  for (int i = 0; i < dataDepGraph_->GetInstCnt(); ++i) {
    const auto &inst = dataDepGraph_->GetInstByIndx(i);
    // For each register this instruction defines, compute the intersection
    // between the recursive successor list of this instruction and the
    // recursive predecessors of the dependent instruction.
    auto recSuccBV = inst->GetRcrsvNghbrBitVector(DIR_FRWRD);
    for (llvm::opt_sched::Register *def : inst->GetDefs()) {
      for (const auto &dependentInst : def->GetUseList()) {
        auto recPredBV = const_cast<SchedInstruction *>(dependentInst)
                             ->GetRcrsvNghbrBitVector(DIR_BKWRD);
        assert(recSuccBV->GetSize() == recPredBV->GetSize() &&
               "Successor list size doesn't match predecessor list size!");
        for (int k = 0; k < recSuccBV->GetSize(); ++k) {
          if (recSuccBV->GetBit(k) & recPredBV->GetBit(k)) {
            if (def->AddToInterval(dataDepGraph_->GetInstByIndx(k))) {
              ++closureLowerBound;
            }
          }
        }
      }
    }
  }

#if defined(IS_DEBUG_SLIL_COST_LOWER_BOUND)
  Logger::Info("SLIL Closur Static Lower Bound Cost is %llu for Dag %s",
               closureLowerBound, dataDepGraph_->GetDagID());
#endif

  // (Chris): A better lower bound can be computed by adding more to the SLIL
  // based on the instructions that use more than one register (defined by
  // different instructions).
  int commonUseLowerBound = closureLowerBound;
  std::vector<std::pair<const SchedInstruction *, llvm::opt_sched::Register *>> usedInsts;
  for (int i = 0; i < dataDepGraph_->GetInstCnt(); ++i) {
    const auto &inst = dataDepGraph_->GetInstByIndx(i);

    // Get a list of instructions that define the registers, in array form.
    usedInsts.clear();
    llvm::transform(inst->GetUses(), std::back_inserter(usedInsts),
                    [&](llvm::opt_sched::Register *reg) {
                      assert(reg->GetDefList().size() == 1 &&
                             "Number of defs for register is not 1!");
                      return std::make_pair(*(reg->GetDefList().begin()), reg);
                    });

#if defined(IS_DEBUG_SLIL_COMMON_USE_LB)
    Logger::Info("Common Use Lower Bound Instruction %d", inst->GetNum());
    Logger::Info("  Instruction %d uses:", inst->GetNum());
    for (const auto &p : usedInsts) {
      Logger::Info("    Instruction %d register %d:%d", p.first->GetNum(),
                   p.second->GetType(), p.second->GetNum());
    }

    for (const auto &p : usedInsts) {
      Logger::Info("  Live interval of Register %d:%d (defined by Inst %d):",
                   p.second->GetType(), p.second->GetNum(), p.first->GetNum());
      for (const auto &s : p.second->GetLiveInterval()) {
        Logger::Info("    %d", s->GetNum());
      }
    }
#endif

    for (size_t j = 0; j < usedInsts.size(); ++j) {
      for (size_t k = j + 1; k < usedInsts.size(); ++k) {
        const auto &jReg = usedInsts[j].second;
        const auto &kReg = usedInsts[k].second;

        // If k is not in the live interval of j AND ALSO j is not in the live
        // interval of k, add k to the live interval of j, and increment the
        // lower bound by 1.
        bool found = jReg->IsInInterval(usedInsts[k].first) ||
                     kReg->IsInInterval(usedInsts[j].first) ||
                     jReg->IsInPossibleInterval(usedInsts[k].first) ||
                     kReg->IsInPossibleInterval(usedInsts[j].first);

        if (!found && usedInsts[j].first != usedInsts[k].first) {
          jReg->AddToPossibleInterval(usedInsts[k].first);
          kReg->AddToPossibleInterval(usedInsts[j].first);

          commonUseLowerBound++;
#if defined(IS_DEBUG_SLIL_COMMON_USE_LB)
          Logger::Info("  Common Use: Found two instructions %d and %d",
                       usedInsts[j].first->GetNum(),
                       usedInsts[k].first->GetNum());
#endif
        }
      }
    }
  }

#if defined(IS_DEBUG_SLIL_COST_LOWER_BOUND)
  if (commonUseLowerBound > closureLowerBound)
    Logger::Info("SLIL Final  Static Lower Bound Cost is %llu for Dag %s",
                 commonUseLowerBound, dataDepGraph_->GetDagID());
#endif

  return static_cast<InstCount>(commonUseLowerBound);
}

InstCount BBInterfacer::UpdtOptmlSched(InstSchedule *crntSched,
                                      LengthCostEnumerator *) {
  InstCount crntCost;
  InstCount crntExecCost;

  crntCost = CmputNormCost_(crntSched, CCM_STTC, crntExecCost, false);


  Logger::Info(
      "Found a feasible sched. of length %d, spill cost %d and tot cost %d",
      crntSched->GetCrntLngth(), crntSched->GetSpillCost(), crntCost);

  if (crntCost < getBestCost()) {

    if (crntSched->GetCrntLngth() > schedLwrBound_)
      Logger::Info("$$$ GOOD_HIT: Better spill cost for a longer schedule");

    setBestCost(crntCost);
    OptmlSpillCost_ = CrntSpillCost_;
    SetBestSchedLength(crntSched->GetCrntLngth());
    enumBestSched_->Copy(crntSched);
    bestSched_ = enumBestSched_;
  }

  return getBestCost();
}

FUNC_RESULT BBWithSpill::Enumerate_(Milliseconds StartTime, 
                                    Milliseconds RgnTimeout,
                                    Milliseconds LngthTimeout,
                                    int *OptimalSolverID) {
  InstCount trgtLngth;
  FUNC_RESULT rslt = RES_SUCCESS;
  int iterCnt = 0;
  int costLwrBound = 0;
  bool timeout = false;

  // Non-parallel enumerator, if finds optimal, then SolverID must be this thread
  // Required by SchedRegion which uses SolverID as index when verifying schedule
  SolverID_ = *OptimalSolverID = 0;

  Milliseconds rgnDeadline, lngthDeadline;
  rgnDeadline =
    (RgnTimeout == INVALID_VALUE) ? INVALID_VALUE : StartTime + RgnTimeout;
  lngthDeadline =
    (RgnTimeout == INVALID_VALUE) ? INVALID_VALUE : StartTime + LngthTimeout;
  

  Milliseconds deadline = IsTimeoutPerInst_ ? lngthDeadline : rgnDeadline;

  for (trgtLngth = schedLwrBound_; trgtLngth <= schedUprBound_; trgtLngth++) {
    InitForSchdulng();
    Logger::Event("Enumerating", "target_length", trgtLngth);

    rslt = Enumrtr_->FindFeasibleSchedule(enumCrntSched_, trgtLngth, this,
                                          costLwrBound, deadline);
    if (rslt == RES_TIMEOUT)
      timeout = true;
    HandlEnumrtrRslt_(rslt, trgtLngth);

   

    if (getBestCost() == 0 || rslt == RES_ERROR ||
        rslt == RES_TIMEOUT ||
        (rslt == RES_SUCCESS && isSecondPass())) {

      // If doing two pass optsched and on the second pass then terminate if a
      // schedule is found with the same min-RP found in first pass.
      if (rslt == RES_SUCCESS && isSecondPass()) {
        Logger::Info("Schedule found in second pass, terminating BB loop.");

        if (trgtLngth < schedUprBound_)
          Logger::Info("Schedule found with length %d is shorter than current "
                       "schedule with length %d.",
                       trgtLngth, schedUprBound_);
      }
      Logger::Info("breaking out of loop");
      break;
    }

    Enumrtr_->Reset();
    enumCrntSched_->Reset();

    if (!isSecondPass())
      CmputSchedUprBound_();

    iterCnt++;
    costLwrBound += 1;
    lngthDeadline = Utilities::GetProcessorTime() + LngthTimeout;
    if (lngthDeadline > rgnDeadline)
      lngthDeadline = rgnDeadline;
  }

  stats::positiveDominationHits.Print(cout);
  stats::nodeSuperiorityInfeasibilityHits.Print(cout);
  stats::costInfeasibilityHits.Print(cout);

#ifdef IS_DEBUG_ITERS
  stats::iterations.Record(iterCnt);
  stats::enumerations.Record(enumrtr_->GetSearchCnt());
  stats::lengths.Record(iterCnt);
#endif

  // Failure to find a feasible sched. in the last iteration is still
  // considered an overall success
  if (rslt == RES_SUCCESS || rslt == RES_FAIL) {
    rslt = RES_SUCCESS;
  }
  if (timeout)
    rslt = RES_TIMEOUT;

  return rslt;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/


BBWithSpill::BBWithSpill(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
              long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
              SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
              bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
              bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
              SchedulerType HeurSchedType, int timeoutToMemblock, bool twoPassEnabled,
              bool IsTimeoutPerInst)
              : BBInterfacer(OST_, dataDepGraph, rgnNum, sigHashSize, lbAlg, hurstcPrirts,
                             enumPrirts, vrfySched, PruningStrategy, SchedForRPOnly, 
                             enblStallEnum, SCW, spillCostFunc, HeurSchedType) {
    SolverID_ = 0;
    NumSolvers_ = 1;
    TwoPassEnabled_ = twoPassEnabled;

    timeoutToMemblock_ = timeoutToMemblock;
    IsTimeoutPerInst_ = IsTimeoutPerInst;
    Logger::Event("FinishedConstBBInterfacer");
}

Enumerator *BBWithSpill::AllocEnumrtr_(Milliseconds timeout) {
  bool enblStallEnum = EnblStallEnum_;

  Enumrtr_ = new LengthCostEnumerator(this,
      dataDepGraph_, machMdl_, schedUprBound_, GetSigHashSize(),
      GetEnumPriorities(), GetPruningStrategy(), SchedForRPOnly_, enblStallEnum,
      timeout, GetSpillCostFunc(), isSecondPass_, 1, timeoutToMemblock_, 0, 0, NULL);

  return Enumrtr_;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

BBWorker::BBWorker(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
              long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
              SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
              bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
              bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc, bool twoPassEnabled,
              SchedulerType HeurSchedType, bool IsSecondPass, InstSchedule *MasterSched, 
              InstCount *MasterCost, InstCount *MasterSpill, InstCount *MasterLength, 
              InstPool4 *GlobalPool, 
              uint64_t *NodeCount, int SolverID,  std::mutex **HistTableLock, std::mutex *GlobalPoolLock, 
              std::mutex *BestSchedLock, std::mutex *NodeCountLock, std::mutex *ImprvmntCntLock,
              std::mutex *RegionSchedLock, vector<FUNC_RESULT> *RsltAddr, int *idleTimes,
              int NumSolvers, vector<InstPool3 *> localPools, std::mutex **localPoolLocks,
              int *inactiveThreads, std::mutex *inactiveThreadLock, int LocalPoolSize, bool WorkSteal,
              bool *WorkStealOn, bool IsTimeoutPerInst, uint64_t *nodeCounts, int timeoutToMemblock, int64_t **subspaceLwrBounds) 
              : BBThread(OST_, dataDepGraph, rgnNum, sigHashSize, lbAlg,
              hurstcPrirts, enumPrirts, vrfySched, PruningStrategy, SchedForRPOnly,
              enblStallEnum, SCW, spillCostFunc, HeurSchedType) {
  assert(SolverID > 0); // do not overwrite master threads structures
  
  DataDepGraph_ = dataDepGraph;
  MachMdl_ = OST_->MM;
  EnumPrirts_ = enumPrirts;
  PruningStrategy_ = PruningStrategy;
  SpillCostFunc_ = spillCostFunc;
  SigHashSize_ = sigHashSize;

  IsSecondPass_ = IsSecondPass;
  TwoPassEnabled_ = twoPassEnabled;
  NumSolvers_ = NumSolvers; // NumSolvers_ is the number of Threads

  // Shared Fields
  MasterSched_ = MasterSched;
  MasterCost_ = MasterCost;
  MasterSpill_ = MasterSpill;
  MasterLength_ = MasterLength;
  GlobalPool_ = GlobalPool;
  NodeCount_ = NodeCount;

  EnumBestSched_ = NULL;
  EnumCrntSched_ = NULL;

  SolverID_ = SolverID;

  HistTableLock_ = HistTableLock;
  GlobalPoolLock_ = GlobalPoolLock;
  BestSchedLock_ = BestSchedLock;
  RegionSchedLock_ =  RegionSchedLock;
  NodeCountLock_ = NodeCountLock;
  ImprvmntCntLock_ = ImprvmntCntLock;

  RsltAddr_ = RsltAddr;

  IdleTime_ = idleTimes;
  nodeCounts_ = nodeCounts;

  localPools_ = localPools;
  localPoolLocks_ = localPoolLocks;

  InactiveThreads_ = inactiveThreads;
  InactiveThreadLock_ = inactiveThreadLock;

  WorkSteal_ = WorkSteal;
  WorkStealOn_ = WorkStealOn;
  assert(!*WorkStealOn_);
  subspaceLwrBounds_ = subspaceLwrBounds;
  IsTimeoutPerInst_ = IsTimeoutPerInst;
  timeoutToMemblock_ = timeoutToMemblock;

  subspaceLwrBounds[SolverID_-2] = &SubspaceLwrBound_;
}

BBWorker::~BBWorker() {
  delete EnumCrntSched_;
}

void BBWorker::setHeurInfo(InstCount SchedUprBound, InstCount HeuristicCost, 
                           InstCount SchedLwrBound)
{
  SchedUprBound_ = SchedUprBound;
  HeuristicCost_ = HeuristicCost;
  SchedLwrBound_ = SchedLwrBound;
}

/*****************************************************************************/
void BBWorker::allocEnumrtr_(Milliseconds Timeout) {

  Enumrtr_ = new LengthCostEnumerator(this,
      DataDepGraph_, MachMdl_, SchedUprBound_, SigHashSize_,
      EnumPrirts_, PruningStrategy_, SchedForRPOnly_, EnblStallEnum_,
      Timeout, SpillCostFunc_, IsSecondPass_, timeoutToMemblock_, NumSolvers_, SolverID_, 0, NULL);

}
/*****************************************************************************/

void BBWorker::setLCEElements_(InstCount costLwrBound)
{
  Enumrtr_->setLCEElements((BBThread *)this, costLwrBound);
}

void BBWorker::setLowerBounds_(InstCount SlilLowerBound) {
  if (SpillCostFunc_ == SCF_SLIL) {
    DynamicSlilLowerBound_ = SlilLowerBound;
    StaticSlilLowerBound_ = SlilLowerBound;
  }
}

/*****************************************************************************/

void BBWorker::allocSched_() {
  EnumCrntSched_ = new InstSchedule(MachMdl_, DataDepGraph_, VrfySched_);
}
/*****************************************************************************/

void BBWorker::initEnumrtr_(bool scheduleRoot) {
  Enumrtr_->Initialize_(EnumCrntSched_, SchedLwrBound_, SolverID_, scheduleRoot);
}

/*****************************************************************************/
void BBWorker::handlEnumrtrRslt_(FUNC_RESULT rslt, InstCount trgtLngth) {
  switch (rslt) {
  case RES_FAIL:
    //    #ifdef IS_DEBUG_ENUM_ITERS
    //Logger::Info("No feasible solution of length %d was found.", trgtLngth);
    //    #endif
    break;
  case RES_SUCCESS:
#ifdef IS_DEBUG_ENUM_ITERS
    Logger::Info("Found a feasible solution of length %d.", trgtLngth);
#endif
    break;
  case RES_TIMEOUT:
    //    #ifdef IS_DEBUG_ENUM_ITERS
    //Logger::Info("Enumeration timedout at length %d.", trgtLngth);
    //    #endif
    break;
  case RES_ERROR:
    //Logger::Info("The processing of DAG \"%s\" was terminated with an error.",
    //             dataDepGraph_->GetDagID(), rgnNum_);
    break;
  case RES_END:
    //    #ifdef IS_DEBUG_ENUM_ITERS
    //Logger::Info("Enumeration ended at length %d.", trgtLngth);
    //    #endif
    break;
  case RES_EXIT:
    break;
  }
}
/*****************************************************************************/

InstCount BBWorker::UpdtOptmlSched(InstSchedule *crntSched,
                                      LengthCostEnumerator *) {
  InstCount crntCost;
  InstCount crntExecCost;

  crntCost = CmputNormCost_(crntSched, CCM_STTC, crntExecCost, false);


  if (crntCost < getBestCost()) {

    if (crntSched->GetCrntLngth() > SchedLwrBound_)
      Logger::Info("$$$ GOOD_HIT: Better spill cost for a longer schedule");

    setBestCost(crntCost);
    OptmlSpillCost_ = CrntSpillCost_;

    writeBestSchedToMaster(crntSched, crntCost, CrntSpillCost_);

   
  }

  return getBestCost();
}


/*****************************************************************************/
bool BBWorker::generateStateFromNode(std::shared_ptr<HalfNode> &GlobalPoolNode){ 
  assert(GlobalPoolNode != NULL);
  bool fsbl = true;
  
    int numNodesToSchedule = GlobalPoolNode->getPrefixSize();
    // need to check feasibility
    fsbl = scheduleArtificialRoot(false);

    if (!fsbl) {
      return false;
    }

    if (numNodesToSchedule > 1) {  // then we have insts to schedule
      for (int i = 0; i < numNodesToSchedule - 1; i++) {
        int temp = GlobalPoolNode->getAndRemoveNextPrefixInst();
        fsbl = Enumrtr_->scheduleIntOrPrune(temp, false); 
        if (!fsbl) {
          return false;
        }
      }
      int temp = GlobalPoolNode->getAndRemoveNextPrefixInst();
      fsbl = Enumrtr_->scheduleIntOrPrune(temp, true); 
      if (!fsbl) return false;
    }
  return true;
}




/*****************************************************************************/
bool BBWorker::generateStateFromNode(EnumTreeNode *GlobalPoolNode, bool isGlobalPoolNode){ 
  assert(GlobalPoolNode != NULL);
  bool fsbl = true;
  
  if (isGlobalPoolNode) {
    Enumrtr_->setIsGenerateState(true);
    int numNodesToSchedule = 1 + GlobalPoolNode->getPrefixSize();
    // need to check feasibility
    fsbl = scheduleArtificialRoot(false);

    if (!fsbl) {
      Enumrtr_->setIsGenerateState(false);
      return false;
    }

    if (numNodesToSchedule > 1) {  // then we have insts to schedule
      for (int i = 0; i < numNodesToSchedule - 1; i++) {
        EnumTreeNode *temp = GlobalPoolNode->getAndRemoveNextPrefixInst();
        fsbl = Enumrtr_->scheduleNodeOrPrune(temp, false); 
        if (!fsbl) {
          Enumrtr_->setIsGenerateState(false);
          return false;
        }
      }
    }
    fsbl = Enumrtr_->scheduleNodeOrPrune(GlobalPoolNode, true);
    Enumrtr_->setIsGenerateState(false);
    if (!fsbl) {
      return false;
    }


  }

  else {
    Enumrtr_->setIsGenerateState(true);
    fsbl = scheduleArtificialRoot(true);
    if (!fsbl) {
      Enumrtr_->setIsGenerateState(false); 
      return false;
    }

    std::stack<EnumTreeNode *> prefix;

    EnumTreeNode *temp;
  
    if (GlobalPoolNode->GetInstNum() != Enumrtr_->getRootInstNum()) {
    

      temp = GlobalPoolNode->GetParent();
      int size = 0;

      while (temp->GetInstNum() != Enumrtr_->getRootInstNum()) {
        prefix.push(temp);
        ++size;
        temp = temp->GetParent();
      }

      #ifdef IS_DEBUG_WORKSTEAL
        int prefixInst = GlobalPoolNode->getNextInstPrefix();
        while (prefixInst == Enumrtr_->getRootInstNum() || prefixInst == INVALID_VALUE) {
          prefixInst = GlobalPoolNode->getNextInstPrefix();
        }
      #endif


      int j = 0;
      while (!prefix.empty()) {
        ++j;
        temp = prefix.top();
        prefix.pop();
        #ifdef IS_DEBUG_WORKSTEAL
          assert(temp->GetInstNum() == prefixInst);
          prefixInst = GlobalPoolNode->getNextInstPrefix();
        #endif

        fsbl = Enumrtr_->scheduleNodeOrPrune(temp, false);
        Enumrtr_->removeInstFromRdyLst_(temp->GetInstNum());
        if (!fsbl) {
          Enumrtr_->setIsGenerateState(false); 
          return false;
        }
        
        // TODO -- delete node
      }
      fsbl = Enumrtr_->scheduleNodeOrPrune(GlobalPoolNode, true);
      Enumrtr_->removeInstFromRdyLst_(GlobalPoolNode->GetInstNum());
      Enumrtr_->setIsGenerateState(false); 
      if (!fsbl) return false;
    }
  }
  return true;
}
/*****************************************************************************/
FUNC_RESULT BBWorker::generateAndEnumerate(std::shared_ptr<HalfNode> GlobalPoolNode,
                                 Milliseconds StartTime, 
                                 Milliseconds RgnTimeout,
                                 Milliseconds LngthTimeout) {


  bool fsbl = (GlobalPoolNode.get() != nullptr);
  if (fsbl) {
    Enumrtr_->setIsGenerateState(true);
    fsbl = generateStateFromNode(GlobalPoolNode);
    Enumrtr_->setIsGenerateState(false);
    //delete GlobalPoolNode;
  }
  else {
    Logger::Info("SolverID %d not given a GP Node", SolverID_);
  }
  ++GlobalPoolNodes;
  auto res = enumerate_(StartTime, RgnTimeout, LngthTimeout, false, fsbl);
  //Enumrtr_->freeNodeAllocator();
  //freeAlctrs();
  return res;
}

FUNC_RESULT BBWorker::enumerate_(Milliseconds StartTime, 
                                 Milliseconds RgnTimeout,
                                 Milliseconds LngthTimeout,
                                 bool isWorkStealing,
                                 bool isNodeFsbl) {

  FUNC_RESULT rslt = RES_SUCCESS;
  bool timeout = false;

  //#ifndef WORK_STEAL
  //  #define WORK_STEAL
  //#endif

  //#ifndef DEBUG_GP_HISTORY
  //  #define DEBUG_GP_HISTORY
  //#endif
  
  if (isNodeFsbl || isWorkStealing) {
      InstCount trgtLngth = SchedLwrBound_;
      int costLwrBound = 0;

      Milliseconds rgnDeadline, lngthDeadline;
      rgnDeadline =
          (RgnTimeout == INVALID_VALUE) ? INVALID_VALUE : StartTime + RgnTimeout;
      lngthDeadline =
          (RgnTimeout == INVALID_VALUE) ? INVALID_VALUE : StartTime + LngthTimeout;
      
      Milliseconds deadline = IsTimeoutPerInst_ ? lngthDeadline : rgnDeadline;

      rslt = Enumrtr_->FindFeasibleSchedule(EnumCrntSched_, trgtLngth, this,
                                          costLwrBound, deadline);

    
        SubspaceLwrBound_ = INVALID_VALUE;
        
        NodeCountLock_->lock();
          *NodeCount_ += Enumrtr_->GetNodeCnt();
        NodeCountLock_->unlock();

        nodeCounts_[SolverID_ - 2] += Enumrtr_->GetNodeCnt();

        Enumrtr_->setNodeCnt(0);
        if (rslt == RES_EXIT) {
          return rslt;
        }
                
  
        if (rslt == RES_TIMEOUT)
          timeout = true;
        handlEnumrtrRslt_(rslt, trgtLngth);
    
        // first pass
        if (*MasterImprvCount_ > 0) {
            RegionSchedLock_->lock(); 
              if (MasterSched_->GetSpillCost() < RegionSched_->GetSpillCost()) {
                RegionSched_->Copy(MasterSched_);
                RegionSched_->SetSpillCost(MasterSched_->GetSpillCost());
              }
            RegionSchedLock_->unlock();
        }

        if (RegionSched_->GetCost() == 0 || rslt == RES_ERROR ||
          (rslt == RES_TIMEOUT)) {
   
            //TODO -- notify all other threads to stop
            if (rslt == RES_SUCCESS || rslt == RES_FAIL) {
                rslt = RES_SUCCESS;
            }
            if (timeout)
              rslt = RES_TIMEOUT;

            (*RsltAddr_)[SolverID_-2] = rslt;
            IdleTime_[SolverID_ - 2] = Utilities::GetProcessorTime();
            return rslt;
        }
    }
  
  assert(getLocalPoolSize(SolverID_ - 2) == 0 || RegionSched_->GetCost() == 0 || rslt == RES_TIMEOUT || rslt == RES_ERROR || rslt == RES_EXIT);


  if (true) {
      DataDepGraph_->resetThreadWriteFields(SolverID_, false);
      Enumrtr_->Reset();
      if (Enumrtr_->IsHistDom())
        Enumrtr_->resetEnumHistoryState();
      EnumCrntSched_->Reset();
      initEnumrtr_();
    
  }


  //TODO -- this may be buggy
  if (!GlobalPool_->empty()) {
    if (RegionSched_->GetCost() == 0) return RES_SUCCESS;

    std::shared_ptr<HalfNode> temp;
    while (true) {
      GlobalPoolLock_->lock();
        if (GlobalPool_->empty()) {
          GlobalPoolLock_->unlock(); //deadlock if we dont unlock
#ifdef DEBUG_GP_HISTORY
          Logger::Info("Solver %d exiting global pool loop", SolverID_);
#endif
          break;
        }
        else {
        ++GlobalPoolNodes;
        temp = GlobalPool_->front();
        GlobalPool_->pop();
        }
      GlobalPoolLock_->unlock();
#ifdef DEBUG_GP_HISTORY
      Logger::Info("SolverID %d launching GlobalPoolNode with inst %d (parent %d)", SolverID_, temp->GetInstNum(), temp->GetParent()->GetInstNum());
#endif
      assert(temp != NULL);
      rslt = generateAndEnumerate(temp, StartTime, RgnTimeout, LngthTimeout);
      if (RegionSched_->GetCost() == 0 || rslt == RES_ERROR || (rslt == RES_TIMEOUT) || rslt == RES_EXIT) {
        return rslt;
      }
      break;
    }
  }

#ifdef DEBUG_GP_HISTORY
  Logger::Info("Solver %d bypassed global pool pulling (size = %d)", SolverID_, GlobalPool_->size());
#endif

if (isWorkSteal()) {
  GlobalPoolLock_->lock();
  if (!isWorkStealOn()) {
    setWorkStealOn(true);
    Logger::Info("solverID_ %d just turned on work stealing", SolverID_);
  }
  GlobalPoolLock_->unlock();
  

  IdleTime_[SolverID_ - 2] = Utilities::GetProcessorTime();
  InactiveThreadLock_->lock();
  (*InactiveThreads_)++;
  InactiveThreadLock_->unlock();
  EnumTreeNode *workStealNode;
  bool stoleWork = false;
  bool workStolenFsbl = false;
  bool isTimedOut = false;
  while (!workStolenFsbl && !(RegionSched_->GetCost() == 0) && !isTimedOut && (*InactiveThreads_) < NumSolvers_) {
    if (true) {
      reset_();
      DataDepGraph_->resetThreadWriteFields(SolverID_, false);
      Enumrtr_->Reset();
      EnumCrntSched_->Reset();
      initForSchdulng();
      initEnumrtr_();
    }

    stoleWork = false;

    int64_t minLB = INVALID_VALUE;
    int IDminLB = -1;
    int victimID;

    for (int i = 1; i < NumSolvers_; i++) {
      victimID = (SolverID_ - 2 + i) % NumSolvers_;

      if (*subspaceLwrBounds_[victimID] == INVALID_VALUE) continue;
      if (*subspaceLwrBounds_[victimID] < minLB || minLB == INVALID_VALUE) {
        minLB = *subspaceLwrBounds_[victimID];
        IDminLB = victimID;
      }
    }

    victimID = IDminLB;

    if (victimID != -1) {
      localPoolLock(victimID);
      if (getLocalPoolSize(victimID) < 1) {
        localPoolUnlock(victimID);
      }

      else {
        // must decrement inactive thread count here (before popping)
        // otherwise it is possible that active thread becomes inactive with this steal
        // and reaches while loop condition before we decrement active thread count
        // leading it to believe all threads are inactive
        InactiveThreadLock_->lock();
        (*InactiveThreads_)--;
        InactiveThreadLock_->unlock();
        workStealNode = localPoolPopTail(victimID);
        workStealNode->GetParent()->setStolen(workStealNode->GetInstNum());
        stoleWork = true;
        localPoolUnlock(victimID);
        setStolenNode(workStealNode);
      }
    }
      

    if (stoleWork) {
      workStolenFsbl = generateStateFromNode(workStealNode, false);
      if (!workStolenFsbl) {
        InactiveThreadLock_->lock();
        (*InactiveThreads_)++;
        InactiveThreadLock_->unlock();
        stoleWork = false;
      }
    }

    else {
      Milliseconds clockTime = Utilities::GetProcessorTime();
      if ((IsTimeoutPerInst_ && clockTime  > StartTime + LngthTimeout) || (!IsTimeoutPerInst_ && clockTime > StartTime + RgnTimeout)) {
        isTimedOut = true;
        timeout = true;
        break;
      }
    }
  }

  assert((*InactiveThreads_) < 2 * NumSolvers_);
  if ((*InactiveThreads_) >= NumSolvers_) {
    return RES_EXIT;
  }


  if (stoleWork && workStolenFsbl && !isTimedOut) {
    rslt = enumerate_(StartTime, RgnTimeout, LngthTimeout, true, true);
    assert(getLocalPoolSize(SolverID_ - 2) == 0 || RegionSched_->GetCost() == 0 || rslt == RES_TIMEOUT || rslt == RES_ERROR || rslt == RES_EXIT);
    if (RegionSched_->GetCost() == 0 || rslt == RES_ERROR || (rslt == RES_TIMEOUT) || rslt == RES_EXIT) {
      return rslt;
    }
  }

  if (isTimedOut) {
    return rslt;
  }

  if (Enumrtr_->WasObjctvMet_()) {
    rslt = RES_SUCCESS;
    return rslt;
  }
}

  Enumrtr_->Reset();
  EnumCrntSched_->Reset();
  
  IdleTime_[SolverID_ - 2] = Utilities::GetProcessorTime();

  if (rslt == RES_SUCCESS || rslt == RES_FAIL) {
    rslt = RES_SUCCESS;
  }
  if (timeout) 
    rslt = RES_TIMEOUT;

  return rslt;
}


void BBWorker::writeBestSchedToMaster(InstSchedule *BestSched, InstCount BestCost, 
                                      InstCount BestSpill)
{
  BestSchedLock_->lock();
    // check that our cost is still better -- (race condition)
    if (BestCost < *MasterCost_) {
      MasterSched_->Copy(BestSched);
      MasterSched_->SetSpillCost(BestSpill);
      *MasterCost_ = BestCost;
      *MasterSpill_ = BestSpill;
      *MasterLength_ = BestSched->GetCrntLngth();     
    }
  BestSchedLock_->unlock();
  
  Logger::Info(
      "SolverID_ %d Found a feasible sched. of length %d, spill cost %d and tot cost %d", SolverID_,
      *MasterLength_, *MasterSpill_, *MasterCost_);

}

void BBWorker::histTableLock(UDT_HASHVAL key) {
  assert(key <= 1 + (UDT_HASHVAL)(((int64_t)(1) << SigHashSize_) - 1));
  HistTableLock_[key]->lock(); 
}
  
void BBWorker::histTableUnlock(UDT_HASHVAL key) {
  assert(key <= 1 + (UDT_HASHVAL)(((int64_t)(1) << SigHashSize_) - 1));
  HistTableLock_[key]->unlock(); 
}

void BBWorker::incrementImprvmntCnt() {
  ImprvmntCntLock_->lock();
    (*MasterImprvCount_)++;
  ImprvmntCntLock_->unlock();
}

void BBWorker::localPoolLock(int SolverID) {localPoolLocks_[SolverID]->lock();}

void BBWorker::localPoolUnlock(int SolverID) {localPoolLocks_[SolverID]->unlock();}


void BBWorker::localPoolPushFront(int SolverID, EnumTreeNode *ele) {
  localPools_[SolverID]->pushToFront(ele);
}

EnumTreeNode* BBWorker::localPoolPopFront(int SolverID) {
      EnumTreeNode *temp = localPools_[SolverID]->front();
      localPools_[SolverID]->popFromFront();

      return temp;
}

void BBWorker::localPoolPushTail(int SolverID, EnumTreeNode *ele) {localPools_[SolverID]->pushToBack(ele);}

EnumTreeNode* BBWorker::localPoolPopTail(int SolverID) {
      EnumTreeNode *temp = localPools_[SolverID]->back();
      localPools_[SolverID]->popFromBack();

      return temp;
}

void BBWorker::localPoolRemoveSpecificElement(int SolverID, SchedInstruction *inst, 
                                                       EnumTreeNode *parent,
                                                       EnumTreeNode *&removed) {
  localPools_[SolverID]->removeSpecificElement(inst, parent, removed);
}

int BBWorker::getLocalPoolSize(int SolverID) {return localPools_[SolverID]->size();} 

int BBWorker::getLocalPoolMaxSize(int SolverID) {return localPools_[SolverID]->getMaxSize();} //return 10;}



/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/


BBMaster::BBMaster(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
             long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
             SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
             bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
             bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
             SchedulerType HeurSchedType, int NumThreads, int MinNodesAsMultiple,
             int MinSplittingDepth, 
             int MaxSplittingDepth, int NumSolvers, int LocalPoolSize, float ExploitationPercent, 
             SPILL_COST_FUNCTION GlobalPoolSCF, int GlobalPoolSort, bool WorkSteal, bool IsTimeoutPerInst,
             int timeoutToMemblock, bool twoPassEnabled)
             : BBInterfacer(OST_, dataDepGraph, rgnNum, sigHashSize, lbAlg, hurstcPrirts,
             enumPrirts, vrfySched, PruningStrategy, SchedForRPOnly, 
             enblStallEnum, SCW, spillCostFunc, HeurSchedType) {
  SolverID_ = 0;
  NumThreads_ = NumThreads; //how many workers
  MinNodesAsMultiple_ = MinNodesAsMultiple;
  MinSplittingDepth_ = MinSplittingDepth;
  MaxSplittingDepth_ = MaxSplittingDepth;
  NumSolvers_ = NumSolvers; //how many scheduling instances in total
  GlobalPool = new InstPool4(GlobalPoolSort);
  Logger::Info("setting localPoolSize to %d", LocalPoolSize);  
  LocalPoolSize_ = LocalPoolSize;
  ExploitationPercent_ = ExploitationPercent;
  Logger::Info("setting globalPoolSCF to %d", GlobalPoolSCF);
  GlobalPoolSCF_ = GlobalPoolSCF;

  Logger::Info("setting work steal to %d", WorkSteal);
  WorkSteal_ = WorkSteal;
  WorkStealOn_ = false;

  TwoPassEnabled_ = twoPassEnabled;

  HistTableSize_ = 1 + (UDT_HASHVAL)(((int64_t)(1) << sigHashSize) - 1);
  HistTableLock = new std::mutex*[HistTableSize_];
  localPoolLocks = new std::mutex*[NumThreads_];
 

  idleTimes = new int[NumThreads_];
  nodeCounts = new uint64_t[NumThreads_];
  localPools.resize(NumSolvers);
  subspaceLwrBounds_ = new int64_t*[NumThreads_];
  for (int i = 0; i < NumThreads_; i++) {
    idleTimes[i] = 0;
    nodeCounts[i] = 0;
    localPools[i] = new InstPool3(LocalPoolSize_);
    localPoolLocks[i] = new mutex();
  }

  for (int i = 0; i < HistTableSize_; i++) {
    HistTableLock[i] = new mutex();
  }

  results.assign(NumThreads_, RES_SUCCESS);
  MasterNodeCount_ = 0;

  InactiveThreads_ = 0;
  IsTimeoutPerInst_ = IsTimeoutPerInst;

  timeoutToMemblock_ = timeoutToMemblock;
                
  initWorkers(OST_, dataDepGraph, rgnNum, sigHashSize, lbAlg, hurstcPrirts, enumPrirts,
              vrfySched, PruningStrategy, SchedForRPOnly, enblStallEnum, SCW, spillCostFunc, TwoPassEnabled_,
              HeurSchedType, BestCost_, schedLwrBound_, enumBestSched_, &OptmlSpillCost_, 
              &bestSchedLngth_, GlobalPool, &MasterNodeCount_, HistTableLock, &GlobalPoolLock, &BestSchedLock, 
              &NodeCountLock, &ImprvCountLock, &RegionSchedLock, &results, idleTimes,
              NumSolvers_, localPools, localPoolLocks, &InactiveThreads_, &InactiveThreadLock, LocalPoolSize_, WorkSteal_, 
              &WorkStealOn_, IsTimeoutPerInst_, nodeCounts, timeoutToMemblock_, subspaceLwrBounds_);
  
  ThreadManager.resize(NumThreads_);

  Logger::Event("FinishedConstBBInterfacer");
}


BBMaster::~BBMaster() {
  delete GlobalPool;

  for (int i = 0; i < HistTableSize_; i++) {
    delete HistTableLock[i];
  }
  delete[] HistTableLock;

  for (int i = 0; i < NumThreads_; i++) {
    delete localPools[i];
    delete Workers[i];
    delete localPoolLocks[i];
  }

  delete[] localPoolLocks;
  delete[] idleTimes;
  delete[] subspaceLwrBounds_;
  delete[] nodeCounts;

  Logger::Info("finished deleting master");
}
/*****************************************************************************/

void BBMaster::initWorkers(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
             long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
             SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
             bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
             bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc, bool twoPassEnabled,
             SchedulerType HeurSchedType, InstCount *BestCost, InstCount schedLwrBound,
             InstSchedule *BestSched, InstCount *BestSpill, 
             InstCount *BestLength, InstPool4 *GlobalPool, 
             uint64_t *NodeCount, std::mutex **HistTableLock, std::mutex *GlobalPoolLock, std::mutex *BestSchedLock, 
             std::mutex *NodeCountLock, std::mutex *ImprvCountLock, std::mutex *RegionSchedLock,
             vector<FUNC_RESULT> *results, int *idleTimes,
             int NumSolvers, vector<InstPool3 *> localPools, std::mutex **localPoolLocks, int *inactiveThreads,
             std::mutex *inactiveThreadLock, int LocalPoolSize, bool WorkSteal, bool *WorkStealOn, bool IsTimeoutPerInst,
             uint64_t *nodeCounts, int timeoutToMemblock, int64_t **subspaceLwrBounds) {
  
  Workers.resize(NumThreads_);
  
  for (int i = 0; i < NumThreads_; i++) {
    Workers[i] = new BBWorker(OST_, dataDepGraph, rgnNum, sigHashSize, lbAlg, hurstcPrirts,
                                   enumPrirts, vrfySched, PruningStrategy, SchedForRPOnly, enblStallEnum, 
                                   SCW, spillCostFunc, twoPassEnabled, HeurSchedType, isSecondPass_, BestSched, BestCost, 
                                   BestSpill, BestLength, GlobalPool, NodeCount, i+2, HistTableLock, 
                                   GlobalPoolLock, BestSchedLock, NodeCountLock, ImprvCountLock, RegionSchedLock, 
                                   results, idleTimes, NumThreads_, localPools, localPoolLocks,
                                   inactiveThreads, inactiveThreadLock, LocalPoolSize, WorkSteal, WorkStealOn,
                                   IsTimeoutPerInst, nodeCounts, timeoutToMemblock, subspaceLwrBounds);
  }
}
/*****************************************************************************/
Enumerator *BBMaster::AllocEnumrtr_(Milliseconds timeout) {
  setWorkerHeurInfo();
  bool fsbl;
  Enumerator *enumrtr = NULL; 
  enumrtr = allocEnumHierarchy_(timeout, &fsbl);

  // TODO -- hacker hour, fix this
  return fsbl == false ? NULL : enumrtr;
}

/*****************************************************************************/
Enumerator *BBMaster::allocEnumHierarchy_(Milliseconds timeout, bool *fsbl) {
  bool enblStallEnum = EnblStallEnum_;


  // Master has ID of 1 (list has ID of 0)
  Enumrtr_ = new LengthCostEnumerator(this,
      dataDepGraph_, machMdl_, schedUprBound_, GetSigHashSize(),
      GetEnumPriorities(), GetPruningStrategy(), SchedForRPOnly_, enblStallEnum,
      timeout, GetSpillCostFunc(), isSecondPass_, NumThreads_, timeoutToMemblock_, 1, 0, NULL);

  Enumrtr_->setLCEElements(this, costLwrBound_);
  InitForSchdulng();
  // Master Enumerator has solverID of 1
  Enumrtr_->Initialize_(enumCrntSched_, schedLwrBound_, 1);

  Enumrtr_->checkTreeFsblty(*fsbl);
  if (!*fsbl) return nullptr;


  // Be sure to not be off by one - BBMaster is solver 0
  for (int i = 0; i < NumThreads_; i++) {
    Workers[i]->allocSched_();
    Workers[i]->allocEnumrtr_(timeout);
    Workers[i]->setLCEElements_(costLwrBound_);
    if (Enumrtr_->IsHistDom())
      Workers[i]->setEnumHistTable(getEnumHistTable());
    Workers[i]->setCostLowerBound(getCostLwrBound());
    Workers[i]->setMasterImprvCount(Enumrtr_->getImprvCnt());
    Workers[i]->setRegionSchedule(bestSched_);
  }

  if (Enumrtr_->IsHistDom()) {
    for (InstCount i = 0; i < Enumrtr_->totInstCnt_; i++) {
      SchedInstruction *masterInst = Enumrtr_->GetInstByIndx(i);

      for (int j = 0; j < NumThreads_; j++) {
        SchedInstruction *temp = Workers[j]->GetInstByIndex(i);
        assert(masterInst->GetNum() == temp->GetNum());
        temp->SetSig(masterInst->GetSig());
      }

    }
  }

  *fsbl = init();

  return Enumrtr_;
}
/*****************************************************************************/

bool BBMaster::initGlobalPool() {
  Logger::Info("init global pool");
  SPILL_COST_FUNCTION TempSCF = GetSpillCostFunc();

  
  // multiple diversity algorithms exist which are distinct in the way that
  // they adhere to diversity. 
  //
  // The most strict (FIXED POINT) prunes at initialization time so that 
  // the launched threads will not immedaitely prune and pick from GlobalPool
  // not according to diversity
  // 
  // The second most strict (EQUAL THREADS) splits until each primary subspace 
  // contains enough nodes to satisfy the requirement that each primary subspace
  // will receive an equal representation
  //
  // The least strict (current implementation) splits by BFS until 
  // number nodes > number threads. In launching threads, then, we do our best to 
  // maximize diversity, but there is no gauarantee that a primary subspace will 
  // receive at most one thread
  
  
  std::shared_ptr<HalfNode> temp, temp2;
  bool fsbl;
  std::shared_ptr<HalfNode> exploreNode(nullptr);


  SpillCostFunc_ = GlobalPoolSCF_;

  InstPool4 *firstInsts = new InstPool4;

  int depth = 1;
  Enumrtr_->splitNode(exploreNode, firstInsts, depth);
  assert(firstInsts);
  firstLevelSize_ = firstInsts->size();


  std::shared_ptr<HalfNode> loopTemp;
    for (int i = 0; i < firstLevelSize_; i++) {
      loopTemp = firstInsts->front();
      firstInsts->pop();
      firstInsts->push(loopTemp);
  }

  assert(firstLevelSize_ > 0);

  // if the global pool has a size that is initially >= 20 * numThreads we dont want
  // to incur overhead of splitting even once despite our minimums
  if (firstLevelSize_ < NumThreads_ * 20 && (
    firstLevelSize_ < NumThreads_ * MinNodesAsMultiple_ || MinSplittingDepth_ > 0)) {
    InstPool4 **diversityPools = new InstPool4*[firstLevelSize_];
    for (int i = 0; i < firstLevelSize_; i++) {
      diversityPools[i] = new InstPool4;
      temp = firstInsts->front();
      temp->setDiversityNum(i);
      diversityPools[i]->push(temp);
      firstInsts->pop();
    }
    int NumNodes = 0;
    while (depth <= MaxSplittingDepth_ && (depth <= MinSplittingDepth_ || NumNodes < NumThreads_ * MinNodesAsMultiple_)) {
      ++depth;
      NumNodes = 0;
      for (int i = 0; i < firstLevelSize_; i++) {
        int childrenAtPreviousDepth = diversityPools[i]->size();
        for (int k = 0; k < childrenAtPreviousDepth; k++) {
          exploreNode = diversityPools[i]->front();         
          exploreNode->setDiversityNum(i);
          diversityPools[i]->pop();
          Enumrtr_->splitNode(exploreNode, diversityPools[i], depth);
        }
        NumNodes += diversityPools[i]->size();
      }
    }

    if (NumNodes < NumThreads_) {
      Logger::Info("Not enough branching at top, dont parse");
    }

    int globalPoolDepth = depth;
    GlobalPool->setDepth(globalPoolDepth);

    for (int i = 0; i < firstLevelSize_; i++) {
      int j = 0;
      while (!diversityPools[i]->empty()) {
        j++;
        temp = diversityPools[i]->front();
        temp->setDiversityNum(i);
        diversityPools[i]->pop();
        GlobalPool->push(temp);
      }
      delete diversityPools[i];
    }
    delete diversityPools;

  }

  else {
    GlobalPool->setDepth(2);
    int i = 0;
    while (!firstInsts->empty()) {
      temp = firstInsts->front();
      assert(temp);
      firstInsts->pop();
      temp->setDiversityNum(i);
      ++i;
      GlobalPool->push(temp);
    }

  }

  delete firstInsts;


  GlobalPool->sort();

  
  Logger::Info("global pool has %d nodes", GlobalPool->size());
  MasterNodeCount_ += Enumrtr_->GetNodeCnt();
  SpillCostFunc_ = TempSCF;
  return true;


  /* FINISHED ALGORITHM FOR EQUAL THREADS
  // get the readyList snapshot after scheduling exploreNode.
  // return the insts as nodes for easy recreation of prefix, with the
  // priority attached for easy sorting
  InstPool *firstInsts = new InstPool;
  Enumrtr_->getRdyListAsNodes(exploreNode.first, firstInsts);
  firstLevelSize_ = firstInsts->size();
  int originalSize = firstLevelSize_;

  if (NumThreads_ > firstLevelSize_) {
    InstPool **diversityPools = new InstPool*[firstLevelSize_];
    int limit = (NumThreads_ % firstLevelSize_ == 0) ? NumThreads_ / firstLevelSize_ : (int) (NumThreads_ / firstLevelSize_) + 1;
    
    for (int i = 0; i < originalSize; i++) {
      diversityPools[i] = new InstPool;
      temp = firstInsts->front();
      assert(temp.first->getDiversityNum() == INVALID_VALUE);
      firstInsts->pop();
      temp.first->setDiversityNum(i);
      diversityPools[i]->push(temp);

      while (diversityPools[i]->size() < limit) {
        temp = diversityPools[i]->front();
        diversityPools[i]->pop();
        Enumrtr_->getRdyListAsNodes(temp.first, diversityPools[i]);
      }

      while (!diversityPools[i]->empty()) {
        temp = diversityPools[i]->front();
        temp.first->setDiversityNum(i);
        diversityPools[i]->pop();
        GlobalPool->push(temp);
      }
    }
    delete[] diversityPools;
  }


  else {
    int i = 0;
    while (!firstInsts->empty()) {
      temp = firstInsts->front();
      firstInsts->pop();
      temp.first->setDiversityNum(i);
      ++i;
      GlobalPool->push(temp);
    }
  }

  delete firstInsts;

  GlobalPool->sort();

  MasterNodeCount_ += Enumrtr_->GetNodeCnt();

  return true;
  */


  
  /* FIXED POINT INNARDS
  // ensure we have enough nodes in each subtree to allow us to ensure diversity
  if (NumThreads_ > firstLevelSize_) {
    InstPool *diversityPools[firstLevelSize_];
    bool fsblSubtree[firstLevelSize_];
    bool prunedTree = false;
    int limit = (NumThreads_ % firstLevelSize_ == 0) ? NumThreads_ / firstLevelSize_ : (int) (NumThreads_ / firstLevelSize_) + 1;
    
    for (int i = 0; i <= originalSize; i++) {
      diversityPools[i] = new InstPool;
      fsblSubtree[i] = true;
      temp = firstInsts->front();
      firstInsts->pop();
      temp.first->setDiversityNum(i);
      if (!Enumrtr_->isFsbl(temp.first, false)) {
        firstLevelSize_ -= 1;
        prunedTree = true;
        if (firstLevelSize_ == 0) return false; // we have pruned all nodes
        continue;
      }

      diversityPools[i]->push(temp);

      while (diversityPools[i]->size() < limit) {
        if (diversityPools[i]->size() == 0) {
          fsblSubtree[i] = false;
          firstLevelSize_ -= 1;
          prunedTree = true;
          if (firstLevelSize_ == 0) return false; // we have pruned all nodes
          break;
        }
        temp = diversityPools[i]->front();
        diversityPools[i]->pop();
        if (Enumrtr_->isFsbl(temp.first, false)) {
          Enumrtr_->getRdyListAsNodes(temp.first, diversityPools[i]);
        }
      }
    }

    // possible to prune subtrees here as well
    // need to implement fixed point alg to continue expanding until no more prunings
    // for now -- assume no prunings here
    if (prunedTree) limit = (NumThreads_ % firstLevelSize_ == 0) ? NumThreads_ / firstLevelSize_ : (int) (NumThreads_ / firstLevelSize_) + 1;
      
    for (int i = 0; i < originalSize; i++) {
      if (fsblSubtree[i])  {
        while (diversityPools[i]->size() < limit) {
          temp = diversityPools[i]->front();
          diversityPools[i]->pop();
          Enumrtr_->getRdyListAsNodes(temp.first, diversityPools[i]);
        }
 
        for (int j = 0; j < diversityPools[i]->size(); j++) {
          temp = diversityPools[i]->front();
          diversityPools[i]->pop();
          temp.first->setDiversityNum(i);
          GlobalPool->push(temp); // we are purposefully avoiding checking the fsblty
        }
      }
      delete diversityPools[i];
    }
  }

  else {
    int i = 0;
    while (!firstInsts->empty()) {
      temp = firstInsts->front();
      firstInsts->pop();

      if (!Enumrtr_->isFsbl(temp.first, false)) {
        firstLevelSize_ -= 1;
        if (firstLevelSize_ == 0) return false; // we have pruned all nodes
        continue;
      }

      temp.first->setDiversityNum(i);
      ++i;
      GlobalPool->push(temp);
    }


    InstPool *fillQueue = new InstPool;

    while (GlobalPool->size() < NumThreads_) {
      exploreNode = GlobalPool->front();
      GlobalPool->pop();
      Enumrtr_->getRdyListAsNodes(exploreNode.first, fillQueue);
      while (!fillQueue->empty()) {
        std::pair<EnumTreeNode *, unsigned long> temp;
        fillQueue->pop();
        temp.first->setDiversityNum(exploreNode.first->getDiversityNum());
        if (Enumrtr_->isFsbl(temp.first, false))
          GlobalPool->push(temp);
      }
    }

    delete fillQueue;
  }

  delete firstInsts;

  GlobalPool->sort();
  return true;
  */
}
/*****************************************************************************/

bool BBMaster::init() {

  for (int i = 0; i < NumThreads_; i++) {
    Workers[i]->setLowerBounds_(StaticSlilLowerBound_);
    Workers[i]->setupForSchdulng();
    Workers[i]->initForSchdulng();
  }


  for (int i = 0; i < NumThreads_; i++) {
    Workers[i]->initEnumrtr_();
  }

  return initGlobalPool() == false ? false : true;
}
/*****************************************************************************/

void BBMaster::setWorkerHeurInfo() {
  for (int i = 0; i < NumThreads_; i++) {
    Workers[i]->setHeurInfo(schedUprBound_, getHeuristicCost(), schedLwrBound_);
  }
}
/*****************************************************************************/

FUNC_RESULT BBMaster::Enumerate_(Milliseconds startTime, Milliseconds rgnTimeout,
                                 Milliseconds lngthTimeout, int *OptimalSolverID) {
                              


 std::shared_ptr<HalfNode> Temp;

  for (int i = 0; i < NumThreads_; i++) { 
    Workers[i]->setMasterSched(enumBestSched_);
  }

  std::vector<std::shared_ptr<HalfNode>> LaunchNodes(NumThreads_);
  int NumNodesPicked = 0;

  bool *subspaceRepresented;
  subspaceRepresented = new bool[firstLevelSize_];
  

  Logger::Info("using exploitationPercent %f", ExploitationPercent_);
  int exploitationCount;
  exploitationCount =  NumThreads_ - (NumThreads_ * (1 - ExploitationPercent_));
  int globalPoolSizeStart = GlobalPool->size();

  if (globalPoolSizeStart < NumThreads_) {
    NumThreadsToLaunch_ = globalPoolSizeStart;
    Logger::Info("we were not able to find enough global pool nodes");
    Logger::Info("Launching %d threads", NumThreadsToLaunch_);

    for (int i = 0; i < NumThreadsToLaunch_; i++) {
      Temp = GlobalPool->front();
      GlobalPool->pop();
      LaunchNodes[i] = Temp;
    }
  }


  else {
    Logger::Event("LaunchingParallelThreads", "num", NumThreads_);
    NumThreadsToLaunch_ = NumThreads_;
  while (NumNodesPicked < NumThreads_) {
    for (int i = 0; i < firstLevelSize_; i++) {
      subspaceRepresented[i] = false;
    }

    // exploitation
    for (int j = 0; j < exploitationCount; j++) {
      Temp = GlobalPool->front();
      GlobalPool->pop();
      int x = Temp->getDiversityNum();
      subspaceRepresented[x] = true;
      LaunchNodes[NumNodesPicked] = Temp;
      NumNodesPicked += 1;
      if (NumNodesPicked >= NumThreads_) break;
    }

    if (NumNodesPicked >= NumThreads_) break;

    // exploration
    int GlobalPoolSize = GlobalPool->size();
    for (int i = 0; i < GlobalPoolSize; i++) {
      Temp = GlobalPool->front();
      GlobalPool->pop();
      int x = Temp->getDiversityNum();
      if (!subspaceRepresented[x]) {
        LaunchNodes[NumNodesPicked] = Temp;
        NumNodesPicked += 1;
        if (NumNodesPicked >= NumThreads_) break;
        subspaceRepresented[x] = true;
      } 
      else GlobalPool->push(Temp);
    }
  }
  }

  mallopt(M_MMAP_THRESHOLD, 128*1024);
  mallopt(M_ARENA_MAX, NumSolvers_ * 2);
  //mallopt(M_ARENA_TEST, 8);

    cpu_set_t cpuset;
  for (int j = 0; j < NumThreadsToLaunch_; j++) {
#ifdef DEBUG_GP_HISTORY
    Logger::Info("SolverID %d launching GlobalPoolNode with inst %d (parent %d)", j+2, LaunchNodes[j]->GetInstNum(), LaunchNodes[j]->prefix_.back()->GetInstNum());
#endif
    ThreadManager[j] = std::thread([=]{Workers[j]->generateAndEnumerate(LaunchNodes[j], startTime, rgnTimeout, lngthTimeout);});
    
    // We want to map each thread to a seperate core. If we let the OS scheduler decide
    // it will potentially map two threads to the same core (e.g. concurrency) which
    // results in threads unnecessarily competing for resources (e.g. L1 cache).
    // This code is architecture specific. Currently, the code assumes that
    // there are two threads per core, and the logical CPUs (label via /proc/cpuinfo) 
    // that share a core are offset by the number of cores.
    // For example, in a 4 core with 2 threads per core machine, this code assumes
    // that Processor 0 and 4 share the first core (POSIX)
    CPU_ZERO(&cpuset);
    CPU_SET(j, &cpuset);
    CPU_SET(j+NumThreads_, &cpuset); //assume 2 threads per core
    int rc = pthread_setaffinity_np(ThreadManager[j].native_handle(),
                                    sizeof(cpu_set_t), &cpuset);
  }

  for (int j = NumThreadsToLaunch_; j < NumThreads_; j++) {
    std::shared_ptr<HalfNode> nullNode = NULL;
    ThreadManager[j] = std::thread([=]{Workers[j]->generateAndEnumerate(nullNode, startTime,rgnTimeout,lngthTimeout);});
    CPU_ZERO(&cpuset);
    CPU_SET(j, &cpuset);
    CPU_SET(j+NumThreads_, &cpuset);
    int rc = pthread_setaffinity_np(ThreadManager[j].native_handle(),
                                    sizeof(cpu_set_t), &cpuset);
  }



  for (int j = 0; j < NumThreads_; j++) {
    ThreadManager[j].join();
    
    Logger::Info("Solver %d stats: ", j+2);
    Logger::Info("StepFrwrds %d" , Workers[j]->StepFrwrds);
    Logger::Info("BackTracks %d", Workers[j]->BackTracks);
    Logger::Info("CostInfsbl %d", Workers[j]->CostInfsbl);
    Logger::Info("HistInfsbl %d" , Workers[j]->HistInfsbl);
    Logger::Info("OtherInfsbl %d", Workers[j]->OtherInfsbl);
    Logger::Info("GlobalPoolNodes %d", Workers[j]->GlobalPoolNodes);
    
  }



  for (int j = 0; j < NumThreads_; j++) {
    Milliseconds endTime = Utilities::GetProcessorTime();
    Milliseconds temp = endTime - idleTimes[j];
    Logger::Info("Idle time for solver %d: %d", j + 1, temp);
  }

  for (int j = 0; j < NumThreads_; j++) {
    Logger::Info("Nodes examined for solver %d: %d", j + 1, nodeCounts[j]);
  }


  int globalPoolSizeEnd = GlobalPool->size();

  Logger::Event("GlobalPoolNodesExplored", "num", globalPoolSizeStart - globalPoolSizeEnd);



  /*  ALGORITHM FOR FAIR REPRESENTATION
  int *ExploitationCount = new int[firstLevelSize_];
  std::fill(ExploitationCount, ExploitationCount + firstLevelSize_ * sizeof(int), 0);

  // the max amount of threads that can be sent to the same diversity num
  int ExploitationLimit;
  int firstLevelNodesLeft = firstLevelSize_;
  int NumThreadsLeft = NumThreads_;

  int i = 0;
  while (i < NumThreads_) {
    ExploitationLimit = (NumThreadsLeft % firstLevelNodesLeft == 0) ?
                          NumThreadsLeft / firstLevelNodesLeft : 
                          (NumThreadsLeft / firstLevelNodesLeft) + 1; 
    Temp = GlobalPool->front();
    GlobalPool->pop();

    if (ExploitationCount[Temp.first->getDiversityNum()] == ExploitationLimit) {
        // we have already fully exploited this diversity node
        // e.g. completed a prior iteration of inner while loop
        // can only happen if more than one iteration of non divisible NumThreadsLeft (e.g. 10 thread, 3 div nodes)
        GlobalPool->push(Temp);
        continue;
    }

    if (false)
      Logger::Info("Probing inst %d", Temp.first->GetInstNum());

    if (false)
      Logger::Info("Stepping forward to inst %d", Temp.first->GetInstNum());

    LaunchNodes[i] = Temp.first;
    i++;

    ExploitationCount[Temp.first->getDiversityNum()] += 1;
    if (ExploitationCount[Temp.first->getDiversityNum()] == ExploitationLimit) {
      firstLevelNodesLeft -= 1;
      NumThreadsLeft -= ExploitationLimit;
    }

  }

  assert(NumThreadsLeft == 0);
  assert(firstLevelNodesLeft + NumThreads_ == firstLevelSize_);

  
  for (int j = 0; j < NumThreads_; j++) {
    ThreadManager[i] = std::thread(&BBWorker::enumerate_, Workers[i], LaunchNodes[j], startTime, rgnTimeout, lngthTimeout);
  }

  for (int j = 0; j < NumThreads_; j++) {
    ThreadManager[j].join();
  }

  delete[] LaunchNodes;
  delete ExploitationCount;
  */


  // TODO remove optimalSolverID
  *OptimalSolverID = 1; //master schedule
  
  if (enumBestSched_->GetSpillCost() < bestSched_->GetSpillCost() && *Enumrtr_->getImprvCnt() > 0)
  {
    bestSched_ = enumBestSched_;
  }


  Enumrtr_->Reset();
  
  vector<std::thread> ThreadManager2(NumThreads_);
  for (int j = 0; j < NumThreads_; j++) {
    ThreadManager2[j] = std::thread([=]{Workers[j]->freeAlctrs();});
  }

  for (int j = 0; j < NumThreads_; j++) {
    ThreadManager2[j].join();
  }
  
  


  bool timeoutFlag = false;
  for (int j = 0; j < NumThreads_; j++) {
    if (results[j] == RES_ERROR) return RES_ERROR;
    if (results[j] == RES_TIMEOUT) timeoutFlag = true;
  }

  return timeoutFlag ? RES_TIMEOUT : RES_SUCCESS;

}
