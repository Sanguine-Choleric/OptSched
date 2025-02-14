/*******************************************************************************
Description:  Defines a history table class.
Author:       Ghassan Shobaki
Created:      Unknown
Last Update:  Mar. 2011
*******************************************************************************/

#ifndef OPTSCHED_ENUM_HIST_TABLE_H
#define OPTSCHED_ENUM_HIST_TABLE_H

#include "opt-sched/Scheduler/defines.h"
#include "opt-sched/Scheduler/enumerator.h"
#include "opt-sched/Scheduler/gen_sched.h"
#include "opt-sched/Scheduler/hash_table.h"
#include "opt-sched/Scheduler/mem_mngr.h"
#include <cstdio>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>
#include <stack>

namespace llvm {
namespace opt_sched {


class EnumTreeNode;
class Enumerator;

// The history version of a tree node to be kept in the history table
class HistEnumTreeNode {
public:
  HistEnumTreeNode();
  virtual ~HistEnumTreeNode();

  InstCount GetTime();
  void PrntPartialSched(std::ostream &out);
  bool CompPartialScheds(HistEnumTreeNode *othrHist);
  InstCount GetInstNum();
  bool IsPrdcsrViaStalls(HistEnumTreeNode *othrNode);
  HistEnumTreeNode *GetParent();
  void Clean();
  void ReplaceParent(HistEnumTreeNode *newParent);
  // Does the scheduled inst. list of this node match that of the given node
  bool DoesMatch(EnumTreeNode *node, Enumerator *enumrtr, bool isWorker, bool isGlobalPoolNode);
  // Is the sub-problem at this node dominated by the given node's?
  bool IsDominated(EnumTreeNode *node, Enumerator *enumrtr);
  // Does the sub-problem at this node dominate the given node's?
  virtual bool DoesDominate(EnumTreeNode *node, Enumerator *enumrtr);
  virtual void Construct(EnumTreeNode *node, bool isTemp, bool isGenerateState, bool setCost = true);
  virtual void SetCostInfo(EnumTreeNode *node, bool isTemp,
                           Enumerator *enumrtr);
  virtual void ResetHistFields(EnumTreeNode *node);
  const std::shared_ptr<std::vector<SchedInstruction *>> &GetSuffix() const;
  void
  SetSuffix(const std::shared_ptr<std::vector<SchedInstruction *>> &suffix);
  std::vector<InstCount> GetPrefix() const;

  inline SchedInstruction *GetInst() {return inst_;}

  inline void setFullyExplored(bool isFullyExplored) {
    fullyExplored_ = isFullyExplored;
  }

  inline bool getFullyExplored() {return fullyExplored_;}

  inline void setCostIsUseable(bool isCostAbsoluteBest) {
    totalCostIsUseable_ = isCostAbsoluteBest;
  }

  inline bool getCostIsUseable() {return totalCostIsUseable_;}
  bool isTemp_;

  void Copy(HistEnumTreeNode *other);

  inline int getInstNum() {return inst_ ? inst_->GetNum() : -1;}

  inline bool isRecycled() {return recycled_;}

  inline void setRecycled(bool recycled) {recycled_ = recycled;}

  inline bool isInserted() {return isInserted_;}

  inline void setInserted(bool inserted) {isInserted_ = inserted;}

protected:
  HistEnumTreeNode *prevNode_;

  // The current time or position (or step number) in the scheduling process.
  // This is equal to the length of the path from the root node to this node.
  InstCount time_;

  SchedInstruction *inst_;

  bool fullyExplored_ = false;
  bool totalCostIsUseable_ = false;
  bool archived_ = false;
  bool recycled_ = false;
  bool isInserted_ = false;

#ifdef IS_DEBUG
  bool isCnstrctd_;
#endif

  bool crntCycleBlkd_;
  ReserveSlot *rsrvSlots_;

  // (Chris)
  std::shared_ptr<std::vector<SchedInstruction *>> suffix_ = nullptr;



  InstCount SetLastInsts_(SchedInstruction *lastInsts[], InstCount thisTime,
                          InstCount minTimeToExmn);

  bool SetBothInstsSchduld_(BitVector *thisInstsSchuld, BitVector *therInstsSchuld, HistEnumTreeNode *otherHist, bool isWorker);
  bool checkSameSubspace_(EnumTreeNode *otherNode);
  void SetInstsSchduld_(BitVector *instsSchduld, bool isWorker);
  // Does this history node dominate the given node or history node?
  bool DoesDominate_(EnumTreeNode *node, HistEnumTreeNode *othrHstry,
                     ENUMTREE_NODEMODE mode, Enumerator *enumrtr,
                     InstCount shft);
  void SetLwrBounds_(InstCount lwrBounds[], SchedInstruction *lastInsts[],
                     InstCount thisTime, InstCount minTimeToExmn,
                     Enumerator *enumrtr);
  void CmputNxtAvlblCycles_(Enumerator *enumrtr, InstCount instsPerType[],
                            InstCount nxtAvlblCycles[]);

  virtual void Init_();
  void AllocLastInsts_(ArrayMemAlloc<SchedInstruction *> *lastInstsAlctr,
                       Enumerator *enumrtr);
  bool IsAbslutDmnnt_();
  InstCount GetMinTimeToExmn_(InstCount nodeTime, Enumerator *enumrtr);
  InstCount GetLwrBound_(SchedInstruction *inst, int16_t issuRate);
  void SetRsrvSlots_(EnumTreeNode *node);

};

class CostHistEnumTreeNode : public HistEnumTreeNode {
public:
  CostHistEnumTreeNode();
  virtual ~CostHistEnumTreeNode();

  void Construct(EnumTreeNode *node, bool isTemp, bool isGenerateState, bool setCost = true) override;
  // Does the sub-problem at this node dominate the given node's?
  bool DoesDominate(EnumTreeNode *node, Enumerator *enumrtr) override;
  void SetCostInfo(EnumTreeNode *node, bool isTemp, Enumerator *enumrtr) override;
  void ResetHistFields(EnumTreeNode *node) override;


  inline void setTotalCostFromLB(InstCount totalCost) {
    totalCost_ = totalCost;
    totalCostIsUseable_ = true;
  }

  inline InstCount getTotalCost() {return totalCost_;}
  inline InstCount getPartialCost() {return partialCost_;}


protected:
  // Why do we need to copy this data from region->tree_node->hist_node
  InstCount cost_;
  InstCount peakSpillCost_;
  InstCount spillCostSum_;

  // (Chris)
  InstCount totalCost_ = INVALID_VALUE;
  InstCount partialCost_ = INVALID_VALUE;
  bool totalCostIsActualCost_ = false;

  bool isLngthFsbl_;
  bool costInfoSet_ = false;

  bool ChkCostDmntnForBBSpill_(EnumTreeNode *node, Enumerator *enumrtr);
  bool ChkCostDmntn_(EnumTreeNode *node, Enumerator *enumrtr,
                     InstCount &maxShft);
  virtual void Init_() override;
};

} // namespace opt_sched
} // namespace llvm

#endif
