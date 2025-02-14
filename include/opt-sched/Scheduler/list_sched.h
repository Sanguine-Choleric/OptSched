/*******************************************************************************
Description:  Defines a list scheduler, based on the defintions of a generic
              scheduler and a constrained scheduler in gen_sched.h, which
              collectively include the meat of the implementation.
Author:       Ghassan Shobaki
Created:      Apr. 2002
Last Update:  Sept. 2013
*******************************************************************************/

#ifndef OPTSCHED_LIST_SCHED_LIST_SCHED_H
#define OPTSCHED_LIST_SCHED_LIST_SCHED_H

#include "opt-sched/Scheduler/gen_sched.h"

namespace llvm {
namespace opt_sched {

class ListScheduler : public ConstrainedScheduler {
public:
  // Creates a list scheduler for the given dependence graph, machine and
  // schedule upper bound, using the specified heuristic.
  ListScheduler(DataDepGraph *dataDepGraph, MachineModel *machMdl,
                InstCount schedUprBound, SchedPriorities prirts);
  virtual ~ListScheduler();

  // Calculates the schedule and returns it in the passed argument.
  FUNC_RESULT FindSchedule(InstSchedule *sched, SchedRegion *rgn) override;

protected:
  bool isDynmcPrirty_;
  // Adds the instructions that have just become ready at this cycle to the
  // ready list.
  void UpdtRdyLst_(InstCount cycleNum, int slotNum) override;

  // Check whether the next node ID instruction is ready -- used to collect
  // scheduling stats for LLVM generating schedules
  bool CheckForInst(int numToPick) const;

  // Pick next instruction to be scheduled. Returns NULL if no instructions are
  // ready.
  virtual SchedInstruction *PickInst() const;
};

// Force the list scheduler to maintain the source ordering of the instructions
// regardless of latency or machine model constraints.
class SequentialListScheduler : public ListScheduler {
public:
  SequentialListScheduler(DataDepGraph *dataDepGraph, MachineModel *machMdl,
                          InstCount schedUprBound, SchedPriorities prirts);

private:
  // Does this instruction come next in the source ordering after all currently
  // scheduled instructions, e.g. 0, 1, 2, 3, 4.
  bool IsSequentialInstruction(const SchedInstruction *Inst) const;

  bool ChkInstLglty_(SchedInstruction *inst) const override;
};

} // namespace opt_sched
} // namespace llvm

#endif
