#include "opt-sched/Scheduler/reg_alloc.h"
#include "opt-sched/Scheduler/data_dep.h"
#include "opt-sched/Scheduler/logger.h"
#include "opt-sched/Scheduler/register.h"
#include "opt-sched/Scheduler/sched_basic_data.h"
#include <climits>
#include <utility>

using namespace llvm::opt_sched;

LocalRegAlloc::LocalRegAlloc(InstSchedule *instSchedule,
                             DataDepGraph *dataDepGraph) {
  instSchedule_ = instSchedule;
  dataDepGraph_ = dataDepGraph;
}

LocalRegAlloc::~LocalRegAlloc() {}

void LocalRegAlloc::AllocRegs() {
#ifdef IS_DEBUG
  Logger::Info("REG_ALLOC: Starting LocalRegAlloc.");
#endif

  InstCount cycle, slot;

  int entryInstNum = instSchedule_->GetFrstInst(cycle, slot);
  SchedInstruction *entryInst = dataDepGraph_->GetInstByIndx(entryInstNum);
  assert(!strcmp(entryInst->GetOpCode(), "__optsched_entry"));
  // Allocate live-in registers.
  AddLiveIn_(entryInst);

  for (InstCount i = instSchedule_->GetFrstInst(cycle, slot);
       i != INVALID_VALUE; i = instSchedule_->GetNxtInst(cycle, slot)) {
    int instNum = i;
    SchedInstruction *inst = dataDepGraph_->GetInstByIndx(instNum);
#ifdef RA_BUG
    Logger::Info("\nParsing inst");
    inst->printMIR();
#endif

    // Skip artificial entry and exit nodes.
    if (!strcmp(inst->GetOpCode(), "__optsched_entry") ||
        !strcmp(inst->GetOpCode(), "__optsched_exit")) {
      Logger::Info("skipping artificial inst");
      continue;
    }

#ifdef IS_DEBUG_REG_ALLOC
    Logger::Info("REG_ALLOC: Processing instruction %d.", instNum);
#endif

    // Find registers for this instruction's VReg uses.
    for (Register *use : inst->GetUses()) {
      int16_t regType = use->GetType();
      int virtRegNum = use->GetNum(0);
#ifdef RA_BUG
      Logger::Info("found use %d", virtRegNum);
#endif
      RegMap &map = regMaps_[regType][virtRegNum];
#ifdef IS_DEBUG_REG_ALLOC
      Logger::Info("REG_ALLOC: Processing use for register %d:%d.", regType,
                   virtRegNum);
#endif

      if (map.assignedReg == -1) {
#ifdef IS_DEBUG_REG_ALLOC
        Logger::Info("REG_ALLOC: Adding load for register %d:%d.", regType,
                     virtRegNum);
#endif
        numLoads_++;
        AllocateReg_(regType, virtRegNum);
      }
    }

    // Kill registers if this is the last use for them.
    for (Register *use : inst->GetUses()) {
      int16_t regType = use->GetType();
      int virtRegNum = use->GetNum(0);
      RegMap &map = regMaps_[regType][virtRegNum];
      std::vector<int> &physRegs = physRegs_[regType];

      assert(map.nextUses.front() == instNum);
      map.nextUses.pop();

      if (map.nextUses.empty() && map.assignedReg != -1) {
        int physRegNum = map.assignedReg;
#ifdef RA_BUG
        Logger::Info("no more uses for %d, freeing pr %d", virtRegNum,
        physRegNum);
#endif
        assert(physRegs[physRegNum] == virtRegNum);
        map.assignedReg = -1;
        map.isDirty = false;
        physRegs[physRegNum] = -1;
        freeRegs_[regType].push(physRegNum);
      }
    }

    // Process definitions.
    for (Register *def : inst->GetDefs()) {
      int16_t regType = def->GetType();
      int virtRegNum = def->GetNum(0);
#ifdef RA_BUG
      Logger::Info("found def %d", virtRegNum);
#endif
#ifdef IS_DEBUG_REG_ALLOC
      Logger::Info("REG_ALLOC: Processing def for register %d:%d.", regType,
                   virtRegNum);
#endif
      AllocateReg_(regType, virtRegNum);
    }
  }

  // Spill all registers that are still live (live-out) and are dirty.
  SpillAll_();
}

void LocalRegAlloc::AllocateReg_(int16_t regType, int virtRegNum) {
#ifdef RA_BUG
  Logger::Info("allocating reg for %d", virtRegNum);
#endif
  std::map<int, RegMap> &regMaps = regMaps_[regType];
  std::stack<int> &free = freeRegs_[regType];
  std::vector<int> &physRegs = physRegs_[regType];
  int physRegNum = -1;

  if (!free.empty()) {
    physRegNum = free.top();
    regMaps[virtRegNum].assignedReg = free.top();
    physRegs[physRegNum] = virtRegNum;
#ifdef RA_BUG
    Logger::Info("found free phys reg, assigning vr %d to pr %d", virtRegNum,
                 physRegNum);
#endif
    free.pop();
  } else {
    // If there are no free registers find one to use.
    int spillCand = FindSpillCand_(regMaps, physRegs);
    if (regMaps[spillCand].isDirty) {
#ifdef IS_DEBUG_REG_ALLOC
      Logger::Info("REG_ALLOC: Adding store for register %d:%d.", regType,
                   spillCand);
#endif
      numStores_++;
    } else {
#ifdef IS_DEBUG_REG_ALLOC
      Logger::Info("REG_ALLOC: Found clean register to use %d:%d.", regType,
                   spillCand);
#endif
    }

    physRegNum = regMaps[spillCand].assignedReg;
#ifdef RA_BUG
    Logger::Info("found spill cand, vr %d, pr %d", spillCand, physRegNum);
    if (physRegNum == -1)
      Logger::Info("about to fire assert, spillCand %d, regType %d", spillCand,
                   regType);
#endif
    assert(physRegNum != -1);
    regMaps[spillCand].assignedReg = -1;
    regMaps[virtRegNum].assignedReg = physRegNum;
    physRegs[physRegNum] = virtRegNum;
  }
#ifdef IS_DEBUG_REG_ALLOC
  Logger::Info("REG_ALLOC: Mapping virtual register %d:%d to %d:%d", regType,
               virtRegNum, regType, physRegNum);
#endif
  regMaps[virtRegNum].isDirty = true;
}

int LocalRegAlloc::FindSpillCand_(std::map<int, RegMap> &regMaps,
                                  std::vector<int> &physRegs) {
  int max = INT_MIN;
  int virtRegWithMaxUse = -1;
  for (size_t i = 0; i < physRegs.size(); i++) {
    int virtReg = physRegs[i];
#ifdef RA_BUG
    if (virtReg == -1) {
      Logger::Info("virtReg == - 1");
      dataDepGraph_->printMF();
    }
#endif
    assert(virtReg != -1);
    RegMap &regMap = regMaps[virtReg];

#ifdef RA_BUG
    if (regMap.assignedReg != i) {
      Logger::Info("regMap.assignedReg != i");
      Logger::Info("virtReg %d, i %d, regMap.assignedReg %d", virtReg, i,
                   regMap.assignedReg);
      dataDepGraph_->printMF();
    }
#endif

    assert(regMap.assignedReg == i);

    // If this register is clean, it can be spilled immediately .
    if (!regMap.isDirty) {
#ifdef IS_DEBUG_REG_ALLOC
      Logger::Info("REG_ALLOC: Found clean register to use %d.", virtReg);
#endif
      return virtReg;
    }

    int next = instSchedule_->GetSchedCycle(regMap.nextUses.front());
    if (next > max) {
      max = next;
      virtRegWithMaxUse = virtReg;
    }
  }
#ifdef IS_DEBUG_REG_ALLOC
  Logger::Info("REG_ALLOC: Register with the latest use %d.",
               virtRegWithMaxUse);
#endif
  return virtRegWithMaxUse;
}

void LocalRegAlloc::SetupForRegAlloc() {
  numLoads_ = 0;
  numStores_ = 0;
  numRegTypes_ = dataDepGraph_->GetRegTypeCnt();
#ifdef IS_DEBUG_REG_ALLOC
  Logger::Info("REG_ALLOC: Found %d register types.", numRegTypes_);
#endif

  // Initialize a free register stack for each register type
  freeRegs_.resize(numRegTypes_);
  physRegs_.resize(numRegTypes_);
  for (int i = 0; i < numRegTypes_; i++)
    for (int j = 0; j < dataDepGraph_->GetPhysRegCnt(i); j++) {
      freeRegs_[i].push(j);
      physRegs_[i].push_back(-1);
    }

  // Initialize list of register's next uses.
  vector<vector<stack<int>>> nextUse_;
  regMaps_.resize(numRegTypes_);
  ScanUses_();
}

void LocalRegAlloc::ScanUses_() {
  InstCount cycle, slot;
  for (InstCount i = instSchedule_->GetFrstInst(cycle, slot);
       i != INVALID_VALUE; i = instSchedule_->GetNxtInst(cycle, slot)) {
    SchedInstruction *inst = dataDepGraph_->GetInstByIndx(i);

    // Skip artificial entry node.
    if (!strcmp(inst->GetOpCode(), "__optsched_entry"))
      continue;

    int instNum = i;
#ifdef IS_DEBUG_REG_ALLOC
    Logger::Info("REG_ALLOC: Scanning for uses for instruction %d.", instNum);
#endif

    for (Register *use : inst->GetUses()) {
      int virtRegNum = use->GetNum(0);
      int16_t regType = use->GetType();
      if (regMaps_[regType].find(virtRegNum) == regMaps_[regType].end()) {
        RegMap m;
        m.nextUses.push(instNum);
        m.assignedReg = -1;
        m.isDirty = false;
        regMaps_[regType][virtRegNum] = m;
      } else {
        regMaps_[regType][virtRegNum].nextUses.push(instNum);
      }
#ifdef IS_DEBUG_REG_ALLOC
      Logger::Info("REG_ALLOC: Adding use for instruction %d.", instNum);
      Logger::Info("REG_ALLOC: Use list for register %d:%d is now:", regType,
                   virtRegNum);
      std::queue<int> next = regMaps_[regType][virtRegNum].nextUses;
      while (!next.empty()) {
        Logger::Info("%d", next.front());
        next.pop();
      }
#endif
    }
  }
}

void LocalRegAlloc::AddLiveIn_(SchedInstruction *artificialEntry) {
  // Process live-in regs.
#ifdef RA_BUG
  Logger::Info("Parsing live ins");
#endif
  for (Register *def : artificialEntry->GetDefs()) {
    int16_t regType = def->GetType();
    int virtRegNum = def->GetNum(0);
#ifdef RA_BUG
    Logger::Info("Found live in def vreg %d", virtRegNum);
#endif
#ifdef IS_DEBUG_REG_ALLOC
    Logger::Info("REG_ALLOC: Processing live-in register %d:%d.", regType,
                 virtRegNum);
#endif
    std::map<int, RegMap> &regMaps = regMaps_[regType];
    std::stack<int> &free = freeRegs_[regType];
    std::vector<int> &physRegs = physRegs_[regType];
    int physRegNum = -1;

    if (!free.empty()) {
      physRegNum = free.top();
      regMaps[virtRegNum].assignedReg = physRegNum;
      physRegs[physRegNum] = virtRegNum;
#ifdef RA_BUG
      Logger::Info("found free phs reg, virtReg %d assigned to physReg %d",
                   virtRegNum, physRegNum);
#endif
      free.pop();
    } else {
#ifdef IS_DEBUG_REG_ALLOC
      Logger::Info("Too many live-in registers to allocate them all at once. "
                   "register is %d:%d.",
                   regType, virtRegNum);
#endif
    }
  }
}

void LocalRegAlloc::SpillAll_() {
  for (int regType = 0; regType < numRegTypes_; regType++) {
    for (size_t j = 0; j < physRegs_[regType].size(); j++) {
      int physReg = physRegs_[regType][j];
      if (physReg != -1 && regMaps_[regType][physReg].isDirty)
        numStores_++;
    }
  }
}

void LocalRegAlloc::PrintSpillInfo(const char *dagName) {
  Logger::Info("OPT_SCHED LOCAL RA: DAG Name: %s Number of spills: %d", dagName,
               numLoads_ + numStores_);
  Logger::Info("Number of stores %d", numStores_);
  Logger::Info("Number of loads %d", numLoads_);
}

int LocalRegAlloc::GetCost() const { return numLoads_ + numStores_; }