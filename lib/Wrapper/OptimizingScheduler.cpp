//===- OptimizingScheduler.cpp - The optimizing scheduler -----------------===//
//
// Implements a combinatorial scheduling algorithm.
//
//===----------------------------------------------------------------------===//

#include "OptSchedMachineWrapper.h"
#include "opt-sched/Scheduler/OptSchedDDGWrapperBase.h"
#include "opt-sched/Scheduler/OptSchedTarget.h"
#include "opt-sched/Scheduler/bb_thread.h"
#include "opt-sched/Scheduler/config.h"
#include "opt-sched/Scheduler/data_dep.h"
#include "opt-sched/Scheduler/enumerator.h"
#include "opt-sched/Scheduler/graph_trans.h"
#include "opt-sched/Scheduler/hist_table.h"
#include "opt-sched/Scheduler/random.h"
#include "opt-sched/Scheduler/register.h"
#include "opt-sched/Scheduler/sched_region.h"
#include "opt-sched/Scheduler/utilities.h"
#include "OptSchedDDGWrapperBasic.h"
#include "OptSchedMachineWrapper.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/MachineInstrBundle.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineScheduler.h"
#include "llvm/CodeGen/RegisterClassInfo.h"
#include "llvm/CodeGen/RegisterPressure.h"
#include "llvm/CodeGen/ScheduleDAG.h"
#include "llvm/CodeGen/ScheduleDAGInstrs.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"
#include <algorithm>
#include <chrono>
#include <string>
#include <malloc.h>

#define DEBUG_TYPE "optsched"

using namespace llvm::opt_sched;

// hack to print spills
bool OPTSCHED_gPrintSpills;

// An array of possible OptSched heuristic names
constexpr struct {
  const char *Name;
  LISTSCHED_HEURISTIC HID;
} HeuristicNames[] = {
    {"CP", LSH_CP},   {"LUC", LSH_LUC}, {"UC", LSH_UC},
    {"NID", LSH_NID}, {"CPR", LSH_CPR}, {"ISO", LSH_ISO},
    {"SC", LSH_SC},   {"LS", LSH_LS},   {"LLVM", LSH_LLVM},
};

// Default path to the the configuration directory for opt-sched.
static constexpr const char *DEFAULT_CFG_DIR = "~/.optsched-cfg/";

// Default path to the scheduler options configuration file for opt-sched.
static constexpr const char *DEFAULT_CFGS_FNAME = "/sched.ini";

// Default path to the list of hot functions to schedule using opt-sched.
static constexpr const char *DEFAULT_CFGHF_FNAME = "/hotfuncs.ini";

// Default path to the machine model specification file for opt-sched.
static constexpr const char *DEFAULT_CFGMM_FNAME = "/machine_model.cfg";

// Default path to the machine model specification file for opt-sched.
static constexpr const char *DEFAULT_CFGOCL_FNAME = "/occupancy_limits.ini";

// Command line options for opt-sched.
static cl::opt<std::string> OptSchedCfg(
    "optsched-cfg", cl::Hidden,
    cl::desc(
        "Path to the directory containing configuration files for opt-sched."),
    cl::init(DEFAULT_CFG_DIR));

static cl::opt<std::string> OptSchedCfgS(
    "optsched-cfg-sched", cl::Hidden,
    cl::desc(
        "Path to the scheduler options configuration file for opt-sched."));

static cl::opt<std::string> OptSchedCfgHF(
    "optsched-cfg-hotfuncs", cl::Hidden,
    cl::desc("Path to the list of hot functions to schedule using opt-sched."));

static cl::opt<std::string> OptSchedCfgMM(
    "optsched-cfg-machine-model", cl::Hidden,
    cl::desc("Path to the machine model specification file for opt-sched."));

static cl::opt<std::string> OptSchedCfgOCL(
    "optsched-cfg-occupancy-limits", cl::Hidden,
    cl::desc("Path to the occupancy limits specification file for opt-sched."));

static void getRealCfgPathCL(SmallString<128> &Path) {
  SmallString<128> Tmp = Path;
  auto EC = sys::fs::real_path(Tmp, Path, true);
  if (EC)
    llvm::report_fatal_error(EC.message() + ": " + Tmp, false);
}

static void reportCfgDirPathError(std::error_code EC,
                                  llvm::StringRef OptSchedCfg) {
  if (OptSchedCfg == DEFAULT_CFG_DIR)
    llvm::report_fatal_error(
        llvm::StringRef(EC.message() +
                        ": Error searching for the OptSched config "
                        "directory in the default location: " +
                        DEFAULT_CFG_DIR),
        false);
  else
    llvm::report_fatal_error(EC.message() + ": " + OptSchedCfg, false);
}

// If this iterator is a debug value, increment until reaching the End or a
// non-debug instruction. static function copied from
// llvm/CodeGen/MachineScheduler.cpp
static MachineBasicBlock::iterator
nextIfDebug(MachineBasicBlock::iterator I,
            MachineBasicBlock::const_iterator End) {
  for (; I != End; ++I) {
    if (!I->isDebugValue())
      break;
  }
  return I;
}

static bool scheduleSpecificRegion(const StringRef RegionName,
                                   const Config &SchedIni) {
  const bool ScheduleSpecificRegions =
      SchedIni.GetBool("SCHEDULE_SPECIFIC_REGIONS");

  if (!ScheduleSpecificRegions)
    return true;

  const std::list<std::string> RegionList =
      SchedIni.GetStringList("REGIONS_TO_SCHEDULE");
  return std::find(std::begin(RegionList), std::end(RegionList), RegionName) !=
         std::end(RegionList);
}

static bool scheduleSpecificFunction(const StringRef FunctionName,
                                   const Config &SchedIni) {
  const bool ScheduleSpecificFunctions =
      SchedIni.GetBool("SCHEDULE_SPECIFIC_FUNCTIONS");


  if (!ScheduleSpecificFunctions)
    return true;

  const std::list<std::string> FunctionList =
      SchedIni.GetStringList("FUNCTIONS_TO_SCHEDULE");
  return std::find(std::begin(FunctionList), std::end(FunctionList), FunctionName) !=
         std::end(FunctionList);
}



static BLOCKS_TO_KEEP blocksToKeep(const Config &SchedIni) {
  const auto &Setting = SchedIni.GetString("BLOCKS_TO_KEEP");
  if (Setting == "ZERO_COST")
    return BLOCKS_TO_KEEP::ZERO_COST;
  if (Setting == "OPTIMAL")
    return BLOCKS_TO_KEEP::OPTIMAL;
  if (Setting == "IMPROVED")
    return BLOCKS_TO_KEEP::IMPROVED;
  if (Setting == "IMPROVED_OR_OPTIMAL")
    return BLOCKS_TO_KEEP::IMPROVED_OR_OPTIMAL;

  return BLOCKS_TO_KEEP::ALL;
}

static SchedulerType parseListSchedType() {
  auto SchedTypeString =
      SchedulerOptions::getInstance().GetString("HEUR_SCHED_TYPE");
  if (SchedTypeString == "LIST")
    return SCHED_LIST;
  if (SchedTypeString == "SEQ")
    return SCHED_SEQ;

  llvm::report_fatal_error(
      llvm::StringRef("Unrecognized option for HEUR_SCHED_TYPE: " +
                      SchedTypeString),
      false);
}

static std::unique_ptr<GraphTrans>
createStaticNodeSupTrans(DataDepGraph *DataDepGraph, bool IsMultiPass = false) {
  return std::make_unique<StaticNodeSupTrans>(DataDepGraph, IsMultiPass);
}

void ScheduleDAGOptSched::addGraphTransformations(
    OptSchedDDGWrapperBasic *BDDG) {
  auto *GraphTransformations = BDDG->GetGraphTrans();

  if (StaticNodeSup) {
    if (LatencyPrecision == LTP_UNITY) {
      GraphTransformations->push_back(
          createStaticNodeSupTrans(BDDG, MultiPassStaticNodeSup));
    } else {
      Logger::Info("Skipping RP-only graph transforms for non-unity pass.");
    }
  }
}

ScheduleDAGOptSched::ScheduleDAGOptSched(
    MachineSchedContext *C, std::unique_ptr<MachineSchedStrategy> S)
    : ScheduleDAGMILive(C, std::move(S)), C(C) {
  LLVM_DEBUG(dbgs() << "********** Optimizing Scheduler **********\n");

  // Find the native paths to the scheduler configuration files.
  getRealCfgPaths();

  // Setup config object
  Config &schedIni = SchedulerOptions::getInstance();
  // load OptSched ini file
  schedIni.Load(PathCfgS.c_str());

  // load hot functions ini file
  HotFunctions.Load(PathCfgHF.c_str());

  OccupancyLimits.Load(PathCfgOCL.c_str());

  // Load config files for the OptScheduler
  loadOptSchedConfig();

  int i = ParallelBB ? NumThreads : 1;
  while (i > 0) {
      EnumNodeAllocs.push_back(new MemAlloc<EnumTreeNode>(1000, -1));
      HistNodeAllocs.push_back(new MemAlloc<CostHistEnumTreeNode>(10000, -1));
      HashTablAllocs.push_back(new MemAlloc<BinHashTblEntry<HistEnumTreeNode>>(10000, -1));

      --i;
  }

  StringRef ArchName = TM.getTargetTriple().getArchName();
  Logger::Info("arch");
  Logger::Info(ArchName.data());
  auto TargetFactory =
      OptSchedTargetRegistry::Registry.getFactoryWithName(ArchName);

  if (!TargetFactory)
    TargetFactory =
        OptSchedTargetRegistry::Registry.getFactoryWithName("generic");

  OST = TargetFactory();

  if ((strncmp("amdgcn", ArchName.data(), 6) == 0) ||
      (strncmp("amdgcn-amd-amdhsa", ArchName.data(), 17) == 0)) {
    OST->SetOccupancyLimit(OccupancyLimit);
    OST->SetShouldLimitOcc(ShouldLimitOccupancy);
    OST->SetOccLimitSource(OccupancyLimitSource);
  }

  MM = OST->createMachineModel(PathCfgMM.c_str());
  MM->convertMachineModel(static_cast<ScheduleDAGInstrs &>(*this),
                          RegClassInfo);
}

ScheduleDAGOptSched::~ScheduleDAGOptSched() {
  int i = ParallelBB ? NumThreads : 1;
  --i;
  while (i >= 0) {
      delete EnumNodeAllocs[i];
      delete HistNodeAllocs[i];
      delete HashTablAllocs[i];
      --i;
  }
}

void ScheduleDAGOptSched::SetupLLVMDag() {
  // Initialize the register pressure tracker used by buildSchedGraph.
  RPTracker.init(&MF, RegClassInfo, LIS, BB, LiveRegionEnd,
                 /*ShouldTrackLaneMasks=*/true, /*TrackUntiedDefs=*/true);

  // Account for liveness generate by the region boundary. LiveRegionEnd is
  // the end of the MBB.
  if (LiveRegionEnd != RegionEnd)
    RPTracker.recede();

  // Build the DAG, and compute current register pressure.
  buildSchedGraph(AA, &RPTracker, nullptr, LIS, /*ShouldTrackLaneMasks=*/true);

  // Finalize live-in
  RPTracker.closeTop();

  if (EnableMutations) {
    Topo.InitDAGTopologicalSorting();
    postprocessDAG();
  }
}

// Add the two passes used for the two pass scheduling approach
void ScheduleDAGOptSched::initSchedulers() {
  // Add passes

  // First
  SchedPasses.push_back(OptSchedMinRP);
  // Second
  SchedPasses.push_back(OptSchedBalanced);
}

// schedule called for each basic block
void ScheduleDAGOptSched::schedule() {
  ShouldTrackPressure = true;
  ShouldTrackLaneMasks = true;
  Config &schedIni = SchedulerOptions::getInstance();

  ++RegionNumber;
  const std::string RegionName = C->MF->getFunction().getName().data() +
                                 std::string(":") +
                                 std::to_string(RegionNumber);

  // If two pass scheduling is enabled then
  // first just record the scheduling region.
  if (OptSchedEnabled && TwoPassEnabled && !TwoPassSchedulingStarted) {
    Regions.push_back(std::make_pair(RegionBegin, RegionEnd));
    LLVM_DEBUG(
        dbgs() << "Recording scheduling region before scheduling with two pass "
                  "scheduler...\n");
    return;
  }

  #ifdef PRINT_MIR
  bool print = false;
#endif


  if (!OptSchedEnabled || !scheduleSpecificRegion(RegionName, schedIni) || !scheduleSpecificFunction(MF.getName(), schedIni)) {
    LLVM_DEBUG(dbgs() << "Skipping region " << RegionName << "\n");
    ScheduleDAGMILive::schedule();
    return;
  }

  #ifdef PRINT_MIR
  else {
    print = true;
    Logger::Info("MIR Before Scheduling");
    C->MF->print(errs());
  }
#endif

  // This log output is parsed by scripts. Don't change its format unless you
  // are prepared to change the relevant scripts as well.
  Logger::Info("********** Opt Scheduling **********");
  Logger::Event("BeginScheduling");
  LLVM_DEBUG(dbgs() << "********** Scheduling Region " << RegionName
                    << " **********\n");
  LLVM_DEBUG(const auto *MBB = RegionBegin->getParent();
             dbgs() << MF.getName() << ":" << printMBBReference(*MBB) << " "
                    << MBB->getName() << "\n  From: " << *begin() << "    To: ";
             if (RegionEnd != MBB->end()) dbgs() << *RegionEnd;
             else dbgs() << "End";
             dbgs() << " RegionInstrs: " << NumRegionInstrs << '\n');

#ifdef IS_DEBUG_PEAK_PRESSURE
  SetupLLVMDag();
  Logger::Info("RP before scheduling");
  RPTracker.dump();
#endif

  // Use LLVM's heuristic schedule as input to the B&B scheduler.
  if (UseLLVMScheduler) {
    ScheduleDAGMILive::schedule();

    OriginalDAG = SUnits;
    // The schedule generated by LLVM
    ISOSchedule.resize(SUnits.size());
    // A vector index by NodeNum. The value
    // is the order in LLVM's schedule.
    std::vector<int> nodeNumMap(SUnits.size());

    // Find the schedule generated by LLVM which is stored in MMB.
    int counter = 0;
    for (MachineBasicBlock::instr_iterator I = BB->instr_begin(),
                                           E = BB->instr_end();
         I != E; ++I) {
      MachineInstr &instr = *I;
      SUnit *su = getSUnit(&instr);

      if (su != NULL && !su->isBoundaryNode()) {
        int nodeNum = su->NodeNum;
#ifdef IS_DEBUG_ISO
        Logger::Info("Node num %d", nodeNum);
#endif
        ISOSchedule[counter] = nodeNum;
        nodeNumMap[nodeNum] = counter;
        counter++;
      }
    }

    // Update SUnits with the discovered schedule. Re-number SUnits to be in
    // sequential order.
    for (size_t i = 0; i < SUnits.size(); i++) {
      // continue if the SUnit is already in the correct place
      unsigned newNum = ISOSchedule[i];
      unsigned curNum = SUnits[i].NodeNum;
      if (curNum == newNum) {
        SUnits[i].NodeNum = i;
        continue;
      }

      for (size_t j = i + 1; j < SUnits.size(); j++) {
        if (SUnits[j].NodeNum == newNum) {
#ifdef IS_DEBUG_ISO
          Logger::Info("Swapping %d with %d for ISO", SUnits[j].NodeNum,
                       SUnits[i].NodeNum);
#endif
          std::swap(SUnits[j], SUnits[i]);
          SUnits[i].NodeNum = i;
          break;
        }
      }
    }

    // Update edges.
    for (size_t i = 0; i < SUnits.size(); i++) {
      SUnit *unit = &SUnits[i];
      // Update successor NodeNums.
      for (SDep &dep : unit->Succs) {
        SUnit *succ = dep.getSUnit();
        // check if the successor is a boundary node
        if (succ->isBoundaryNode())
          continue;
        dep.setSUnit(&SUnits[nodeNumMap[succ->NodeNum]]);
      }

      // Update predecessor NodeNums.
      for (SDep &dep : unit->Preds) {
        SUnit *pred = dep.getSUnit();
        // check if the predecessor is a boundary node
        if (pred->isBoundaryNode())
          continue;
        dep.setSUnit(&SUnits[nodeNumMap[pred->NodeNum]]);
      }
    }
  } else {
    // Only call SetupLLVMDag if ScheduleDAGMILive::schedule() was not invoked.
    // ScheduleDAGMILive::schedule() will perform the same post processing
    // steps that SetupLLVMDag() does when called, and if the post processing
    // is called a second time the post processing will be applied a second
    // time.  This will lead to the leads to the a different LLVM DAG and will
    // cause the LLVM heuristic to produce a schedule which is significantly
    // different from the one produced by LLVM without OptSched.
    SetupLLVMDag();
  }

  OST->initRegion(this, MM.get(), OccupancyLimits);
  
  // Convert graph
  auto DDG =
      OST->createDDGWrapper(C, this, MM.get(), LatencyPrecision, RegionName, NumSolvers);

  int size = DDG.get()->getSize();
  DataDepGraph *dataDepGraph_ = static_cast<DataDepGraph *>(DDG.get());
  dataDepGraph_->setMF_(&MF);
  int preFiltered = false;

  Logger::Info("fin create ddg");

  //Logger::Info("DDG size is %d", DDG.getsize());
  if ((size < MinDDGSize || size > 1000) && !SecondPass) {
    Logger::Info("DDG of size %d is outside limits, not parallelizing", size);
    Logger::Info("Limits: min (%d), max (%d)", MinDDGSize, 1000);
    preFiltered = true;
    //return;
  }

  // In the second pass, ignore artificial edges before running the sequential
  // heuristic list scheduler.
  if (SecondPass)
    DDG->convertSUnits(false, true);
  else
    DDG->convertSUnits(false, false);


  Logger::Info("fin convert sunit");
  DDG->convertRegFiles();

  auto *BDDG = static_cast<OptSchedDDGWrapperBasic *>(DDG.get());
  addGraphTransformations(BDDG);

  Logger::Info("fin grph xform & convert reg");

  bool IsEasy = false;
  InstCount NormBestCost = 0;
  InstCount BestSchedLngth = 0;
  InstCount NormHurstcCost = 0;
  InstCount HurstcSchedLngth = 0;
  InstSchedule *Sched = NULL;
  FUNC_RESULT Rslt;
  bool FilterByPerp = schedIni.GetBool("FILTER_BY_PERP");

  int CurrentRegionTimeout = RegionTimeout;
  int CurrentLengthTimeout = LengthTimeout;
  if (IsTimeoutPerInst) {
    // Re-calculate timeout values if timeout setting is per instruction
    // because we want a unique value per DAG size
    CurrentRegionTimeout = RegionTimeout * SUnits.size();
    CurrentLengthTimeout = LengthTimeout * SUnits.size();
  }

  // create region
  if (!ParallelBB || SecondPass || preFiltered) {
    auto region = std::make_unique<BBWithSpill>(
        OST.get(), dataDepGraph_, 0, HistTableHashBits,
        LowerBoundAlgorithm, HeuristicPriorities, EnumPriorities, VerifySchedule,
        PruningStrategy, SchedForRPOnly, EnumStalls, SCW, SCF, HeurSchedType, TimeoutToMemblock,
        TwoPassEnabled, IsTimeoutPerInst, EnumNodeAllocs, HistNodeAllocs, HashTablAllocs);

      // Used for two-pass-optsched to alter upper bound value.
    if (SecondPass) 
      region->InitSecondPass();

    // Setup time before scheduling
    Utilities::startTime = std::chrono::high_resolution_clock::now();
    // Schedule region.

    Rslt = region->FindOptimalSchedule(CurrentRegionTimeout, CurrentLengthTimeout,
                                       IsEasy, NormBestCost, BestSchedLngth,
                                       NormHurstcCost, HurstcSchedLngth, Sched,
                                       FilterByPerp, blocksToKeep(schedIni), ParallelBB);


    if ((!(Rslt == RES_SUCCESS || Rslt == RES_TIMEOUT) || Sched == NULL)) {
      LLVM_DEBUG(
          Logger::Info("OptSched run failed: rslt=%d, sched=%p. Falling back.",
                       Rslt, (void *)Sched));
      // Scheduling with opt-sched failed.
      // fallbackScheduler();
      return;
    }

    LLVM_DEBUG(Logger::Info("OptSched succeeded."));
    OST->finalizeRegion(Sched);
    if (!OST->shouldKeepSchedule()) {
      for (size_t i = 0; i < SUnits.size(); i++) {
        SUnit SU = SUnits[i];
        ResetFlags(SU);
      }
      return;
    }
    // Count simulated spills.
    if (isSimRegAllocEnabled()) {
      SimulatedSpills += region->GetSimSpills();
    }    
  }
  else
  {
    Logger::Info("Running parallel B&B");
    auto region = std::make_unique<BBMaster>(
        OST.get(), dataDepGraph_, 0, HistTableHashBits,
        LowerBoundAlgorithm, HeuristicPriorities, EnumPriorities, VerifySchedule,
        PruningStrategy, SchedForRPOnly, EnumStalls, SCW, SCF, HeurSchedType, 
        NumThreads, MinNodesAsMultiple, MinSplittingDepth, MaxSplittingDepth, NumSolvers, LocalPoolSize, ExploitationPercent, GlobalPoolSCF,
        GlobalPoolSort, WorkSteal, IsTimeoutPerInst, TimeoutToMemblock, TwoPassEnabled, EnumNodeAllocs, HistNodeAllocs, HashTablAllocs);

      // Used for two-pass-optsched to alter upper bound value.
    if (SecondPass)
      region->InitSecondPass();

    // Setup time before scheduling
    Utilities::startTime = std::chrono::high_resolution_clock::now();
    // Schedule region.

    Rslt = region->FindOptimalSchedule(CurrentRegionTimeout, CurrentLengthTimeout,
                                       IsEasy, NormBestCost, BestSchedLngth,
                                       NormHurstcCost, HurstcSchedLngth, Sched,
                                       FilterByPerp, blocksToKeep(schedIni), ParallelBB);

    if ((!(Rslt == RES_SUCCESS || Rslt == RES_TIMEOUT) || Sched == NULL)) {
      LLVM_DEBUG(
          Logger::Info("OptSched run failed: rslt=%d, sched=%p. Falling back.",
                       Rslt, (void *)Sched));
      // Scheduling with opt-sched failed.
      // fallbackScheduler();
      return;
    }

    OST->finalizeRegion(Sched);
    if (!OST->shouldKeepSchedule()) {
      for (size_t i = 0; i < SUnits.size(); i++) {
        SUnit SU = SUnits[i];
        ResetFlags(SU);
      }
      return;
    }

      
    // Count simulated spills.
    if (isSimRegAllocEnabled()) {
      SimulatedSpills += region->GetSimSpills();
    }
  }

  // Convert back to LLVM.
  // Advance past initial DebugValues.
  CurrentTop = nextIfDebug(RegionBegin, RegionEnd);
  CurrentBottom = RegionEnd;
  InstCount cycle, slot;
  for (InstCount i = Sched->GetFrstInst(cycle, slot); i != INVALID_VALUE;
       i = Sched->GetNxtInst(cycle, slot)) {
    // Skip artificial instrs.
    if (i > static_cast<int>(SUnits.size()) - 1)
      continue;

    if (i == SCHD_STALL)
      ScheduleNode(NULL, cycle);
    else {
      SUnit *unit = &SUnits[i];
      if (unit && unit->isInstr())
        ScheduleNode(unit, cycle);
    }
  }

  if (Sched != NULL) delete Sched;
  placeDebugValues();
  #ifdef PRINT_MIR
  Logger::Info("MIR After Scheduling");
  if (print)
    MF.print(errs());
#endif


#ifdef IS_DEBUG_PEAK_PRESSURE
  Logger::Info("Register pressure after");
  RPTracker.dump();
#endif

}

void ScheduleDAGOptSched::ResetFlags(SUnit &SU) {
  RegisterOperands RegOpers;
  RegOpers.collect(*SU.getInstr(), *TRI, MRI, true, false);
  // Adjust liveness and add missing dead+read-undef flags.
  auto SlotIdx = LIS->getInstructionIndex(*SU.getInstr()).getRegSlot();
  RegOpers.adjustLaneLiveness(*LIS, MRI, SlotIdx, SU.getInstr());
}

void ScheduleDAGOptSched::ScheduleNode(SUnit *SU, unsigned CurCycle) {
#ifdef IS_DEBUG_CONVERT_LLVM
  Logger::Info("*** Scheduling [%lu]: ", CurCycle);
#endif
  if (SU) {
    MachineInstr *instr = SU->getInstr();
    // Reset read - undef flags and update them later.
    for (MIBundleOperands MIO(*instr); MIO.isValid(); ++MIO) {
      if (MIO->isReg() && MIO->isDef())
        MIO->setIsUndef(false);
    }

    if (&*CurrentTop == instr)
      CurrentTop = nextIfDebug(++CurrentTop, CurrentBottom);
    else
      moveInstruction(instr, CurrentTop);

    RegisterOperands RegOpers;
    RegOpers.collect(*instr, *TRI, MRI, true, false);
    // Adjust liveness and add missing dead+read-undef flags.
    auto SlotIdx = LIS->getInstructionIndex(*instr).getRegSlot();
    RegOpers.adjustLaneLiveness(*LIS, MRI, SlotIdx, instr);
  } else {
#ifdef IS_DEBUG_CONVERT_LLVM
    Logger::Info("Stall");
#endif
  }
}

// call the default "Fallback Scheduler" on a region
void ScheduleDAGOptSched::fallbackScheduler() {
  // If the heuristic is ISO the order of the SUnits will be
  // the order of LLVM's heuristic schedule. Otherwise reset
  // the BB to LLVM's original order, the order of the SUnits,
  // then call their scheduler.
  if (!UseLLVMScheduler)
    ScheduleDAGMILive::schedule();
  // Restore the original llvm schedule that was found earlier.
  else {
    CurrentTop = nextIfDebug(RegionBegin, RegionEnd);
    CurrentBottom = RegionEnd;

    for (size_t i = 0; i < SUnits.size(); i++) {
      MachineInstr *instr = SUnits[i].getInstr();

      if (CurrentTop == NULL) {
        LLVM_DEBUG(Logger::Error("Currenttop is NULL"));
        return;
      }

      if (&*CurrentTop == instr)
        CurrentTop = nextIfDebug(++CurrentTop, CurrentBottom);
      else
        moveInstruction(instr, CurrentTop);
    }
  }
}

void ScheduleDAGOptSched::loadOptSchedConfig() {
  SchedulerOptions &schedIni = SchedulerOptions::getInstance();
  // setup OptScheduler configuration options
  OptSchedEnabled = isOptSchedEnabled();
  TwoPassEnabled = isTwoPassEnabled();
  TwoPassSchedulingStarted = false;
  SecondPass = false;
  LatencyPrecision = fetchLatencyPrecision();
  TreatOrderAsDataDeps = schedIni.GetBool("TREAT_ORDER_DEPS_AS_DATA_DEPS");

  UseLLVMScheduler = false;
  // should we print spills for the current function
  OPTSCHED_gPrintSpills = shouldPrintSpills();
  StaticNodeSup = schedIni.GetBool("STATIC_NODE_SUPERIORITY", false);
  MultiPassStaticNodeSup =
      schedIni.GetBool("MULTI_PASS_NODE_SUPERIORITY", false);
  // setup pruning
  PruningStrategy.rlxd = schedIni.GetBool("APPLY_RELAXED_PRUNING");
  PruningStrategy.nodeSup = schedIni.GetBool("DYNAMIC_NODE_SUPERIORITY");
  PruningStrategy.histDom = schedIni.GetBool("APPLY_HISTORY_DOMINATION");
  PruningStrategy.spillCost = schedIni.GetBool("APPLY_SPILL_COST_PRUNING");
  PruningStrategy.useSuffixConcatenation =
      schedIni.GetBool("ENABLE_SUFFIX_CONCATENATION");
  MultiPassStaticNodeSup = schedIni.GetBool("MULTI_PASS_NODE_SUPERIORITY");
  SchedForRPOnly = schedIni.GetBool("SCHEDULE_FOR_RP_ONLY");
  HistTableHashBits =
      static_cast<int16_t>(schedIni.GetInt("HIST_TABLE_HASH_BITS"));
  VerifySchedule = schedIni.GetBool("VERIFY_SCHEDULE");
  EnableMutations = schedIni.GetBool("LLVM_MUTATIONS");
  EnumStalls = schedIni.GetBool("ENUMERATE_STALLS");
  SCW = schedIni.GetInt("SPILL_COST_WEIGHT");
  LowerBoundAlgorithm = parseLowerBoundAlgorithm();
  HeuristicPriorities = parseHeuristic(schedIni.GetString("HEURISTIC"));
  EnumPriorities = parseHeuristic(schedIni.GetString("ENUM_HEURISTIC"));
  SecondPassEnumPriorities =
      parseHeuristic(schedIni.GetString("SECOND_PASS_ENUM_HEURISTIC"));
  SCF = parseSpillCostFunc();
  RegionTimeout = schedIni.GetInt("REGION_TIMEOUT");
  FirstPassRegionTimeout = schedIni.GetInt("FIRST_PASS_REGION_TIMEOUT");
  SecondPassRegionTimeout = schedIni.GetInt("SECOND_PASS_REGION_TIMEOUT");
  LengthTimeout = schedIni.GetInt("LENGTH_TIMEOUT");
  FirstPassLengthTimeout = schedIni.GetInt("FIRST_PASS_LENGTH_TIMEOUT");
  SecondPassLengthTimeout = schedIni.GetInt("SECOND_PASS_LENGTH_TIMEOUT");

  TimeoutToMemblock = schedIni.GetInt("TIMEOUT_TO_MEMBLOCK_RATIO");

  OccupancyLimit = schedIni.GetInt("OCCUPANCY_LIMIT");
  ShouldLimitOccupancy = schedIni.GetBool("SHOULD_LIMIT_OCCUPANCY");

  OccupancyLimitSource = OCC_LIMIT_TYPE::OLT_NONE;
  if (ShouldLimitOccupancy)
    OccupancyLimitSource =
        parseOccLimit(schedIni.GetString("OCCUPANCY_LIMIT_SOURCE"));

  ParallelBB = schedIni.GetBool("USE_PARALLEL_BB");
  NumThreads = schedIni.GetInt("NUMBER_THREADS");
  MinSplittingDepth = schedIni.GetInt("MIN_SPLITTING_DEPTH");
  MaxSplittingDepth = schedIni.GetInt("MAX_SPLITTING_DEPTH");
  MinNodesAsMultiple = schedIni.GetInt("MIN_NODES_AS_MULTIPLE_THREADS");
  WorkSteal = schedIni.GetBool("USE_WORK_STEAL");

  MinDDGSize = schedIni.GetInt("DDG_SIZE_MIN");

  LocalPoolSize = schedIni.GetInt("LOCAL_POOL_SIZE");
  ExploitationPercent = schedIni.GetFloat("EXPLOITATION_PERCENT");

  GlobalPoolSort = parseGlobalPoolSort();
  GlobalPoolSCF = parseGlobalPoolSpillCostFunc();

  
  // TODO change architecture so we only need NumThreads Solvers
  // We need NumThreads + 2 NumSolvers in parallel for: NumThreads parallelThreads, + 1 master, + 1 best sched
  NumSolvers = ParallelBB ? NumThreads + 2: 1;

  if (schedIni.GetString("TIMEOUT_PER") == "INSTR")
    IsTimeoutPerInst = true;
  else
    IsTimeoutPerInst = false;
  int randomSeed = schedIni.GetInt("RANDOM_SEED", 0);
  if (randomSeed == 0)
    randomSeed = time(NULL);
  RandomGen::SetSeed(randomSeed);
  HeurSchedType = parseListSchedType();
}

bool ScheduleDAGOptSched::isOptSchedEnabled() const {
  // check scheduler ini file to see if optsched is enabled
  auto optSchedOption =
      SchedulerOptions::getInstance().GetString("USE_OPT_SCHED");
  if (optSchedOption == "YES") {
    return true;
  } else if (optSchedOption == "HOT_ONLY") {
    // get the name of the function this scheduler was created for
    std::string functionName = C->MF->getFunction().getName().data();
    // check the list of hot functions for the name of the current function
    return HotFunctions.GetBool(functionName, false);
  } else if (optSchedOption == "NO") {
    return false;
  }

  llvm::report_fatal_error(
      llvm::StringRef("Unrecognized option for USE_OPT_SCHED setting: " +
                      optSchedOption),
      false);
}

bool ScheduleDAGOptSched::isTwoPassEnabled() const {
  // check scheduler ini file to see if two pass scheduling is enabled
  auto twoPassOption =
      SchedulerOptions::getInstance().GetString("USE_TWO_PASS");
  if (twoPassOption == "YES")
    return true;
  else if (twoPassOption == "NO")
    return false;

  llvm::report_fatal_error(
      llvm::StringRef("Unrecognized option for USE_TWO_PASS setting: " +
                      twoPassOption),
      false);
}

LATENCY_PRECISION ScheduleDAGOptSched::fetchLatencyPrecision() const {
  std::string lpName =
      SchedulerOptions::getInstance().GetString("LATENCY_PRECISION");
  if (lpName == "FILE" || lpName == "PRECISE") {
    return LTP_PRECISE;
  } else if (lpName == "LLVM" || lpName == "ROUGH") {
    return LTP_ROUGH;
  } else if (lpName == "UNIT" || lpName == "UNITY") {
    return LTP_UNITY;
  }

  llvm::report_fatal_error(
      llvm::StringRef("Unrecognized option for LATENCY_PRECISION setting: " +
                      lpName),
      false);
}

LB_ALG ScheduleDAGOptSched::parseLowerBoundAlgorithm() const {
  std::string LBalg = SchedulerOptions::getInstance().GetString("LB_ALG");
  if (LBalg == "RJ") {
    return LBA_RJ;
  } else if (LBalg == "LC") {
    return LBA_LC;
  }

  llvm::report_fatal_error(
      llvm::StringRef("Unrecognized option for LB_ALG setting: " + LBalg),
      false);
}

// Helper function to find the next substring which is a heuristic name in Str
static LISTSCHED_HEURISTIC GetNextHeuristicName(const std::string &Str,
                                                size_t &StartIndex) {
  size_t Walk;
  for (Walk = StartIndex; Walk <= Str.length(); ++Walk) {
    if (Str[Walk] == '_')
      break;
  }

  // Match heuristic name to enum id
  for (const auto &LSH : HeuristicNames)
    if (!Str.compare(StartIndex, Walk - StartIndex, LSH.Name)) {
      StartIndex = Walk + 1;
      return LSH.HID;
    }

  llvm::report_fatal_error(
      llvm::StringRef("Unrecognized heuristic used: " + Str), false);
}

SchedPriorities ScheduleDAGOptSched::parseHeuristic(const std::string &Str) {
  SchedPriorities Priorities;
  size_t StartIndex = 0;
  Priorities.cnt = 0;
  Priorities.isDynmc = false;
  do {
    LISTSCHED_HEURISTIC LSH = GetNextHeuristicName(Str, StartIndex);
    Priorities.vctr[Priorities.cnt++] = LSH;
    switch (LSH) {
    // Is LUC still the only dynamic heuristic?
    case LSH_LUC:
      Priorities.isDynmc = true;
      break;
    case LSH_LLVM:
      UseLLVMScheduler = true;
      break;
    default:
      break;
    }
  } while (!(StartIndex > Str.length()));

  return Priorities;
}

// TODO(Jeff) use enum to encode rather than int (return value)
int ScheduleDAGOptSched::parseGlobalPoolSort() const {
  std::string name =
      SchedulerOptions::getInstance().GetString("GLOBAL_POOL_SORT");
  
  if (name == "SPILL_COST") {
    return 0;
  }
  else if (name == "HEURISTIC") {
    return 1;
  }

    llvm::report_fatal_error(
      llvm::StringRef("Unrecognized option for GLOBAL_POOL_SORT setting: " + name), false);

}

SPILL_COST_FUNCTION ScheduleDAGOptSched::parseSpillCostFunc() const {
  std::string name =
      SchedulerOptions::getInstance().GetString("SPILL_COST_FUNCTION");
  // PERP used to be called PEAK.
  if (name == "PERP" || name == "PEAK") {
    return SCF_PERP;
  } else if (name == "PRP") {
    return SCF_PRP;
  } else if (name == "PEAK_PER_TYPE") {
    return SCF_PEAK_PER_TYPE;
  } else if (name == "SUM") {
    return SCF_SUM;
  } else if (name == "PEAK_PLUS_AVG") {
    return SCF_PEAK_PLUS_AVG;
  } else if (name == "SLIL") {
    return SCF_SLIL;
  } else if (name == "OCC" || name == "TARGET") {
    return SCF_TARGET;
  }

  llvm::report_fatal_error(
      llvm::StringRef("Unrecognized option for SPILL_COST_FUNCTION setting: " + name), false);
}


SPILL_COST_FUNCTION ScheduleDAGOptSched::parseGlobalPoolSpillCostFunc() const {
  std::string name =
      SchedulerOptions::getInstance().GetString("GLOBAL_POOL_SPILL_COST_FUNCTION");
  // PERP used to be called PEAK.
  if (name == "PERP" || name == "PEAK") {
    return SCF_PERP;
  } else if (name == "PRP") {
    return SCF_PRP;
  } else if (name == "PEAK_PER_TYPE") {
    return SCF_PEAK_PER_TYPE;
  } else if (name == "SUM") {
    return SCF_SUM;
  } else if (name == "PEAK_PLUS_AVG") {
    return SCF_PEAK_PLUS_AVG;
  } else if (name == "SLIL") {
    return SCF_SLIL;
  } else if (name == "OCC" || name == "TARGET") {
    return SCF_TARGET;
  }

  llvm::report_fatal_error(
      llvm::StringRef("Unrecognized option for SPILL_COST_FUNCTION setting: " + name), false);
}


OCC_LIMIT_TYPE
ScheduleDAGOptSched::parseOccLimit(const std::string Str) {
  OCC_LIMIT_TYPE result = OCC_LIMIT_TYPE::OLT_NONE;

  if (Str == "NONE") {
    return OCC_LIMIT_TYPE::OLT_NONE;
  } else if (Str == "HEURISTIC") {
    return OCC_LIMIT_TYPE::OLT_HEUR;
  } else if (Str == "FILE") {
    return OCC_LIMIT_TYPE::OLT_FILE;
  }

  llvm::report_fatal_error(
      llvm::StringRef("Unrecognized option for LATENCY_PRECISION setting: " +
                      Str),
      false);
  return result;
}


bool ScheduleDAGOptSched::shouldPrintSpills() const {
  std::string printSpills =
      SchedulerOptions::getInstance().GetString("PRINT_SPILL_COUNTS");
  if (printSpills == "YES") {
    return true;
  } else if (printSpills == "NO") {
    return false;
  } else if (printSpills == "HOT_ONLY") {
    std::string functionName = C->MF->getFunction().getName().data();
    return HotFunctions.GetBool(functionName, false);
  }

  llvm::report_fatal_error(
      llvm::StringRef("Unrecognized option for PRINT_SPILL_COUNTS setting: " + printSpills),
      false);
}

bool ScheduleDAGOptSched::rpMismatch(InstSchedule *sched) {
  SetupLLVMDag();

  // LLVM peak register pressure
  const std::vector<unsigned> &RegionPressure =
      RPTracker.getPressure().MaxSetPressure;
  // OptSched peak register pressure
  const unsigned *regPressures = nullptr;
  // auto regTypeCount = sched->GetPeakRegPressures(regPressures);

  for (unsigned i = 0, e = RegionPressure.size(); i < e; ++i) {
    if (RegionPressure[i] != regPressures[i])
      return true;
  }

  return false;
}

void ScheduleDAGOptSched::finalizeSchedule() {
  if (TwoPassEnabled && OptSchedEnabled) {
    initSchedulers();

    LLVM_DEBUG(dbgs() << "Starting two pass scheduling approach\n");
    TwoPassSchedulingStarted = true;
    for (const SchedPassStrategy &S : SchedPasses) {
      MachineBasicBlock *MBB = nullptr;
      // Reset
      RegionNumber = ~0u;

      for (auto &Region : Regions) {
        RegionBegin = Region.first;
        RegionEnd = Region.second;

        if (RegionBegin->getParent() != MBB) {
          if (MBB)
            finishBlock();
          MBB = RegionBegin->getParent();
          startBlock(MBB);
        }
        unsigned NumRegionInstrs = std::distance(begin(), end());
        enterRegion(MBB, begin(), end(), NumRegionInstrs);

        // Skip empty scheduling regions (0 or 1 schedulable instructions).
        if (begin() == end() || begin() == std::prev(end())) {
          exitRegion();
          continue;
        }
        runSchedPass(S);
        Region = std::make_pair(RegionBegin, RegionEnd);
        exitRegion();
      }
      finishBlock();
    }
  }

  ScheduleDAGMILive::finalizeSchedule();

  LLVM_DEBUG(if (isSimRegAllocEnabled()) {
    dbgs() << "*************************************\n";
    dbgs() << "Function: " << MF.getName()
           << "\nTotal Simulated Spills: " << SimulatedSpills << "\n";
    dbgs() << "*************************************\n";
  });
}

void ScheduleDAGOptSched::runSchedPass(SchedPassStrategy S) {
  switch (S) {
  case OptSchedMinRP:
    scheduleOptSchedMinRP();
    break;
  case OptSchedBalanced:
    scheduleOptSchedBalanced();
    break;
  }
}

void ScheduleDAGOptSched::scheduleOptSchedMinRP() {
  LatencyPrecision = LTP_UNITY;
  // Set times for the first pass
  RegionTimeout = FirstPassRegionTimeout;
  LengthTimeout = FirstPassLengthTimeout;
  HeurSchedType = SCHED_LIST;
  Logger::Info("Start of first pass through\n");

  schedule();
  Logger::Event("PassFinished", "num", 1);
  // TODO(justin): Remove once relevant scripts have been updated:
  // get-benchmark-stats.py, get-optsched-stats.py, get-sched-length.py,
  // plaidbench-validation-test.py
  Logger::Info("End of first pass through\n");
}

void ScheduleDAGOptSched::scheduleOptSchedBalanced() {
  SecondPass = true;
  LatencyPrecision = LTP_ROUGH;

  // Set times for the second pass
  RegionTimeout = SecondPassRegionTimeout;
  LengthTimeout = SecondPassLengthTimeout;

  // Set the heuristic for the enumerator in the second pass.
  EnumPriorities = SecondPassEnumPriorities;

  // Force the input to the balanced scheduler to be the sequential order of the
  // (hopefully) good register pressure schedule. We don't want the list
  // scheduler to mangle the input because of latency or resource constraints.
  HeurSchedType = SCHED_SEQ;

  // Force disable LLVM scheduler so that it doesn't re-order schedule
  // from first pass.
  UseLLVMScheduler = false;

  // Disable RP-only for 2nd pass.
  SchedForRPOnly = false;

  // Disable RP-only graph transformations in balanced mode
  StaticNodeSup = false;
  MultiPassStaticNodeSup = false;

  Logger::Info("Start of second pass through\n");

  schedule();
  Logger::Event("PassFinished", "num", 2);
  // TODO(justin): Remove once relevant scripts have been updated:
  // get-benchmark-stats.py, get-optsched-stats.py, get-sched-length.py,
  // plaidbench-validation-test.py
  Logger::Info("End of second pass through");
}

bool ScheduleDAGOptSched::isSimRegAllocEnabled() const {
  // This will return false if only the list schedule is allocated.
  return (
      OPTSCHED_gPrintSpills &&
      (SchedulerOptions::getInstance().GetString(
           "SIMULATE_REGISTER_ALLOCATION") == "BEST" ||
       SchedulerOptions::getInstance().GetString(
           "SIMULATE_REGISTER_ALLOCATION") == "BOTH" ||
       SchedulerOptions::getInstance().GetString(
           "SIMULATE_REGISTER_ALLOCATION") == "TAKE_SCHED_WITH_LEAST_SPILLS"));
}

void ScheduleDAGOptSched::getRealCfgPaths() {
  // Find full path to OptSchedCfg directory.
  SmallString<128> PathCfg;
  auto EC = sys::fs::real_path(OptSchedCfg, PathCfg, true);
  if (EC)
    reportCfgDirPathError(EC, OptSchedCfg);

  // If the path to any of the config files are not specified, use the default
  // values.
  if (OptSchedCfgS.empty())
    (PathCfg + DEFAULT_CFGS_FNAME).toVector(PathCfgS);
  else {
    PathCfgS = OptSchedCfgS;
    getRealCfgPathCL(PathCfgS);
  }

  if (OptSchedCfgHF.empty())
    (PathCfg + DEFAULT_CFGHF_FNAME).toVector(PathCfgHF);
  else {
    PathCfgHF = OptSchedCfgHF;
    getRealCfgPathCL(PathCfgHF);
  }

  if (OptSchedCfgMM.empty())
    (PathCfg + DEFAULT_CFGMM_FNAME).toVector(PathCfgMM);
  else {
    PathCfgMM = OptSchedCfgMM;
    getRealCfgPathCL(PathCfgMM);
  }

  if (OptSchedCfgOCL.empty())
    (PathCfg + DEFAULT_CFGOCL_FNAME).toVector(PathCfgOCL);
  else {
    PathCfgOCL = OptSchedCfgOCL;
    getRealCfgPathCL(PathCfgOCL);
  }

  // Convert full paths to native fromat.
  sys::path::native(PathCfgS);
  sys::path::native(PathCfgHF);
  sys::path::native(PathCfgMM);
}

#if !defined(NDEBUG) || defined(LLVM_ENABLE_DUMP)
// Print registers from RegisterMaskPair vector
static Printable
printMaskPairs(const SmallVectorImpl<RegisterMaskPair> &RegPairs,
               const TargetRegisterInfo *TRI, const MachineRegisterInfo &MRI) {
  return Printable([&RegPairs, TRI, &MRI](raw_ostream &OS) {
    for (const auto &P : RegPairs) {
      const TargetRegisterClass *RegClass;

      if (P.RegUnit.isPhysicalRegister(P.RegUnit))
        RegClass = TRI->getMinimalPhysRegClass(P.RegUnit);
      else if (P.RegUnit.isVirtualRegister(P.RegUnit))
        RegClass = MRI.getRegClass(P.RegUnit);
      else
        RegClass = nullptr;

      OS << printReg(P.RegUnit, TRI, 0) << ':' << PrintLaneMask(P.LaneMask)
         << " (" << (RegClass ? TRI->getRegClassName(RegClass) : "noclass")
         << ") ";
    }
  });
}

LLVM_DUMP_METHOD
void ScheduleDAGOptSched::dumpLLVMRegisters() const {
  dbgs() << "LLVM Registers\n";
  for (const auto &SU : SUnits) {
    assert(SU.isInstr());
    const MachineInstr *MI = SU.getInstr();
    dbgs() << "Instr: ";
    dbgs() << "(" << SU.NodeNum << ") " << TII->getName(MI->getOpcode())
           << '\n';

    RegisterOperands RegOpers;
    RegOpers.collect(*MI, *TRI, MRI, true, false);

    dbgs() << "\t--Defs: " << printMaskPairs(RegOpers.Defs, TRI, MRI);
    dbgs() << '\n';

    dbgs() << "\t--Uses: " << printMaskPairs(RegOpers.Uses, TRI, MRI);
    dbgs() << "\n\n";
  }

  // Print registers used/defined by the region boundary
  const MachineInstr *MI = getRegionEnd();
  // Make sure EndRegion is not a sentinel value
  if (MI) {
    dbgs() << "Instr: ";
    dbgs() << "(ExitSU) " << TII->getName(MI->getOpcode()) << '\n';

    RegisterOperands RegOpers;
    RegOpers.collect(*MI, *TRI, MRI, true, false);

    dbgs() << "\t--Defs: " << printMaskPairs(RegOpers.Defs, TRI, MRI);
    dbgs() << '\n';

    dbgs() << "\t--Uses: " << printMaskPairs(RegOpers.Uses, TRI, MRI);
    dbgs() << "\n\n";
  }

  // Print live-in/live-out register
  dbgs() << "Live-In/Live-Out registers:\n";
  dbgs() << "\t--Live-In: "
         << printMaskPairs(getRegPressure().LiveInRegs, TRI, MRI);
  dbgs() << '\n';

  dbgs() << "\t--Live-Out: "
         << printMaskPairs(getRegPressure().LiveOutRegs, TRI, MRI);
  dbgs() << "\n\n";
}
#endif
