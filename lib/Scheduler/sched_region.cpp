#include <algorithm>
#include <cstdio>
#include <memory>
#include <utility>

#include "Wrapper/OptSchedDDGWrapperBasic.h"
#include "opt-sched/Scheduler/aco.h"
#include "opt-sched/Scheduler/bb_thread.h"
#include "opt-sched/Scheduler/config.h"
#include "opt-sched/Scheduler/graph_trans.h"
#include "opt-sched/Scheduler/list_sched.h"
#include "opt-sched/Scheduler/logger.h"
#include "opt-sched/Scheduler/random.h"
#include "opt-sched/Scheduler/reg_alloc.h"
#include "opt-sched/Scheduler/relaxed_sched.h"
#include "opt-sched/Scheduler/sched_region.h"
#include "opt-sched/Scheduler/stats.h"
#include "opt-sched/Scheduler/utilities.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FileSystem.h"

#define SPILLCOSTWEIGHT 10000

extern bool OPTSCHED_gPrintSpills;

using namespace llvm::opt_sched;

namespace fs = llvm::sys::fs;

static bool GetDumpDDGs() {
  // Cache the result so that we don't have to keep looking it up.
  // This is in a function so that the initialization is definitely delayed
  // until after the SchedulerOptions has a chance to be initialized.
  static bool DumpDDGs =
      SchedulerOptions::getInstance().GetBool("DUMP_DDGS", false);
  return DumpDDGs;
}

static std::string ComputeDDGDumpPath() {
  std::string Path =
      SchedulerOptions::getInstance().GetString("DDG_DUMP_PATH", "");

  if (GetDumpDDGs()) {
    // Force the user to set DDG_DUMP_PATH
    if (Path.empty())
      llvm::report_fatal_error(
          llvm::StringRef("DDG_DUMP_PATH must be set if trying to DUMP_DDGS."),
          false);

    // Do some niceness to the input path to produce the actual path.
    llvm::SmallString<32> FixedPath;
    const std::error_code ec =
        llvm::sys::fs::real_path(Path, FixedPath, /* expand_tilde = */ true);
    if (ec)
      llvm::report_fatal_error(
          llvm::StringRef("Unable to expand DDG_DUMP_PATH. " + ec.message()),
          false);
    Path.assign(FixedPath.begin(), FixedPath.end());

    // The path must be a directory, and it must exist.
    if (!llvm::sys::fs::is_directory(Path))
      llvm::report_fatal_error(
          llvm::StringRef("DDG_DUMP_PATH is set to a non-existent directory or "
                          "non-directory " +
                          Path),
          false);

    // Force the path to be considered a directory.
    // Note that redundant `/`s are okay in the path.
    Path.push_back('/');
  }

  return Path;
}

static std::string GetDDGDumpPath() {
  static std::string DDGDumpPath = ComputeDDGDumpPath();
  return DDGDumpPath;
}

SchedRegion::SchedRegion(MachineModel *machMdl, DataDepGraph *dataDepGraph,
                         long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
                         SchedPriorities hurstcPrirts,
                         SchedPriorities enumPrirts, bool vrfySched,
                         Pruning PruningStrategy, SchedulerType HeurSchedType,
                         SmallVector<MemAlloc<EnumTreeNode> *, 16> &EnumNodeAllocs,
                         SmallVector<MemAlloc<CostHistEnumTreeNode> *, 16> &HistNodeAllocs, 
                         SmallVector<MemAlloc<BinHashTblEntry<HistEnumTreeNode>> *, 16> &HashTablAllocs,
                         SPILL_COST_FUNCTION spillCostFunc) {
  machMdl_ = machMdl;
  dataDepGraph_ = dataDepGraph;
  rgnNum_ = rgnNum;
  sigHashSize_ = sigHashSize;
  lbAlg_ = lbAlg;
  hurstcPrirts_ = hurstcPrirts;
  enumPrirts_ = enumPrirts;
  vrfySched_ = vrfySched;
  prune_ = PruningStrategy;
  HeurSchedType_ = HeurSchedType;
  isSecondPass_ = false;

  totalSimSpills_ = INVALID_VALUE;
  bestCost_ = INVALID_VALUE;
  bestSchedLngth_ = INVALID_VALUE;
  hurstcCost_ = INVALID_VALUE;
  enumCrntSched_ = NULL;
  enumBestSched_ = NULL;
  schedLwrBound_ = 0;
  schedUprBound_ = INVALID_VALUE;

  spillCostFunc_ = spillCostFunc;

  OptimalSolverID_ = new int;  

  EnumNodeAllocs_ = EnumNodeAllocs;
  HistNodeAllocs_ = HistNodeAllocs;
  HashTablAllocs_ = HashTablAllocs;
  
  DumpDDGs_ = GetDumpDDGs();
  DDGDumpPath_ = GetDDGDumpPath();
}

void SchedRegion::UseFileBounds_() {
  InstCount fileLwrBound, fileUprBound;

  dataDepGraph_->UseFileBounds();
  dataDepGraph_->GetFileSchedBounds(fileLwrBound, fileUprBound);
  assert(fileLwrBound >= schedLwrBound_);
  schedLwrBound_ = fileLwrBound;
}

InstSchedule *SchedRegion::AllocNewSched_() {
  InstSchedule *newSched =
      new InstSchedule(machMdl_, dataDepGraph_, vrfySched_);
  return newSched;
}

void SchedRegion::CmputAbslutUprBound_() {
  abslutSchedUprBound_ = dataDepGraph_->GetAbslutSchedUprBound();
}

static bool isBbEnabled(Config &schedIni, Milliseconds rgnTimeout) {
  bool EnableBbOpt = schedIni.GetBool("ENUM_ENABLED");
  if (!EnableBbOpt)
    return false;

  if (rgnTimeout <= 0) {
    Logger::Info("Disabling enumerator because region timeout is set to zero.");
    return false;
  }

  return true;
}

static void dumpDDG(DataDepGraph *DDG, llvm::StringRef DDGDumpPath,
                    llvm::StringRef Suffix = "") {
  std::string Path = DDGDumpPath.data();
  Path += DDG->GetDagID();

  if (!Suffix.empty()) {
    Path += '.';
    Path += Suffix;
  }

  Path += ".ddg";
  // DagID has a `:` in the name, which symbol is not allowed in a path name.
  // Replace the `:` with a `.` to produce a legal path name.
  std::replace(Path.begin(), Path.end(), ':', '.');

  Logger::Info("Writing DDG to %s", Path.c_str());

  FILE *f = std::fopen(Path.c_str(), "w");
  if (!f) {
    Logger::Error("Unable to open the file: %s. %s", Path.c_str(),
                  std::strerror(errno));
    return;
  }
  DDG->WriteToFile(f, RES_SUCCESS, 1, 0, 0);
  std::fclose(f);
}

FUNC_RESULT SchedRegion::FindOptimalSchedule(
    Milliseconds rgnTimeout, Milliseconds lngthTimeout, bool &isLstOptml,
    InstCount &bestCost, InstCount &bestSchedLngth, InstCount &hurstcCost,
    InstCount &hurstcSchedLngth, InstSchedule *&bestSched, bool filterByPerp,
    const BLOCKS_TO_KEEP blocksToKeep, bool isParallelBB) {
  ConstrainedScheduler *lstSchdulr = NULL;
  InstSchedule *InitialSchedule = nullptr;
  InstSchedule *lstSched = NULL;
  InstSchedule *AcoSchedule = nullptr;
  InstCount InitialScheduleLength = 0;
  InstCount InitialScheduleCost = 0;
  FUNC_RESULT rslt = RES_SUCCESS;
  Milliseconds hurstcTime = 0;
  Milliseconds boundTime = 0;
  Milliseconds enumTime = 0;
  Milliseconds vrfyTime = 0;
  Milliseconds AcoTime = 0;
  Milliseconds AcoStart = 0;
  InstCount heuristicScheduleLength = INVALID_VALUE;
  InstCount AcoScheduleLength_ = INVALID_VALUE;
  InstCount AcoScheduleCost_ = INVALID_VALUE;

  enumCrntSched_ = NULL;
  enumBestSched_ = NULL;
  bestSched = bestSched_ = NULL;

  bool AcoBeforeEnum = false;
  bool AcoAfterEnum = false;

  // Do we need to compute the graph's transitive closure?
  bool needTransitiveClosure = false;

  // Algorithm run order:
  // 1) Heuristic Scheduler
  // 2) ACO
  // 3) Branch & Bound Enumerator
  // 4) ACO
  // Each of these 4 algorithms can be individually disabled, but either the
  // heuristic scheduler or ACO before the branch & bound enumerator must be
  // enabled.
  Config &schedIni = SchedulerOptions::getInstance();
  bool HeuristicSchedulerEnabled = schedIni.GetBool("HEUR_ENABLED");
  bool AcoSchedulerEnabled = schedIni.GetBool("ACO_ENABLED");
  bool BbSchedulerEnabled = isBbEnabled(schedIni, rgnTimeout);

  if (AcoSchedulerEnabled) {
    AcoBeforeEnum = schedIni.GetBool("ACO_BEFORE_ENUM");
    AcoAfterEnum = schedIni.GetBool("ACO_AFTER_ENUM");
  }

  if (!HeuristicSchedulerEnabled && !AcoBeforeEnum) {
    // Abort if ACO and heuristic algorithms are disabled.
    llvm::report_fatal_error(
        llvm::StringRef("Heuristic list scheduler or ACO must be enabled "
                        "before enumerator."),
        false);
    return RES_ERROR;
  }

  Logger::Info("---------------------------------------------------------------"
               "------------");
  Logger::Event("ProcessDag", "name", dataDepGraph_->GetDagID(),
                "num_instructions", dataDepGraph_->GetInstCnt(), //
                "max_latency", dataDepGraph_->GetMaxLtncy());
  
  // TODO(justin): Remove once relevant scripts have been updated:
  // func-stats.py, rp-compare.py, get-benchmark-stats.py,
  // get-optsched-stats.py, get-sched-length.py, runspec-wrapper-SLIL.py
  Logger::Info("Processing DAG %s with %d insts and max latency %d.",
               dataDepGraph_->GetDagID(), dataDepGraph_->GetInstCnt(),
               dataDepGraph_->GetMaxLtncy());

  stats::problemSize.Record(dataDepGraph_->GetInstCnt());

  const auto *GraphTransformations = dataDepGraph_->GetGraphTrans();
  if (BbSchedulerEnabled || GraphTransformations->size() > 0 ||
      spillCostFunc_ == SCF_SLIL)
    needTransitiveClosure = true;

  rslt = dataDepGraph_->SetupForSchdulng(needTransitiveClosure);
  if (rslt != RES_SUCCESS) {
    Logger::Info("Invalid input DAG");
    return rslt;
  }

  if (DumpDDGs_) {
    dumpDDG(dataDepGraph_, DDGDumpPath_);
  }


  // Apply graph transformations
  for (auto &GT : *GraphTransformations) {
    rslt = GT->ApplyTrans();

    if (rslt != RES_SUCCESS)
      return rslt;

    // Update graph after each transformation
    rslt = dataDepGraph_->UpdateSetupForSchdulng(needTransitiveClosure);
    if (rslt != RES_SUCCESS) {
      Logger::Info("Invalid DAG after graph transformations");
      return rslt;
    }
  }

  SetupForSchdulng_();
  CmputAbslutUprBound_();
  schedLwrBound_ = dataDepGraph_->GetSchedLwrBound();

  // Step #1: Find the heuristic schedule if enabled.
  // Note: Heuristic scheduler is required for the two-pass scheduler
  // to use the sequential list scheduler which inserts stalls into
  // the schedule found in the first pass.
  if (HeuristicSchedulerEnabled || isSecondPass_) {
    Milliseconds hurstcStart = Utilities::GetProcessorTime();
    lstSched = new InstSchedule(machMdl_, dataDepGraph_, vrfySched_);

    lstSchdulr = AllocHeuristicScheduler_();

    rslt = lstSchdulr->FindSchedule(lstSched, this);

    if (rslt != RES_SUCCESS) {
      llvm::report_fatal_error(llvm::StringRef("List scheduling failed"),
                               false);
      delete lstSchdulr;
      delete lstSched;
      return rslt;
    }

    hurstcTime = Utilities::GetProcessorTime() - hurstcStart;
    stats::heuristicTime.Record(hurstcTime);
    if (hurstcTime > 0)
      Logger::Info("Heuristic_Time %d", hurstcTime);
  }

  // After the sequential scheduler in the second pass, add the artificial edges
  // to the DDG. Some mutations were adding artificial edges which caused a
  // conflict with the sequential scheduler. Therefore, wait until the
  // sequential scheduler is done before adding artificial edges.
  if (isSecondPass_) {
    static_cast<OptSchedDDGWrapperBasic *>(dataDepGraph_)->addArtificialEdges();
    rslt = dataDepGraph_->UpdateSetupForSchdulng(needTransitiveClosure);
    if (rslt != RES_SUCCESS) {
      Logger::Info("Invalid DAG after adding artificial cluster edges");
      return rslt;
    }
  }

  // This must be done after SetupForSchdulng() or UpdateSetupForSchdulng() to
  // avoid resetting lower bound values.
  if (!BbSchedulerEnabled)
    costLwrBound_ = cmputCostLwrBound();
  else
    CmputLwrBounds_(false, 0);
  //TODO JEFF do we need to re CmputLwrBounds_ after resetting DDG?

  // Cost calculation must be below lower bounds calculation
  if (HeuristicSchedulerEnabled || isSecondPass_) {
    heuristicScheduleLength = lstSched->GetCrntLngth();
    InstCount hurstcExecCost;
    // Compute cost for Heuristic list scheduler, this must be called before
    // calling GetCost() on the InstSchedule instance.
    CmputNormCost_(lstSched, CCM_DYNMC, hurstcExecCost, true);
    hurstcCost_ = lstSched->GetCost();

    // This schedule is optimal so ACO will not be run
    // so set bestSched here.
    if (hurstcCost_ == 0) {
      isLstOptml = true;
      bestSched = bestSched_ = lstSched;
      bestSchedLngth_ = heuristicScheduleLength;
      bestCost_ = hurstcCost_;
      *OptimalSolverID_ = 0;
    }

    static_cast<BBInterfacer *>(this)->resetRegFields();

    FinishHurstc_();

    Logger::Event("HeuristicResult", "length", heuristicScheduleLength, //
                  "spill_cost", lstSched->GetSpillCost(), "cost", hurstcCost_);
    // TODO(justin): Remove once relevant scripts have been updated:
    // get-sched-length.py, runspec-wrapper-SLIL.py
    Logger::Info(
        "The list schedule is of length %d and spill cost %d. Tot cost = %d",
        heuristicScheduleLength, lstSched->GetSpillCost(), hurstcCost_);

#ifdef IS_DEBUG_PRINT_SCHEDS
    lstSched->Print(Logger::GetLogStream(), "Heuristic");
#endif
#ifdef IS_DEBUG_PRINT_BOUNDS
    dataDepGraph_->PrintLwrBounds(DIR_FRWRD, Logger::GetLogStream(),
                                  "CP Lower Bounds");
#endif
  }

  // Log the lower bound on the cost, allowing tools reading the log to compare
  // absolute rather than relative costs.
  Logger::Event("CostLowerBound", "cost", costLwrBound_);
  // TODO(justin): Remove once relevant scripts have been updated:
  // plaidbench-validation-test.py, runspec-wrapper-SLIL.py
  Logger::Info("Lower bound of cost before scheduling: %d", costLwrBound_);

  // TODO - unintended consequences?
  // Need to clear the thread dependent structures before reusing
  if (isParallelBB && !isSecondPass_)
    dataDepGraph_->resetThreadWriteFields();

  // Step #2: Use ACO to find a schedule if enabled and no optimal schedule is
  // yet to be found.
  if (AcoBeforeEnum && !isLstOptml) {
    AcoStart = Utilities::GetProcessorTime();
    AcoSchedule = new InstSchedule(machMdl_, dataDepGraph_, vrfySched_);

    rslt = runACO(AcoSchedule, lstSched, false);
    if (rslt != RES_SUCCESS) {
      llvm::report_fatal_error(llvm::StringRef("ACO scheduling failed"), false);
      if (lstSchdulr)
        delete lstSchdulr;
      if (lstSched)
        delete lstSched;
      delete AcoSchedule;
      return rslt;
    }

    AcoTime = Utilities::GetProcessorTime() - AcoStart;
    stats::AcoTime.Record(AcoTime);
    if (AcoTime > 0)
      Logger::Info("ACO_Time %d", AcoTime);

    AcoScheduleLength_ = AcoSchedule->GetCrntLngth();
    AcoScheduleCost_ = AcoSchedule->GetCost();

    // If ACO is run then that means either:
    // 1.) Heuristic was not run
    // 2.) Heuristic was not optimal
    // In both cases, the current best will be ACO if
    // ACO is optimal so set bestSched here.
    if (AcoScheduleCost_ == 0) {
      isLstOptml = true;
      bestSched = bestSched_ = AcoSchedule;
      bestSchedLngth_ = AcoScheduleLength_;
      bestCost_ = AcoScheduleCost_;

      // TODO -- when using parallel ACO, make sure ID is correctly set
      *OptimalSolverID_ = 0;
    }

    // Need to clear the thread dependent structures before reusing
    if (isParallelBB && !isSecondPass_)
      dataDepGraph_->resetThreadWriteFields();
  }

  // If an optimal schedule was found then it should have already
  // been taken care of when optimality was discovered.
  // Thus we only account for cases where no optimal schedule
  // was found.
  if (!isLstOptml) {
    // There are 3 possible situations:
    // A) ACO was never run. In that case, just use Heuristic and run with its
    // results, into B&B.
    if (!AcoBeforeEnum) {
      bestSched = bestSched_ = lstSched;
      bestSchedLngth_ = heuristicScheduleLength;
      bestCost_ = hurstcCost_;
    }
    // B) Heuristic was never run. In that case, just use ACO and run with its
    // results, into B&B.
    else if (!HeuristicSchedulerEnabled) {
      bestSched = bestSched_ = AcoSchedule;
      bestSchedLngth_ = AcoScheduleLength_;
      bestCost_ = AcoScheduleCost_;
      // C) Neither scheduler was optimal. In that case, compare the two
      // schedules and use the one that's better as the input (initialSched) for
      // B&B.
    } else {
      bestSched_ = AcoScheduleCost_ < hurstcCost_ ? AcoSchedule : lstSched;
      bestSched = bestSched_;
      bestSchedLngth_ = bestSched_->GetCrntLngth();
      bestCost_ = bestSched_->GetCost();
    }
  }
  // Step #3: Compute the cost upper bound.
  Milliseconds boundStart = Utilities::GetProcessorTime();
  assert(bestSchedLngth_ >= schedLwrBound_);
  assert(schedLwrBound_ <= bestSched_->GetCrntLngth());

  // Calculate upper bounds with the best schedule found
  CmputUprBounds_(bestSched_, false);
  boundTime = Utilities::GetProcessorTime() - boundStart;
  stats::boundComputationTime.Record(boundTime);

#ifdef IS_DEBUG_PRINT_SCHEDS
  lstSched->Print(Logger::GetLogStream(), "Heuristic");
#endif
#ifdef IS_DEBUG_PRINT_BOUNDS
  dataDepGraph_->PrintLwrBounds(DIR_FRWRD, Logger::GetLogStream(),
                                "CP Lower Bounds");
#endif

  // (Chris): If the cost function is SLIL, then the list schedule is considered
  // optimal if PERP is 0.
  if (filterByPerp && !isLstOptml && spillCostFunc_ == SCF_SLIL) {
    const InstCount *regPressures = nullptr;
    auto regTypeCount = lstSched->GetPeakRegPressures(regPressures);
    InstCount sumPerp = 0;
    for (int i = 0; i < regTypeCount; ++i) {
      int perp = regPressures[i] - machMdl_->GetPhysRegCnt(i);
      if (perp > 0)
        sumPerp += perp;
    }
    if (sumPerp == 0) {
      isLstOptml = true;
      Logger::Info("Marking SLIL list schedule as optimal due to zero PERP.");
    }
  }

#if defined(IS_DEBUG_SLIL_OPTIMALITY)
  // (Chris): This code prints a statement when a schedule is SLIL-optimal but
  // not PERP-optimal.
  if (spillCostFunc_ == SCF_SLIL && bestCost_ == 0) {
    const InstCount *regPressures = nullptr;
    auto regTypeCount = lstSched->GetPeakRegPressures(regPressures);
    InstCount sumPerp = 0;
    for (int i = 0; i < regTypeCount; ++i) {
      int perp = regPressures[i] - machMdl_->GetPhysRegCnt(i);
      if (perp > 0)
        sumPerp += perp;
    }
    if (sumPerp > 0) {
      Logger::Info("Dag %s is SLIL optimal but not PERP optimal (PERP=%d).",
                   dataDepGraph_->GetDagID(), sumPerp);
    }
  }
#endif
  if (EnableEnum_() == false) {
    delete lstSchdulr;
    return RES_FAIL;
  }

#ifdef IS_DEBUG_BOUNDS
  Logger::Info("Sched LB = %d, Sched UB = %d", schedLwrBound_, schedUprBound_);
#endif

  InitialSchedule = bestSched_;
  InitialScheduleCost = bestCost_;
  InitialScheduleLength = bestSchedLngth_;
  // Step #4: Find the optimal schedule if the heuristic and ACO was not
  // optimal.
  if (BbSchedulerEnabled) {
    Milliseconds enumStart = Utilities::GetProcessorTime();
    if (!isLstOptml) {
      dataDepGraph_->SetHard(true);
      if (isSecondPass_ && dataDepGraph_->GetMaxLtncy() <= 1) {
        *OptimalSolverID_ = 0;
        Logger::Info("Problem size not increased after introducing latencies, "
                     "skipping second pass enumeration");
      }
      else
        rslt = Optimize_(enumStart, rgnTimeout, lngthTimeout, OptimalSolverID_);
      Milliseconds enumTime = Utilities::GetProcessorTime() - enumStart;

      // TODO: Implement this stat for ACO also.
      if (hurstcTime > 0) {
        enumTime /= hurstcTime;
        stats::enumerationToHeuristicTimeRatio.Record(enumTime);
      }

      Logger::Info("bestCost_ %d, InitialSchedulCost %d", bestCost_, InitialScheduleCost);
      if (bestCost_ < InitialScheduleCost) {
        assert(enumBestSched_ != NULL);
        bestSched = bestSched_ = enumBestSched_;
#ifdef IS_DEBUG_PRINT_SCHEDS
        enumBestSched_->Print(Logger::GetLogStream(), "Optimal");
#endif
      }
    } else 
    if (rgnTimeout == 0) {
      Logger::Event("BypassZeroTimeLimit", "cost", bestCost_);
      // TODO(justin): Remove once relevant scripts have been updated:
      // runspec-wrapper-SLIL.py
      Logger::Info(
          "Bypassing optimal scheduling due to zero time limit with cost %d",
          bestCost_);
    } else {
      Logger::Event("HeuristicScheduleOptimal", "length", bestSchedLngth_,
                    "cost", bestCost_);
    }

    if (rgnTimeout != 0) {
      bool optimalSchedule = isLstOptml || (rslt == RES_SUCCESS);
      Logger::Event("BestResult", "name", dataDepGraph_->GetDagID(), //
                    "cost", bestCost_, "length", bestSchedLngth_,  //
                    "spillCost", (bestCost_ - (bestSchedLngth_ * 100 - costLwrBound_))/SPILLCOSTWEIGHT,  
                    "optimal", optimalSchedule);
      // TODO(justin): Remove once relevant scripts have been updated:
      // get-sched-length.py, plaidbench-validation-test.py
      Logger::Info("Best schedule for DAG %s has cost %d and length %d. The "
                   "schedule is %s",
                   dataDepGraph_->GetDagID(), bestCost_, bestSchedLngth_,
                   optimalSchedule ? "optimal" : "not optimal");
    }

#ifdef IS_DEBUG_PRINT_PERP_AT_EACH_STEP
    Logger::Info("Printing PERP at each step in the schedule.");

    int costSum = 0;
    for (int i = 0; i < dataDepGraph_->GetInstCnt(); ++i) {
      Logger::Info("Cycle: %lu Cost: %lu", i, bestSched_->GetSpillCost(i));
      costSum += bestSched_->GetSpillCost(i);
    }
    Logger::Info("Cost Sum: %lu", costSum);
#endif

    if (SchedulerOptions::getInstance().GetString(
            "SIMULATE_REGISTER_ALLOCATION") != "NO") {
      //#ifdef IS_DEBUG
      RegAlloc_(bestSched, InitialSchedule);
      //#endif
    }

    enumTime = Utilities::GetProcessorTime() - enumStart;
    stats::enumerationTime.Record(enumTime);

    //bestSched_->Print(std::cout, "Best Schedule after Enum");
  }

  // Step 5: Run ACO if schedule from enumerator is not optimal
  if (bestCost_ != 0 && AcoAfterEnum) {
    Logger::Info("Final cost is not optimal, running ACO.");
    InstSchedule *AcoAfterEnumSchedule =
        new InstSchedule(machMdl_, dataDepGraph_, vrfySched_);

    FUNC_RESULT acoRslt = runACO(AcoAfterEnumSchedule, bestSched, true);
    if (acoRslt != RES_SUCCESS) {
      Logger::Info("Running final ACO failed");
      delete AcoAfterEnumSchedule;
    } else {
      InstCount AcoAfterEnumCost = AcoAfterEnumSchedule->GetCost();
      if (AcoAfterEnumCost < bestCost_) {
        InstCount AcoAfterEnumLength = AcoAfterEnumSchedule->GetCrntLngth();
        InstCount imprvmnt = bestCost_ - AcoAfterEnumCost;
        Logger::Info(
            "ACO found better schedule with length=%d, spill cost = %d, "
            "tot cost = %d, cost imp=%d.",
            AcoAfterEnumLength, AcoAfterEnumSchedule->GetSpillCost(),
            AcoAfterEnumCost, imprvmnt);
        bestSched_ = bestSched = AcoAfterEnumSchedule;
        bestCost_ = AcoAfterEnumCost;
        bestSchedLngth_ = AcoAfterEnumLength;
      } else {
        Logger::Info("ACO was unable to find a better schedule.");
        delete AcoAfterEnumSchedule;
      }
    }
  }

  // What to do when solver finds better sched but not optimal
  Milliseconds vrfyStart = Utilities::GetProcessorTime();
  if (vrfySched_) {
    bool isValidSchdul = bestSched->Verify(machMdl_, dataDepGraph_, *OptimalSolverID_);

    if (isValidSchdul == false) {
      stats::invalidSchedules++;
    }
  }

  vrfyTime = Utilities::GetProcessorTime() - vrfyStart;
  stats::verificationTime.Record(vrfyTime);

  InstCount finalLwrBound = costLwrBound_;
  InstCount finalUprBound = costLwrBound_ + bestCost_;
  if (rslt == RES_SUCCESS)
    finalLwrBound = finalUprBound;

  dataDepGraph_->SetFinalBounds(finalLwrBound, finalUprBound);

  FinishOptml_();

  
  bool tookBest = ChkSchedule_(bestSched, InitialSchedule);
  if (tookBest == false) {
    bestCost_ = InitialScheduleCost;
    bestSchedLngth_ = InitialScheduleLength;
  }

  if (lstSchdulr) {
    delete lstSchdulr;
  }
  if (NULL != lstSched && bestSched != lstSched) {
    delete lstSched;
  }
  if (NULL != AcoSchedule && bestSched != AcoSchedule) {
    delete AcoSchedule;
  }
  if (enumBestSched_ != NULL && (bestSched != enumBestSched_ || enumBestSched_ == lstSched))
    delete enumBestSched_;
  if (enumCrntSched_ != NULL)
    delete enumCrntSched_;

  bestCost = bestCost_;
  bestSchedLngth = bestSchedLngth_;
  hurstcCost = hurstcCost_;
  hurstcSchedLngth = heuristicScheduleLength;

  // (Chris): Experimental. Discard the schedule based on sched.ini setting.
  if (spillCostFunc_ == SCF_SLIL) {
    bool optimal = isLstOptml || (rslt == RES_SUCCESS);
    if ((blocksToKeep == BLOCKS_TO_KEEP::ZERO_COST && bestCost != 0) ||
        (blocksToKeep == BLOCKS_TO_KEEP::OPTIMAL && !optimal) ||
        (blocksToKeep == BLOCKS_TO_KEEP::IMPROVED &&
         !(bestCost < InitialScheduleCost)) ||
        (blocksToKeep == BLOCKS_TO_KEEP::IMPROVED_OR_OPTIMAL &&
         !(optimal || bestCost < InitialScheduleCost))) {
      delete bestSched;
      bestSched = nullptr;
      return rslt;
    }
  }

  // TODO: Update this to account for using heuristic scheduler and ACO.
#if defined(IS_DEBUG_COMPARE_SLIL_BB)
  {
    const auto &status = [&]() {
      switch (rslt) {
      case RES_SUCCESS:
        return "optimal";
      case RES_TIMEOUT:
        return "timeout";
      default:
        return "failed";
      }
    }();
    if (!isLstOptml) {
      Logger::Info("Dag %s %s cost %d time %lld", dataDepGraph_->GetDagID(),
                   status, bestCost_, enumTime);
      Logger::Info("Dag %s %s absolute cost %d time %lld",
                   dataDepGraph_->GetDagID(), status, bestCost_ + costLwrBound_,
                   enumTime);
    }
  }
  {
    if (spillCostFunc_ == SCF_SLIL && rgnTimeout != 0) {
      // costLwrBound_: static lower bound
      // bestCost_: total cost of the best schedule relative to static lower
      // bound

      auto isEnumerated = [&]() { return (!isLstOptml) ? "True" : "False"; }();

      auto isOptimal = [&]() {
        return (isLstOptml || (rslt == RES_SUCCESS)) ? "True" : "False";
      }();

      auto isPerpHigherThanHeuristic = [&]() {
        auto getSumPerp = [&](InstSchedule *sched) {
          const InstCount *regPressures = nullptr;
          auto regTypeCount = sched->GetPeakRegPressures(regPressures);
          InstCount sumPerp = 0;
          for (int i = 0; i < regTypeCount; ++i) {
            int perp = regPressures[i] - machMdl_->GetPhysRegCnt(i);
            if (perp > 0)
              sumPerp += perp;
          }
          return sumPerp;
        };

        if (lstSched == bestSched)
          return "False";

        auto heuristicPerp = getSumPerp(lstSched);
        auto bestPerp = getSumPerp(bestSched);

        return (bestPerp > heuristicPerp) ? "True" : "False";
      }();

      Logger::Event("SlilStats", "name", dataDepGraph_->GetDagID(), //
                    "static_lb", costLwrBound_, "gap_size", bestCost_,
                    "is_enumerated", isEnumerated, "is_optimal", isOptimal,
                    "is_perp_higher", isPerpHigherThanHeuristic);
      // TODO(justin): Remove once relevant scripts have been updated:
      // gather-SLIL-stats.py
      Logger::Info("SLIL stats: DAG %s static LB %d gap size %d enumerated %s "
                   "optimal %s PERP higher %s",
                   dataDepGraph_->GetDagID(), costLwrBound_, bestCost_,
                   isEnumerated, isOptimal, isPerpHigherThanHeuristic);
    }
  }
#endif
#if defined(IS_DEBUG_FINAL_SPILL_COST)
  // (Chris): Unconditionally Print out the spill cost of the final schedule.
  // This makes it easy to compare results.
  Logger::Info("Final spill cost is %d for DAG %s.", bestSched_->GetSpillCost(),
               dataDepGraph_->GetDagID());
#endif
#if defined(IS_DEBUG_PRINT_PEAK_FOR_ENUMERATED)
  if (!isLstOptml) {
    InstCount maxSpillCost = 0;
    for (int i = 0; i < dataDepGraph_->GetInstCnt(); ++i) {
      if (bestSched->GetSpillCost(i) > maxSpillCost)
        maxSpillCost = bestSched->GetSpillCost(i);
    }
    Logger::Info("DAG %s PEAK %d", dataDepGraph_->GetDagID(), maxSpillCost);
  }
#endif
  return rslt;
}

FUNC_RESULT SchedRegion::Optimize_(Milliseconds startTime,
                                   Milliseconds rgnTimeout,
                                   Milliseconds lngthTimeout,
                                   int *OptimalSolverID) {
  Enumerator *enumrtr = NULL;
  FUNC_RESULT rslt = RES_SUCCESS;

  enumCrntSched_ = AllocNewSched_();
  enumBestSched_ = AllocNewSched_();

  InstCount initCost = bestCost_;
  
  Milliseconds timeout = IsTimeoutPerInst_ ? lngthTimeout : rgnTimeout;
  enumrtr = AllocEnumrtr_(timeout, EnumNodeAllocs_, HistNodeAllocs_, HashTablAllocs_);
  
  if (enumrtr) {
    //#ifndef IS_TRACK_INFSBLTY_HITS
    //  #define IS_TRACK_INFSBLTY_HITS
    //#endif
    enumBestSched_->Copy(bestSched_);
    rslt = Enumerate_(startTime, rgnTimeout, lngthTimeout, OptimalSolverID);
    Logger::Event("NodeExamineCount", "num_nodes", getExaminedNodeCount());
    stats::nodeCount.Record(getExaminedNodeCount());
  
  }

  else {
    Logger::Info("Pruned the whole tree!");
    delete enumCrntSched_;
    enumCrntSched_ = NULL;
    delete enumBestSched_;
    enumBestSched_ = NULL;
    rslt = RES_SUCCESS;     // we cost pruned the whole enum tree
    Logger::Event("NodeExamineCount", "num_nodes", 1);
    stats::nodeCount.Record(1);
    *OptimalSolverID = 0;
  }

  Milliseconds solutionTime = Utilities::GetProcessorTime() - startTime;
  stats::solutionTime.Record(solutionTime);

  const InstCount improvement = initCost - bestCost_;
  if (rslt == RES_SUCCESS) {
    Logger::Event("DagSolvedOptimally", "solution_time", solutionTime, //
                  "length", bestSchedLngth_,                           //
                  "spill_cost", bestSched_->GetSpillCost(),            //
                  "total_cost", bestCost_, "cost_improvement", improvement);
    // TODO(justin): Remove once relevant scripts have been updated:
    // runspec-wrapper-SLIL.py
    Logger::Info("DAG solved optimally in %lld ms with "
                 "length=%d, spill cost = %d, tot cost = %d, cost imp=%d.",
                 solutionTime, bestSchedLngth_, bestSched_->GetSpillCost(),
                 bestCost_, improvement);
    stats::solvedProblemSize.Record(dataDepGraph_->GetInstCnt());
    stats::solutionTimeForSolvedProblems.Record(solutionTime);
  } else {
    if (rslt == RES_TIMEOUT) {
      Logger::Event("DagTimedOut", "length", bestSchedLngth_, //
                    "spill_cost", bestSched_->GetSpillCost(), //
                    "total_cost", bestCost_, "cost_improvement", improvement);
    }
    stats::unsolvedProblemSize.Record(dataDepGraph_->GetInstCnt());
  }

  return rslt;
}

void SchedRegion::CmputLwrBounds_(bool useFileBounds, int SolverID) {
  RelaxedScheduler *rlxdSchdulr = NULL;
  RelaxedScheduler *rvrsRlxdSchdulr = NULL;
  InstCount rlxdUprBound = dataDepGraph_->GetAbslutSchedUprBound();

  switch (lbAlg_) {
  case LBA_LC:
    rlxdSchdulr = new LC_RelaxedScheduler(dataDepGraph_, machMdl_, rlxdUprBound,
                                          DIR_FRWRD, SolverID);
    rvrsRlxdSchdulr = new LC_RelaxedScheduler(dataDepGraph_, machMdl_,
                                              rlxdUprBound, DIR_BKWRD, SolverID);
    break;
  case LBA_RJ:
    rlxdSchdulr = new RJ_RelaxedScheduler(dataDepGraph_, machMdl_, rlxdUprBound,
                                          DIR_FRWRD, RST_STTC, SolverID);
    rvrsRlxdSchdulr = new RJ_RelaxedScheduler(
        dataDepGraph_, machMdl_, rlxdUprBound, DIR_BKWRD, RST_STTC, SolverID);
    break;
  }

  InstCount frwrdLwrBound = 0;
  InstCount bkwrdLwrBound = 0;
  frwrdLwrBound = rlxdSchdulr->FindSchedule();
  bkwrdLwrBound = rvrsRlxdSchdulr->FindSchedule();
  InstCount rlxdLwrBound = std::max(frwrdLwrBound, bkwrdLwrBound);

  assert(rlxdLwrBound >= schedLwrBound_);

  if (rlxdLwrBound > schedLwrBound_)
    schedLwrBound_ = rlxdLwrBound;

#ifdef IS_DEBUG_PRINT_BOUNDS
  dataDepGraph_->PrintLwrBounds(DIR_FRWRD, Logger::GetLogStream(),
                                "Relaxed Forward Lower Bounds");
  dataDepGraph_->PrintLwrBounds(DIR_BKWRD, Logger::GetLogStream(),
                                "Relaxed Backward Lower Bounds");
#endif

  if (useFileBounds)
    UseFileBounds_();

  costLwrBound_ = cmputCostLwrBound();

  delete rlxdSchdulr;
  delete rvrsRlxdSchdulr;
}

bool SchedRegion::CmputUprBounds_(InstSchedule *schedule, bool useFileBounds) {
  if (useFileBounds) {
    hurstcCost_ = dataDepGraph_->GetFileCostUprBound();
    hurstcCost_ -= costLwrBound_;
  }

  if (bestCost_ == 0) {
    // If the heuristic schedule is optimal, we are done!
    schedUprBound_ = bestSchedLngth_;
    return true;
  } else if (isSecondPass_) {
    // In the second pass, the upper bound is the length of the min-RP schedule
    // that was found in the first pass with stalls inserted.
    schedUprBound_ = schedule->GetCrntLngth();
    return false;
  } else {
    CmputSchedUprBound_();
    return false;
  }

  return false;
}

SPILL_COST_FUNCTION SchedRegion::GetSpillCostFunc() { return spillCostFunc_; }

void SchedRegion::HandlEnumrtrRslt_(FUNC_RESULT rslt, InstCount trgtLngth) {
  switch (rslt) {
  case RES_FAIL:
    //    #ifdef IS_DEBUG_ENUM_ITERS
    Logger::Info("No feasible solution of length %d was found.", trgtLngth);
    //    #endif
    break;
  case RES_SUCCESS:
#ifdef IS_DEBUG_ENUM_ITERS
    Logger::Info("Found a feasible solution of length %d.", trgtLngth);
#endif
    break;
  case RES_TIMEOUT:
    //    #ifdef IS_DEBUG_ENUM_ITERS
    Logger::Info("Enumeration timedout at length %d.", trgtLngth);
    //    #endif
    break;
  case RES_ERROR:
    Logger::Info("The processing of DAG \"%s\" was terminated with an error.",
                 dataDepGraph_->GetDagID(), rgnNum_);
    break;
  case RES_END:
    //    #ifdef IS_DEBUG_ENUM_ITERS
    Logger::Info("Enumeration ended at length %d.", trgtLngth);
    //    #endif
    break;
  }
}


void SchedRegion::RegAlloc_(InstSchedule *&bestSched, InstSchedule *&lstSched) {
  std::unique_ptr<LocalRegAlloc> u_regAllocBest = nullptr;
  std::unique_ptr<LocalRegAlloc> u_regAllocList = nullptr;
  const LocalRegAlloc *regAllocChoice = nullptr;

  if (SchedulerOptions::getInstance().GetString(
          "SIMULATE_REGISTER_ALLOCATION") == "HEURISTIC" ||
      SchedulerOptions::getInstance().GetString(
          "SIMULATE_REGISTER_ALLOCATION") == "BOTH" ||
      SchedulerOptions::getInstance().GetString(
          "SIMULATE_REGISTER_ALLOCATION") == "TAKE_SCHED_WITH_LEAST_SPILLS") {
    // Simulate register allocation using the heuristic schedule.
    u_regAllocList = std::unique_ptr<LocalRegAlloc>(
        new LocalRegAlloc(lstSched, dataDepGraph_));
    regAllocChoice = u_regAllocList.get();

    u_regAllocList->SetupForRegAlloc();
    u_regAllocList->AllocRegs();

    Logger::Event("HeuristicLocalRegAllocSimulation",           //
                  "dag_name", dataDepGraph_->GetDagID(),        //
                  "num_spills", u_regAllocList->GetCost(),      //
                  "num_stores", u_regAllocList->GetNumStores(), //
                  "num_loads", u_regAllocList->GetNumLoads());
  }
  if (SchedulerOptions::getInstance().GetString(
          "SIMULATE_REGISTER_ALLOCATION") == "BEST" ||
      SchedulerOptions::getInstance().GetString(
          "SIMULATE_REGISTER_ALLOCATION") == "BOTH" ||
      SchedulerOptions::getInstance().GetString(
          "SIMULATE_REGISTER_ALLOCATION") == "TAKE_SCHED_WITH_LEAST_SPILLS") {
    // Simulate register allocation using the best schedule.
    u_regAllocBest = std::unique_ptr<LocalRegAlloc>(
        new LocalRegAlloc(bestSched, dataDepGraph_));
    regAllocChoice = u_regAllocBest.get();

    u_regAllocBest->SetupForRegAlloc();
    u_regAllocBest->AllocRegs();

    totalSimSpills_ = u_regAllocBest->GetCost();

    Logger::Event("BestLocalRegAllocSimulation",                //
                  "dag_name", dataDepGraph_->GetDagID(),        //
                  "num_spills", u_regAllocBest->GetCost(),      //
                  "num_stores", u_regAllocBest->GetNumStores(), //
                  "num_loads", u_regAllocBest->GetNumLoads());
  }

  if (SchedulerOptions::getInstance().GetString(
          "SIMULATE_REGISTER_ALLOCATION") == "TAKE_SCHED_WITH_LEAST_SPILLS") {
    if (u_regAllocList->GetCost() < u_regAllocBest->GetCost()) {
      bestSched = lstSched;
      regAllocChoice = u_regAllocList.get();
#ifdef IS_DEBUG
      Logger::Info(
          "Taking list schedule because of less spilling with simulated RA.");
#endif
    } else {
      regAllocChoice = u_regAllocBest.get();
    }
  }

  Logger::Event("LocalRegAllocSimulationChoice",              //
                "dag_name", dataDepGraph_->GetDagID(),        //
                "num_spills", regAllocChoice->GetCost(),      //
                "num_stores", regAllocChoice->GetNumStores(), //
                "num_loads", regAllocChoice->GetNumLoads());
}

void SchedRegion::InitSecondPass() { isSecondPass_ = true; }

FUNC_RESULT SchedRegion::runACO(InstSchedule *ReturnSched,
                                InstSchedule *InitSched, bool IsPostBB) {
  // TODO(JEFF) ID Should be specific if using combined B&B / ACO parallel approach
  int SolverID = 0;
  InitForSchdulng();
  ACOScheduler *AcoSchdulr =
      new ACOScheduler(dataDepGraph_, machMdl_, abslutSchedUprBound_,
                       hurstcPrirts_, vrfySched_, IsPostBB, SolverID);
  AcoSchdulr->setInitialSched(InitSched);
  FUNC_RESULT Rslt = AcoSchdulr->FindSchedule(ReturnSched, this);
  delete AcoSchdulr;
  return Rslt;
}

// ported from BBWithSpill
ConstrainedScheduler *SchedRegion::AllocHeuristicScheduler_() {
  switch (GetHeuristicSchedulerType()) {
  case SCHED_LIST:
    return new ListScheduler(dataDepGraph_, machMdl_, abslutSchedUprBound_,
                             GetHeuristicPriorities());
    break;
  case SCHED_SEQ:
    return new SequentialListScheduler(dataDepGraph_, machMdl_,
                                       abslutSchedUprBound_,
                                       GetHeuristicPriorities());
    break;
  }
  llvm_unreachable("Unknown heuristic scheduler type!");
}

void SchedRegion::FinishHurstc_() {

#ifdef IS_DEBUG_BBSPILL_COST
  stats::traceCostLowerBound.Record(costLwrBound_);
  stats::traceHeuristicCost.Record(hurstcCost_);
  stats::traceHeuristicScheduleLength.Record(hurstcSchedLngth_);
#endif
}
