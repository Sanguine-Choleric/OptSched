#include "opt-sched/Scheduler/sched_basic_data.h"
#include "opt-sched/Scheduler/register.h"
#include "opt-sched/Scheduler/stats.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/ErrorHandling.h"
#include <string>

using namespace llvm::opt_sched;

SchedInstruction::SchedInstruction(InstCount num, const string &name,
                                   InstType instType, const string &opCode,
                                   InstCount maxInstCnt, int nodeID,
                                   InstCount fileSchedOrder,
                                   InstCount fileSchedCycle, InstCount fileLB,
                                   InstCount fileUB, MachineModel *model, 
                                   const int NumSolvers)
    : GraphNode(num, maxInstCnt, NumSolvers) {

  NumSolvers_ = NumSolvers;
  
  // Static data that is computed only once.
  name_ = name;
  opCode_ = opCode;
  instType_ = instType;

  frwrdLwrBound_ = INVALID_VALUE;         
  bkwrdLwrBound_ = INVALID_VALUE;
  abslutFrwrdLwrBound_ = INVALID_VALUE;   
  abslutBkwrdLwrBound_ = INVALID_VALUE;
  crtclPathFrmRoot_ = INVALID_VALUE;
  crtclPathFrmLeaf_ = INVALID_VALUE;

  ltncyPerPrdcsr_ = NULL;
  memAllocd_ = false;
  sortedPrdcsrLst_ = NULL;
  sortedScsrLst_ = NULL;

  crtclPathFrmRcrsvScsr_ = NULL;
  crtclPathFrmRcrsvPrdcsr_ = NULL;

  // Dynamic data that changes during scheduling.
  rdyCyclePerPrdcsr_ = NULL;
  prevMinRdyCyclePerPrdcsr_ = NULL;
  unschduldPrdcsrCnt_ = NULL;
  unschduldScsrCnt_ = NULL;

  crntRlxdCycle_ = SCHD_UNSCHDULD;
  sig_ = 0;
  preFxdCycle_ = INVALID_VALUE;

  blksCycle_ = model->BlocksCycle(instType);
  pipelined_ = model->IsPipelined(instType);

  defCnt_ = 0;
  useCnt_ = 0;

  nodeID_ = nodeID;
  fileSchedOrder_ = fileSchedOrder;
  fileSchedCycle_ = fileSchedCycle;
  fileLwrBound_ = fileLB;
  fileUprBound_ = fileUB;

  mustBeInBBEntry_ = false;
  mustBeInBBExit_ = false;
}

SchedInstruction::~SchedInstruction() {
  
  if (memAllocd_)
    DeAllocMem_();

}

void SchedInstruction::resetThreadWriteFields(int SolverID, bool full) {
  resetGraphNodeThreadWriteFields(SolverID);

  if (SolverID == -1) {  
    for (int SolverID_ = 0; SolverID_ < NumSolvers_; SolverID_++)
    {
      // currently we dont use sortedScsrLst_
      /*if (sortedScsrLst_[SolverID] != NULL)
        delete sortedScsrLst_[SolverID];*/
      if (sortedPrdcsrLst_ != NULL) 
        if (sortedPrdcsrLst_[SolverID_] != NULL) 
          delete sortedPrdcsrLst_[SolverID_]; 
      if (rdyCyclePerPrdcsr_ != NULL) 
        if (rdyCyclePerPrdcsr_[SolverID_] != NULL) 
          delete[] rdyCyclePerPrdcsr_[SolverID_]; 
      if (prevMinRdyCyclePerPrdcsr_ != NULL) 
        if (prevMinRdyCyclePerPrdcsr_[SolverID_] != NULL) 
          delete[] prevMinRdyCyclePerPrdcsr_[SolverID_];
      /*if (crntRange_ != NULL)
        if (crntRange_[SolverID_] != NULL)
          delete crntRange_[SolverID];*/
    }
  
    if (rdyCyclePerPrdcsr_ != NULL)
      delete[] rdyCyclePerPrdcsr_; 
    if (prevMinRdyCyclePerPrdcsr_ != NULL) 
      delete[] prevMinRdyCyclePerPrdcsr_;
    if (sortedPrdcsrLst_ != NULL) 
      delete[] sortedPrdcsrLst_;
    if (sortedScsrLst_ != NULL) 
      delete[] sortedScsrLst_;
    //if (crntRange_ != NULL)
    //  delete[] crntRange_;
    
    if (crntSchedSlot_ != NULL) 
      delete[] crntSchedSlot_;
    if (ready_ != NULL) 
      delete[] ready_;
    if (minRdyCycle_ != NULL) 
      delete[] minRdyCycle_;
    if (crntSchedCycle_ != NULL) 
      delete[] crntSchedCycle_;
    if (lastUseCnt_ != NULL) 
      delete[] lastUseCnt_;
    if (unschduldScsrCnt_ != NULL)
      delete[] unschduldScsrCnt_; 
    if (unschduldPrdcsrCnt_ != NULL)
      delete[] unschduldPrdcsrCnt_;
  
    // Alloc Fields
    ready_ = new bool[NumSolvers_];
    minRdyCycle_ = new InstCount[NumSolvers_];
    crntSchedCycle_ = new InstCount[NumSolvers_];
    lastUseCnt_ = new int16_t[NumSolvers_];
    //crntRange_ = new SchedRange*[NumSolvers_];
    unschduldScsrCnt_ = new InstCount[NumSolvers_];
    unschduldPrdcsrCnt_ = new InstCount[NumSolvers_];
    rdyCyclePerPrdcsr_ = new InstCount*[NumSolvers_];
    prevMinRdyCyclePerPrdcsr_ = new InstCount*[NumSolvers_];
    sortedPrdcsrLst_ = new PriorityList<SchedInstruction>*[NumSolvers_];
    crntSchedSlot_ = new InstCount[NumSolvers_];
  
    scsrCnt_ = GetScsrCnt();
    prdcsrCnt_ = GetPrdcsrCnt();
  
    // Initialize
    for (int SolverID_ = 0; SolverID_ < NumSolvers_; SolverID_++)
    {
      ready_[SolverID_] = false;
      minRdyCycle_[SolverID_] = INVALID_VALUE;
      crntSchedCycle_[SolverID_] = SCHD_UNSCHDULD;
      lastUseCnt_[SolverID_] = 0;
      //crntRange_[SolverID_] = new SchedRange(this);
      unschduldScsrCnt_[SolverID_] = scsrCnt_;
      unschduldPrdcsrCnt_[SolverID_] = prdcsrCnt_;
      rdyCyclePerPrdcsr_[SolverID_] = new InstCount[prdcsrCnt_];
      prevMinRdyCyclePerPrdcsr_[SolverID_] = new InstCount[prdcsrCnt_];
      sortedPrdcsrLst_[SolverID_] = new PriorityList<SchedInstruction>;
  
      for (int i = 0; i < prdcsrCnt_; i++)
      {
        rdyCyclePerPrdcsr_[SolverID_][i] = INVALID_VALUE;
        prevMinRdyCyclePerPrdcsr_[SolverID_][i] = INVALID_VALUE;
      }
    }
  
    for (GraphEdge *edge = GetFrstPrdcsrEdge(0); edge != NULL;
        edge = GetNxtPrdcsrEdge(0)) {
      for (int SolverID_ = 0; SolverID_ < NumSolvers_; SolverID_++)
        sortedPrdcsrLst_[SolverID_]->InsrtElmnt((SchedInstruction *)edge->GetOtherNode(this),
                                      edge->label, true);
    }

    for (int SolverID_ = 0; SolverID_ < NumSolvers_; SolverID_++) {
      crntRange_[SolverID_]->resetState();
      crntRange_[SolverID_]->SetBounds(frwrdLwrBound_, bkwrdLwrBound_);
      //if (GetNum() == 2 && SolverID_ == 2) Logger::Info("just set inst2 frwrdLB to %d", crntRange_[SolverID_]->GetLwrBound(DIR_FRWRD));
    }
  }

  // We are resetting a specific solver
  else {
    ready_[SolverID] = false;
    minRdyCycle_[SolverID] = INVALID_VALUE;
    crntSchedCycle_[SolverID] = SCHD_UNSCHDULD;
    lastUseCnt_[SolverID] = 0;
    //crntRange_[SolverID] = new SchedRange(this);
    unschduldScsrCnt_[SolverID] = scsrCnt_;
    unschduldPrdcsrCnt_[SolverID] = prdcsrCnt_;
    //rdyCyclePerPrdcsr_[SolverID] = new InstCount[prdcsrCnt_];
    //prevMinRdyCyclePerPrdcsr_[SolverID] = new InstCount[prdcsrCnt_];
    //sortedPrdcsrLst_[SolverID] = new PriorityList<SchedInstruction>;

    //if (GetNum() == 1)
    //  Logger::Info("schedinst %d isScheduld ? %d", GetNum(), IsSchduld(SolverID));

    for (int i = 0; i < prdcsrCnt_; i++)
    {
      rdyCyclePerPrdcsr_[SolverID][i] = INVALID_VALUE;
      prevMinRdyCyclePerPrdcsr_[SolverID][i] = INVALID_VALUE;
    }
 
    /*
    for (GraphEdge *edge = GetFrstPrdcsrEdge(SolverID); edge != NULL;
         edge = GetNxtPrdcsrEdge(SolverID)) {

        sortedPrdcsrLst_[SolverID]->InsrtElmnt((SchedInstruction *)edge->GetOtherNode(this),
                                      edge->label, true);
    }*/




    if (full) {
      if (sortedPrdcsrLst_ != NULL) 
        if (sortedPrdcsrLst_[SolverID] != NULL) 
          delete sortedPrdcsrLst_[SolverID]; 

      sortedPrdcsrLst_[SolverID] = new PriorityList<SchedInstruction>;

      for (GraphEdge *edge = GetFrstPrdcsrEdge(SolverID); edge != NULL; edge = GetNxtPrdcsrEdge(SolverID)) {
        sortedPrdcsrLst_[SolverID]->InsrtElmnt((SchedInstruction *)edge->GetOtherNode(this),
                                      edge->label, true);
      }
    
      crntRange_[SolverID]->resetState();
      crntRange_[SolverID]->SetBounds(frwrdLwrBound_, bkwrdLwrBound_);      
    }
  }
}

void SchedInstruction::SetupForSchdulng(InstCount instCnt, bool isCP_FromScsr,
                                        bool isCP_FromPrdcsr) {
  if (memAllocd_)
    DeAllocMem_();
  AllocMem_(instCnt, isCP_FromScsr, isCP_FromPrdcsr);


    SetPrdcsrNums_();
    SetScsrNums_();

  ComputeAdjustedUseCnt_();
}

bool SchedInstruction::UseFileBounds() {
  bool match = true;
#ifdef IS_DEBUG_BOUNDS
  stats::totalInstructions++;

  if (frwrdLwrBound_ == fileLwrBound_) {
    stats::instructionsWithEqualLB++;
  }

  if (fileLwrBound_ > frwrdLwrBound_) {
    stats::instructionsWithTighterFileLB++;
    stats::cyclesTightenedForTighterFileLB += fileLwrBound_ - frwrdLwrBound_;
  }

  if (frwrdLwrBound_ > fileLwrBound_) {
    stats::instructionsWithTighterRelaxedLB++;
    stats::cyclesTightenedForTighterRelaxedLB += frwrdLwrBound_ - fileLwrBound_;
  }

  if (frwrdLwrBound_ != fileLwrBound_) {
    match = false;
    Logger::Info("File LB =%d, Rec LB=%d, instNum=%d, pred Cnt=%d",
                 fileLwrBound_, frwrdLwrBound_, num_, prdcsrCnt_);
  }

  if (bkwrdLwrBound_ == fileUprBound_) {
    stats::instructionsWithEqualUB++;
  }

  if (fileUprBound_ > bkwrdLwrBound_) {
    stats::instructionsWithTighterFileUB++;
    stats::cyclesTightenedForTighterFileUB += fileUprBound_ - bkwrdLwrBound_;
  }

  if (bkwrdLwrBound_ > fileUprBound_) {
    stats::instructionsWithTighterRelaxedUB++;
    stats::cyclesTightenedForTighterRelaxedUB += bkwrdLwrBound_ - fileUprBound_;
  }

  if (bkwrdLwrBound_ != fileUprBound_) {
    match = false;
    Logger::Info("File UB =%d, Rec UB=%d, instNum=%d, pred Cnt=%d",
                 fileUprBound_, bkwrdLwrBound_, num_, prdcsrCnt_);
  }
#endif
  SetBounds(fileLwrBound_, fileUprBound_);
  return match;
}

bool SchedInstruction::InitForSchdulng(int SolverID, InstCount schedLngth, 
                                       LinkedList<SchedInstruction> *fxdLst) {
  crntSchedCycle_[SolverID] = SCHD_UNSCHDULD;
  crntRlxdCycle_ = SCHD_UNSCHDULD;

  for (InstCount i = 0; i < prdcsrCnt_; i++) {
    rdyCyclePerPrdcsr_[SolverID][i] = INVALID_VALUE;
    prevMinRdyCyclePerPrdcsr_[SolverID][i] = INVALID_VALUE;
  }

  ready_[SolverID] = false;
  minRdyCycle_[SolverID] = INVALID_VALUE;
  unschduldPrdcsrCnt_[SolverID] = prdcsrCnt_;
  unschduldScsrCnt_[SolverID] = scsrCnt_;
  lastUseCnt_[SolverID] = 0;


  if (schedLngth != INVALID_VALUE) {
    bool fsbl = crntRange_[SolverID]->SetBounds(frwrdLwrBound_, bkwrdLwrBound_,
                                      schedLngth, fxdLst);
    if (!fsbl)
      return false;
  }

  return true;


}

void SchedInstruction::AllocMem_(InstCount instCnt, bool isCP_FromScsr,
                                 bool isCP_FromPrdcsr) {
  
  // Thread dependent structures
  // TODO: cacheline dep, combine to struct
  ready_ = new bool[NumSolvers_];
  minRdyCycle_ = new InstCount[NumSolvers_];
  crntSchedCycle_ = new InstCount[NumSolvers_];
  lastUseCnt_ = new int16_t[NumSolvers_];

  crntRange_ = new SchedRange*[NumSolvers_];
  unschduldScsrCnt_ = new InstCount[NumSolvers_];
  unschduldPrdcsrCnt_ = new InstCount[NumSolvers_];
  rdyCyclePerPrdcsr_ = new InstCount*[NumSolvers_];
  prevMinRdyCyclePerPrdcsr_ = new InstCount*[NumSolvers_];
  sortedPrdcsrLst_ = new PriorityList<SchedInstruction>*[NumSolvers_];

  scsrCnt_ = GetScsrCnt();
  prdcsrCnt_ = GetPrdcsrCnt();

  for (int SolverID = 0; SolverID < NumSolvers_; SolverID++)
  {
    // Each thread needs their own memory
    ready_[SolverID] = false;
    minRdyCycle_[SolverID] = INVALID_VALUE;
    crntSchedCycle_[SolverID] = SCHD_UNSCHDULD;
    lastUseCnt_[SolverID] = 0;
    crntRange_[SolverID] = new SchedRange(this);     
    unschduldScsrCnt_[SolverID] = scsrCnt_;
    unschduldPrdcsrCnt_[SolverID] = prdcsrCnt_;
    rdyCyclePerPrdcsr_[SolverID] = new InstCount[prdcsrCnt_];
    prevMinRdyCyclePerPrdcsr_[SolverID] = new InstCount[prdcsrCnt_];
    sortedPrdcsrLst_[SolverID] = new PriorityList<SchedInstruction>;

    for (int i = 0; i < prdcsrCnt_; i++)
    {
      rdyCyclePerPrdcsr_[SolverID][i] = INVALID_VALUE;
      prevMinRdyCyclePerPrdcsr_[SolverID][i] = INVALID_VALUE;
    }
  }

  crntSchedSlot_ = new InstCount[NumSolvers_];

  ltncyPerPrdcsr_ = new InstCount[prdcsrCnt_];


  InstCount predecessorIndex = 0;
  Logger::Info("about to enter get prdedge loop");
  for (GraphEdge *edge = GetFrstPrdcsrEdge(0); edge != NULL;
       edge = GetNxtPrdcsrEdge(0)) {
    Logger::Info("found first or next pred edge");
    ltncyPerPrdcsr_[predecessorIndex++] = edge->label;
    for (int i = 0; i < NumSolvers_; i++)
      sortedPrdcsrLst_[i]->InsrtElmnt((SchedInstruction *)edge->GetOtherNode(this),
                                    edge->label, true);
  }

  if (isCP_FromScsr) {
    crtclPathFrmRcrsvScsr_ = new InstCount[instCnt];

    for (InstCount i = 0; i < instCnt; i++) {
      crtclPathFrmRcrsvScsr_[i] = INVALID_VALUE;
    }

    crtclPathFrmRcrsvScsr_[GetNum()] = 0;
  }

  if (isCP_FromPrdcsr) {
    crtclPathFrmRcrsvPrdcsr_ = new InstCount[instCnt];

    for (InstCount i = 0; i < instCnt; i++) {
      crtclPathFrmRcrsvPrdcsr_[i] = INVALID_VALUE;
    }

    crtclPathFrmRcrsvPrdcsr_[GetNum()] = 0;
  }

  memAllocd_ = true;
}

void SchedInstruction::DeAllocMem_() {
  assert(memAllocd_);

  for (int SolverID = 0; SolverID < NumSolvers_; SolverID++)
  {
    // currently we dont use sortedScsrLst_
    /*if (sortedScsrLst_[SolverID] != NULL)
      delete sortedScsrLst_[SolverID];*/
    if (sortedPrdcsrLst_ != NULL)
      if (sortedPrdcsrLst_[SolverID] != NULL)
        delete sortedPrdcsrLst_[SolverID];
    if (rdyCyclePerPrdcsr_ != NULL)
      if (rdyCyclePerPrdcsr_[SolverID] != NULL)
        delete[] rdyCyclePerPrdcsr_[SolverID];
    if (prevMinRdyCyclePerPrdcsr_ != NULL)
      if (prevMinRdyCyclePerPrdcsr_[SolverID] != NULL)
        delete[] prevMinRdyCyclePerPrdcsr_[SolverID];
    if (crntRange_ != NULL)
      if (crntRange_[SolverID] != NULL)
        delete crntRange_[SolverID];
  }

  if (rdyCyclePerPrdcsr_ != NULL)
    delete[] rdyCyclePerPrdcsr_;
  if (prevMinRdyCyclePerPrdcsr_ != NULL)
    delete[] prevMinRdyCyclePerPrdcsr_;
  if (sortedPrdcsrLst_ != NULL)
    delete[] sortedPrdcsrLst_;
  if (sortedScsrLst_ != NULL)
    delete[] sortedScsrLst_;
  if (crntRange_ != NULL)
    delete[] crntRange_;

  if (ltncyPerPrdcsr_ != NULL)
    delete[] ltncyPerPrdcsr_;
  if (crtclPathFrmRcrsvScsr_ != NULL)
    delete[] crtclPathFrmRcrsvScsr_;
  if (crtclPathFrmRcrsvPrdcsr_ != NULL)
    delete[] crtclPathFrmRcrsvPrdcsr_;
  if (crntSchedSlot_ != NULL)
    delete[] crntSchedSlot_;
  if (ready_ != NULL)
    delete[] ready_;
  if (minRdyCycle_ != NULL)
    delete[] minRdyCycle_;
  if (crntSchedCycle_ != NULL)
    delete[] crntSchedCycle_;
  if (lastUseCnt_ != NULL)
    delete[] lastUseCnt_;
  if (unschduldScsrCnt_ != NULL)
    delete[] unschduldScsrCnt_;
  if (unschduldPrdcsrCnt_ != NULL)
    delete[] unschduldPrdcsrCnt_;

  memAllocd_ = false;

}

InstCount SchedInstruction::CmputCrtclPath_(DIRECTION dir,
                                            SchedInstruction *ref) {
  // The idea of this function is considering each predecessor (successor) and
  // calculating the length of the path from the root (leaf) through that
  // predecessor (successor) and then taking the maximum value among all these
  // paths.
  InstCount crtclPath = 0;
  LinkedList<GraphEdge> *nghbrLst = GetNghbrLst(dir);

  for (GraphEdge *edg = nghbrLst->GetFrstElmnt(); edg != NULL;
       edg = nghbrLst->GetNxtElmnt()) {
    UDT_GLABEL edgLbl = edg->label;
    SchedInstruction *nghbr = (SchedInstruction *)(edg->GetOtherNode(this));

    InstCount nghbrCrtclPath;
    if (ref == NULL) {
      nghbrCrtclPath = nghbr->GetCrtclPath(dir);
    } else {
      // When computing relative critical paths, we only need to consider
      // neighbors that belong to the sub-tree rooted at the reference.
      if (!ref->IsRcrsvNghbr(dir, nghbr))
        continue;
      nghbrCrtclPath = nghbr->GetRltvCrtclPath(dir, ref);
    }
    assert(nghbrCrtclPath != INVALID_VALUE);

    if ((nghbrCrtclPath + edgLbl) > crtclPath) {
      crtclPath = nghbrCrtclPath + edgLbl;
    }
  }

  return crtclPath;
}

bool SchedInstruction::ApplyPreFxng(LinkedList<SchedInstruction> *tightndLst,
                                    LinkedList<SchedInstruction> *fxdLst, 
                                    int SolverID) {
  return crntRange_[SolverID]->Fix(preFxdCycle_, tightndLst, fxdLst, SolverID);
}

void SchedInstruction::AddDef(Register *reg) {
  if (defCnt_ >= MAX_DEFS_PER_INSTR) {
    llvm::report_fatal_error("An instruction can't have more than " +
                                 std::to_string(MAX_DEFS_PER_INSTR) + " defs",
                             false);
  }
  // Logger::Info("Inst %d defines reg %d of type %d and physNum %d and useCnt
  // %d",
  // num_, reg->GetNum(), reg->GetType(), reg->GetPhysicalNumber(),
  // reg->GetUseCnt());
  assert(reg != NULL);
  defs_[defCnt_++] = reg;
}

void SchedInstruction::AddUse(Register *reg) {
  if (useCnt_ >= MAX_USES_PER_INSTR) {
    llvm::report_fatal_error("An instruction can't have more than " +
                                 std::to_string(MAX_USES_PER_INSTR) + " uses",
                             false);
  }
  // Logger::Info("Inst %d uses reg %d of type %d and physNum %d and useCnt %d",
  // num_, reg->GetNum(), reg->GetType(), reg->GetPhysicalNumber(),
  // reg->GetUseCnt());
  assert(reg != NULL);
  uses_[useCnt_++] = reg;
}

bool SchedInstruction::FindDef(Register *reg) const {
  return llvm::any_of(GetDefs(), [reg](const Register *r) { return reg == r; });
}

bool SchedInstruction::FindUse(Register *reg) const {
  return llvm::any_of(GetUses(), [reg](const Register *r) { return reg == r; });
}

bool SchedInstruction::BlocksCycle() const { return blksCycle_; }

bool SchedInstruction::IsPipelined() const { return pipelined_; }

bool SchedInstruction::MustBeInBBEntry() const {
  return mustBeInBBEntry_;
  //  return opCode_=="CopyFromReg" || opCode_=="ADJCALLSTACKDOWN32";
}

bool SchedInstruction::MustBeInBBExit() const {
  return mustBeInBBExit_;
  //  return opCode_=="CopyToReg";
}

void SchedInstruction::SetMustBeInBBEntry(bool val) { mustBeInBBEntry_ = val; }

void SchedInstruction::SetMustBeInBBExit(bool val) { mustBeInBBExit_ = val; }

const char *SchedInstruction::GetName() const { return name_.c_str(); }

const char *SchedInstruction::GetOpCode() const { return opCode_.c_str(); }

int SchedInstruction::GetNodeID() const { return nodeID_; }

void SchedInstruction::SetNodeID(int nodeID) { nodeID_ = nodeID; }

int SchedInstruction::GetLtncySum() const { return GetScsrLblSum(); }

int SchedInstruction::GetMaxLtncy() const { return GetMaxEdgeLabel(); }

SchedInstruction *SchedInstruction::GetFrstPrdcsr(int SolverID,
                                                  InstCount *scsrNum,
                                                  UDT_GLABEL *ltncy,
                                                  DependenceType *depType) {
  GraphEdge *edge = GetFrstPrdcsrEdge(SolverID);
  if (!edge)
    return NULL;
  if (scsrNum)
    *scsrNum = edge->succOrder;
  if (ltncy)
    *ltncy = edge->label;
  if (depType)
    *depType = (DependenceType)edge->label2;
  return (SchedInstruction *)(edge->from);
}

SchedInstruction *SchedInstruction::GetNxtPrdcsr(int SolverID,
                                                 InstCount *scsrNum,
                                                 UDT_GLABEL *ltncy,
                                                 DependenceType *depType) {
  GraphEdge *edge = GetNxtPrdcsrEdge(SolverID);
  if (!edge)
    return NULL;
  if (scsrNum)
    *scsrNum = edge->succOrder;
  if (ltncy)
    *ltncy = edge->label;
  if (depType)
    *depType = (DependenceType)edge->label2;
  return (SchedInstruction *)(edge->from);
}

SchedInstruction *SchedInstruction::GetFrstScsr(int SolverID,
                                                InstCount *prdcsrNum,
                                                UDT_GLABEL *ltncy,
                                                DependenceType *depType,
                                                bool *IsArtificial) {
  GraphEdge *edge = GetFrstScsrEdge(SolverID);
  if (!edge)
    return NULL;
  if (prdcsrNum)
    *prdcsrNum = edge->predOrder;
  if (ltncy)
    *ltncy = edge->label;
  if (depType)
    *depType = (DependenceType)edge->label2;
  if (IsArtificial)
    *IsArtificial = edge->IsArtificial;
  return (SchedInstruction *)(edge->to);
}

SchedInstruction *SchedInstruction::GetNxtScsr(int SolverID,
                                               InstCount *prdcsrNum,
                                               UDT_GLABEL *ltncy,
                                               DependenceType *depType,
                                               bool *IsArtificial) {
  GraphEdge *edge = GetNxtScsrEdge(SolverID);
  if (!edge)
    return NULL;
  if (prdcsrNum)
    *prdcsrNum = edge->predOrder;
  if (ltncy)
    *ltncy = edge->label;
  if (depType)
    *depType = (DependenceType)edge->label2;
  if (IsArtificial)
    *IsArtificial = edge->IsArtificial;
  return (SchedInstruction *)(edge->to);
}

SchedInstruction *SchedInstruction::GetLastScsr(int SolverID, InstCount *prdcsrNum) {
  GraphEdge *edge = GetLastScsrEdge(SolverID);
  if (!edge)
    return NULL;
  if (prdcsrNum)
    *prdcsrNum = edge->predOrder;
  return (SchedInstruction *)(edge->to);
}

SchedInstruction *SchedInstruction::GetPrevScsr(int SolverID, InstCount *prdcsrNum) {
  GraphEdge *edge = GetPrevScsrEdge(SolverID);
  if (!edge)
    return NULL;
  if (prdcsrNum)
    *prdcsrNum = edge->predOrder;
  return (SchedInstruction *)(edge->to);
}

SchedInstruction *SchedInstruction::GetFrstNghbr(int SolverID,
                                                 DIRECTION dir,
                                                 UDT_GLABEL *ltncy) {
  GraphEdge *edge = dir == DIR_FRWRD ? GetFrstScsrEdge(SolverID) : GetFrstPrdcsrEdge(SolverID);
  if (edge == NULL)
    return NULL;
  if (ltncy)
    *ltncy = edge->label;
  return (SchedInstruction *)((dir == DIR_FRWRD) ? edge->to : edge->from);
}

SchedInstruction *SchedInstruction::GetNxtNghbr(int SolverID,
                                                DIRECTION dir,
                                                UDT_GLABEL *ltncy) {
  GraphEdge *edge = dir == DIR_FRWRD ? GetNxtScsrEdge(SolverID) : GetNxtPrdcsrEdge(SolverID);
  if (edge == NULL)
    return NULL;
  if (ltncy)
    *ltncy = edge->label;
  return (SchedInstruction *)((dir == DIR_FRWRD) ? edge->to : edge->from);
}

InstCount SchedInstruction::CmputCrtclPathFrmRoot() {
  crtclPathFrmRoot_ = CmputCrtclPath_(DIR_FRWRD);
  return crtclPathFrmRoot_;
}

InstCount SchedInstruction::CmputCrtclPathFrmLeaf() {
  crtclPathFrmLeaf_ = CmputCrtclPath_(DIR_BKWRD);
  return crtclPathFrmLeaf_;
}

InstCount
SchedInstruction::CmputCrtclPathFrmRcrsvPrdcsr(SchedInstruction *ref) {
  InstCount refInstNum = ref->GetNum();
  crtclPathFrmRcrsvPrdcsr_[refInstNum] = CmputCrtclPath_(DIR_FRWRD, ref);
  return crtclPathFrmRcrsvPrdcsr_[refInstNum];
}

InstCount SchedInstruction::CmputCrtclPathFrmRcrsvScsr(SchedInstruction *ref) {
  InstCount refInstNum = ref->GetNum();
  crtclPathFrmRcrsvScsr_[refInstNum] = CmputCrtclPath_(DIR_BKWRD, ref);
  return crtclPathFrmRcrsvScsr_[refInstNum];
}

InstCount SchedInstruction::GetCrtclPath(DIRECTION dir) const {
  return dir == DIR_FRWRD ? crtclPathFrmRoot_ : crtclPathFrmLeaf_;
}

InstCount SchedInstruction::GetRltvCrtclPath(DIRECTION dir,
                                             SchedInstruction *ref) {
  InstCount refInstNum = ref->GetNum();

  if (dir == DIR_FRWRD) {
    assert(crtclPathFrmRcrsvPrdcsr_[refInstNum] != INVALID_VALUE);
    return crtclPathFrmRcrsvPrdcsr_[refInstNum];
  } else {
    assert(dir == DIR_BKWRD);
    assert(crtclPathFrmRcrsvScsr_[refInstNum] != INVALID_VALUE);
    return crtclPathFrmRcrsvScsr_[refInstNum];
  }
}

InstCount SchedInstruction::GetLwrBound(DIRECTION dir) const {
  return dir == DIR_FRWRD ? frwrdLwrBound_ : bkwrdLwrBound_;
}

//TODO -- Should this be thead dependent?
void SchedInstruction::SetLwrBound(DIRECTION dir, InstCount bound, bool isAbslut) {
  if (dir == DIR_FRWRD) {
    assert(!isAbslut || bound >= frwrdLwrBound_);
    frwrdLwrBound_ = bound;

    if (isAbslut) {
      abslutFrwrdLwrBound_ = bound;
      for (int SolverID = 0; SolverID < NumSolvers_; SolverID++)
        crntRange_[SolverID]->SetFrwrdBound(frwrdLwrBound_);
    }
  } else {
    assert(!isAbslut || bound >= bkwrdLwrBound_);
    bkwrdLwrBound_ = bound;

    if (isAbslut) {
      abslutBkwrdLwrBound_ = bound;
      for (int SolverID = 0; SolverID < NumSolvers_; SolverID++) 
        crntRange_[SolverID]->SetBkwrdBound(bkwrdLwrBound_);
    }
  }
}

//TODO -- Should this be thead dependent?
void SchedInstruction::RestoreAbsoluteBounds() {
    frwrdLwrBound_ = abslutFrwrdLwrBound_;
    bkwrdLwrBound_ = abslutBkwrdLwrBound_;
    for (int SolverID = 0; SolverID < NumSolvers_; SolverID++) 
      crntRange_[SolverID]->SetBounds(frwrdLwrBound_, bkwrdLwrBound_);
}

//TODO -- Should this be thead dependent?
void SchedInstruction::SetBounds(InstCount flb, InstCount blb) {
  frwrdLwrBound_ = flb;
  bkwrdLwrBound_ = blb;
  abslutFrwrdLwrBound_ = flb;
  abslutBkwrdLwrBound_ = blb;
    for (int SolverID = 0; SolverID < NumSolvers_; SolverID++) 
    crntRange_[SolverID]->SetBounds(frwrdLwrBound_, bkwrdLwrBound_);
}

bool SchedInstruction::PrdcsrSchduld(InstCount prdcsrNum, InstCount cycle,
                                     InstCount &rdyCycle, int SolverID) {
  assert(prdcsrNum < prdcsrCnt_);
  rdyCyclePerPrdcsr_[SolverID][prdcsrNum] = cycle + ltncyPerPrdcsr_[prdcsrNum];
  prevMinRdyCyclePerPrdcsr_[SolverID][prdcsrNum] = minRdyCycle_[SolverID];

  if (rdyCyclePerPrdcsr_[SolverID][prdcsrNum] > minRdyCycle_[SolverID]) {
    minRdyCycle_[SolverID] = rdyCyclePerPrdcsr_[SolverID][prdcsrNum];
  }

  rdyCycle = minRdyCycle_[SolverID];
  unschduldPrdcsrCnt_[SolverID]--;
  return (unschduldPrdcsrCnt_[SolverID] == 0);
}

bool SchedInstruction::PrdcsrUnSchduld(InstCount prdcsrNum,
                                       InstCount &rdyCycle, int SolverID) {
  assert(prdcsrNum < prdcsrCnt_);
  assert(rdyCyclePerPrdcsr_[SolverID][prdcsrNum] != INVALID_VALUE);
  rdyCycle = minRdyCycle_[SolverID];
  minRdyCycle_[SolverID] = prevMinRdyCyclePerPrdcsr_[SolverID][prdcsrNum];
  rdyCyclePerPrdcsr_[SolverID][prdcsrNum] = INVALID_VALUE;
  unschduldPrdcsrCnt_[SolverID]++;
  assert(unschduldPrdcsrCnt_[SolverID] != prdcsrCnt_ || minRdyCycle_[SolverID] == INVALID_VALUE);
  return (unschduldPrdcsrCnt_[SolverID] == 1);
}

bool SchedInstruction::ScsrSchduld() {
  unschduldScsrCnt_--;
  return unschduldScsrCnt_ == 0;
}

void SchedInstruction::SetInstType(InstType type) { instType_ = type; }

void SchedInstruction::SetIssueType(IssueType type) { issuType_ = type; }

InstType SchedInstruction::GetInstType() const { return instType_; }

IssueType SchedInstruction::GetIssueType() const { return issuType_; }

bool SchedInstruction::IsSchduld(int SolverID, InstCount *cycle) const {
  if (cycle)
    *cycle = crntSchedCycle_[SolverID];
  return crntSchedCycle_[SolverID] != SCHD_UNSCHDULD;
}

InstCount SchedInstruction::GetSchedCycle(int SolverID) const { 
  return crntSchedCycle_[SolverID]; 
}

InstCount SchedInstruction::GetSchedSlot(int SolverID) const { 
  return crntSchedSlot_[SolverID]; 
}

InstCount SchedInstruction::GetCrntDeadline(int SolverID) const {
  return IsSchduld(SolverID) ? crntSchedCycle_[SolverID] : crntRange_[SolverID]->GetDeadline();
}

InstCount SchedInstruction::GetCrntReleaseTime(int SolverID) const {
  return IsSchduld(SolverID) ? crntSchedCycle_[SolverID] : GetCrntLwrBound(DIR_FRWRD, SolverID);
}

InstCount SchedInstruction::GetRlxdCycle(int SolverID) const {
  return IsSchduld(SolverID) ? crntSchedCycle_[SolverID] : crntRlxdCycle_;
}

void SchedInstruction::SetRlxdCycle(InstCount cycle) { crntRlxdCycle_ = cycle; }

void SchedInstruction::Schedule(InstCount cycleNum, InstCount slotNum, int SolverID) {
  //Logger::Info("SolverID %d Scheduling %d", SolverID, GetNum());
  assert(crntSchedCycle_[SolverID] == SCHD_UNSCHDULD);
  crntSchedCycle_[SolverID] = cycleNum;
  crntSchedSlot_[SolverID] = slotNum;

  //if (GetNum() == 1) Logger::Info("just set sched cycle for inst 1 to %d", cycleNum);
}

bool SchedInstruction::IsInReadyList(int SolverID) const { return ready_[SolverID]; }

void SchedInstruction::PutInReadyList(int SolverID) { ready_[SolverID] = true; }

void SchedInstruction::RemoveFromReadyList(int SolverID) { ready_[SolverID] = false; }

InstCount SchedInstruction::GetCrntLwrBound(DIRECTION dir, int SolverID) const {
  return crntRange_[SolverID]->GetLwrBound(dir);
}

void SchedInstruction::SetCrntLwrBound(DIRECTION dir, InstCount bound, int SolverID) {
  crntRange_[SolverID]->SetLwrBound(dir, bound);
}

void SchedInstruction::UnSchedule(int SolverID) {
  //Logger::Info("solverid %d unscheduling inst %d", SolverID, GetNum());
  assert(crntSchedCycle_[SolverID] != SCHD_UNSCHDULD);
  crntSchedCycle_[SolverID] = SCHD_UNSCHDULD;
  crntSchedSlot_[SolverID] = SCHD_UNSCHDULD;

  //if (GetNum() == 1) Logger::Info("just unscheduled inst 1");
}

void SchedInstruction::UnTightnLwrBounds(int SolverID) { 
  crntRange_[SolverID]->UnTightnLwrBounds(); 
}

void SchedInstruction::CmtLwrBoundTightnng(int SolverID) {
  crntRange_[SolverID]->CmtLwrBoundTightnng();
}

void SchedInstruction::SetSig(InstSignature sig) { sig_ = sig; }

InstSignature SchedInstruction::GetSig() const { return sig_; }


InstCount SchedInstruction::GetFxdCycle(int SolverID) const {
  assert(crntRange_[SolverID]->IsFxd());
  return crntRange_[SolverID]->GetLwrBound(DIR_FRWRD);
}


bool SchedInstruction::IsFxd(int SolverID) const { return crntRange_[SolverID]->IsFxd(); }

InstCount SchedInstruction::GetPreFxdCycle() const { return preFxdCycle_; }

bool SchedInstruction::TightnLwrBound(DIRECTION dir, InstCount newLwrBound,
                                      LinkedList<SchedInstruction> *tightndLst,
                                      LinkedList<SchedInstruction> *fxdLst,
                                      bool enforce, int SolverID) {
  return crntRange_[SolverID]->TightnLwrBound(dir, newLwrBound, tightndLst, fxdLst,
                                    enforce, SolverID);
}

bool SchedInstruction::TightnLwrBoundRcrsvly(
    DIRECTION dir, InstCount newLwrBound,
    LinkedList<SchedInstruction> *tightndLst,
    LinkedList<SchedInstruction> *fxdLst, bool enforce,
    int SolverID) {
  return crntRange_[SolverID]->TightnLwrBoundRcrsvly(dir, newLwrBound, tightndLst, fxdLst,
                                           enforce, SolverID);
}

bool SchedInstruction::ProbeScsrsCrntLwrBounds(InstCount cycle, int SolverID) {
  if (cycle <= crntRange_[SolverID]->GetLwrBound(DIR_FRWRD))
    return false;

  for (GraphEdge *edg = GetFrstScsrEdge(SolverID); edg != NULL;
       edg = GetNxtScsrEdge(SolverID)) {
    UDT_GLABEL edgLbl = edg->label;
    SchedInstruction *nghbr = (SchedInstruction *)(edg->GetOtherNode(this));
    InstCount nghbrNewLwrBound = cycle + edgLbl;

    // If this neighbor will get delayed by scheduling this instruction in the
    // given cycle.
    if (nghbrNewLwrBound > nghbr->GetCrntLwrBound(DIR_FRWRD, SolverID))
      return true;
  }

  return false;
}

void SchedInstruction::ComputeAdjustedUseCnt_() {
  adjustedUseCnt_ =
      NumUses() - llvm::count_if(GetUses(), [](const Register *use) {
        return use->IsLiveOut();
      });
}

InstCount SchedInstruction::GetFileSchedOrder() const {
  return fileSchedOrder_;
}

InstCount SchedInstruction::GetFileSchedCycle() const {
  return fileSchedCycle_;
}

// Called via SetupForSchduling (Sched Region)
// Done for all threads simultaneously
// TODO -- do we need to loop?
void SchedInstruction::SetScsrNums_() {
  InstCount scsrNum = 0;

  for (GraphEdge *edge = GetFrstScsrEdge(0); edge != NULL;
       edge = GetNxtScsrEdge(0)) {
    edge->succOrder = scsrNum++;
  }


  assert(scsrNum == GetScsrCnt());
}

void SchedInstruction::SetPrdcsrNums_() {
  InstCount prdcsrNum = 0;

  for (GraphEdge *edge = GetFrstPrdcsrEdge(0); edge != NULL;
       edge = GetNxtPrdcsrEdge(0)) {
    edge->predOrder = prdcsrNum++;
  }


  if (prdcsrNum != GetPrdcsrCnt()) Logger::Info("end of parsing prdcsr list, prdNum %d numprds %d", prdcsrNum, GetPrdcsrCnt());
  assert(prdcsrNum == GetPrdcsrCnt());
}

int16_t SchedInstruction::CmputLastUseCnt(int SolverID) {
  lastUseCnt_[SolverID] = 0;

  for (int i = 0; i < useCnt_; i++) {
    Register *reg = uses_[i];
    if (reg) 
     assert(reg->GetCrntUseCnt(SolverID) < reg->GetUseCnt());
    
    
    if (reg->GetCrntUseCnt(SolverID) + 1 == reg->GetUseCnt())
      lastUseCnt_[SolverID]++;
  }

  return lastUseCnt_[SolverID];
}

/******************************************************************************
 * SchedRange                                                                 *
 ******************************************************************************/

SchedRange::SchedRange(SchedInstruction *inst) {
  InitVars_();
  inst_ = inst;
  frwrdLwrBound_ = INVALID_VALUE;
  bkwrdLwrBound_ = INVALID_VALUE;
  lastCycle_ = INVALID_VALUE;
}

void SchedRange::resetState() {
  frwrdLwrBound_ = INVALID_VALUE;
  bkwrdLwrBound_ = INVALID_VALUE;
  lastCycle_ = INVALID_VALUE;
}

bool SchedRange::TightnLwrBound(DIRECTION dir, InstCount newBound,
                                LinkedList<SchedInstruction> *tightndLst,
                                LinkedList<SchedInstruction> *fxdLst,
                                bool enforce, int SolverID) {
  InstCount *boundPtr = (dir == DIR_FRWRD) ? &frwrdLwrBound_ : &bkwrdLwrBound_;
  InstCount crntBound = *boundPtr;
  InstCount othrBound = (dir == DIR_FRWRD) ? bkwrdLwrBound_ : frwrdLwrBound_;

  assert(enforce || IsFsbl_());
  assert(newBound > crntBound);
  InstCount boundSum = newBound + othrBound;

  bool fsbl = true;
  if (boundSum > lastCycle_) {
    fsbl = false;
    if (!enforce)
      return false;
  }

  //Logger::Info("SolverID is %d", SolverID);
  //Logger::Info("!inst_->ISScheduld(SolverID) %d", !inst_->IsSchduld(SolverID));
  //Logger::Info("inst_->getNum() %d", inst_->GetNum());
  assert(enforce || !inst_->IsSchduld(SolverID));
  assert(enforce || !isFxd_);

  // If the range equals exactly one cycle.
  if (boundSum == lastCycle_) {
    isFxd_ = true;
    fxdLst->InsrtElmnt(inst_);
  }

  bool *isTightndPtr = (dir == DIR_FRWRD) ? &isFrwrdTightnd_ : &isBkwrdTightnd_;
  bool isTightnd = isFrwrdTightnd_ || isBkwrdTightnd_;
  InstCount *prevBoundPtr =
      (dir == DIR_FRWRD) ? &prevFrwrdLwrBound_ : &prevBkwrdLwrBound_;

  // If this instruction is not already in the tightened instruction list.
  if (!isTightnd || enforce) {
    // Add it to the list.
    tightndLst->InsrtElmnt(inst_);
  }

  // If this particular LB has not been tightened.
  if (!*isTightndPtr && !enforce) {
    *prevBoundPtr = crntBound;
    *isTightndPtr = true;
  }

  // Now change the bound to the new bound.
  *boundPtr = newBound;

  return fsbl;
}

bool SchedRange::TightnLwrBoundRcrsvly(DIRECTION dir, InstCount newBound,
                                       LinkedList<SchedInstruction> *tightndLst,
                                       LinkedList<SchedInstruction> *fxdLst,
                                       bool enforce, int SolverID) {                                 

  auto getNextNeighbor =
      dir == DIR_FRWRD
          ? +[](SchedRange &range, int SolverID) { return range.inst_->GetNxtScsrEdge(SolverID); }
          : +[](SchedRange &range, int SolverID) { return range.inst_->GetNxtPrdcsrEdge(SolverID); };

  InstCount crntBound = (dir == DIR_FRWRD) ? frwrdLwrBound_ : bkwrdLwrBound_;
  bool fsbl = IsFsbl_();

  assert(enforce || fsbl);
  assert(newBound >= crntBound);

  if (newBound > crntBound) {
    fsbl = TightnLwrBound(dir, newBound, tightndLst, fxdLst, enforce, SolverID);

    if (!fsbl && !enforce)
      return false;

    for (GraphEdge *edg = dir == DIR_FRWRD ? inst_->GetFrstScsrEdge(SolverID)
                                           : inst_->GetFrstPrdcsrEdge(SolverID);
         edg != NULL; edg = getNextNeighbor(*this, SolverID)) {
      UDT_GLABEL edgLbl = edg->label;
      SchedInstruction *nghbr = (SchedInstruction *)(edg->GetOtherNode(inst_));
      InstCount nghbrNewBound = newBound + edgLbl;

     if (nghbrNewBound > nghbr->GetCrntLwrBound(dir, SolverID)) {
       //if (SolverID == 2) {
       //   Logger::Log((Logger::LOG_LEVEL) 4, false, "need to tightn nghbr %d to LB %d (currently %d)",nghbr->GetNum(), nghbrNewBound, nghbr->GetCrntLwrBound(dir, SolverID));
       //}
        //Logger::Info("nghbr->GetNum() %d", nghbr->GetNum());
        //Logger::Info("nghbr->IsScheduld(SolverID) %d", nghbr->IsSchduld(SolverID));
        bool nghbrFsblty = nghbr->TightnLwrBoundRcrsvly(
            dir, nghbrNewBound, tightndLst, fxdLst, enforce, SolverID);
        if (!nghbrFsblty) {
          fsbl = false;
          if (!enforce)
            return false;
        }
      }
    }
  }

  assert(enforce || fsbl);
  return fsbl;
}

bool SchedRange::Fix(InstCount cycle, LinkedList<SchedInstruction> *tightndLst,
                     LinkedList<SchedInstruction> *fxdLst, int SolverID) {
  if (cycle < frwrdLwrBound_ || cycle > GetDeadline())
    return false;
  InstCount backBnd = lastCycle_ - cycle;
  return (TightnLwrBoundRcrsvly(DIR_FRWRD, cycle, tightndLst, fxdLst, false, SolverID) &&
          TightnLwrBoundRcrsvly(DIR_BKWRD, backBnd, tightndLst, fxdLst, false, SolverID));
}

void SchedRange::SetBounds(InstCount frwrdLwrBound, InstCount bkwrdLwrBound) {
  InitVars_();
  frwrdLwrBound_ = frwrdLwrBound;
  bkwrdLwrBound_ = bkwrdLwrBound;
}

bool SchedRange::SetBounds(InstCount frwrdLwrBound, InstCount bkwrdLwrBound,
                           InstCount schedLngth,
                           LinkedList<SchedInstruction> *fxdLst) {
  InitVars_();
  frwrdLwrBound_ = frwrdLwrBound;
  bkwrdLwrBound_ = bkwrdLwrBound;
  assert(schedLngth != INVALID_VALUE);
  lastCycle_ = schedLngth - 1;

  if (!IsFsbl_())
    return false;

  if (GetLwrBoundSum_() == lastCycle_) {
    isFxd_ = true;
    assert(fxdLst != NULL);
    fxdLst->InsrtElmnt(inst_);
  }

  return true;
}

void SchedRange::InitVars_() {
  prevFrwrdLwrBound_ = INVALID_VALUE;
  prevBkwrdLwrBound_ = INVALID_VALUE;
  isFrwrdTightnd_ = false;
  isBkwrdTightnd_ = false;
  isFxd_ = false;
}

void SchedRange::SetFrwrdBound(InstCount bound) {
  assert(bound >= frwrdLwrBound_);
  frwrdLwrBound_ = bound;
}

void SchedRange::SetBkwrdBound(InstCount bound) {
  assert(bound >= bkwrdLwrBound_);
  bkwrdLwrBound_ = bound;
}

InstCount SchedRange::GetLwrBoundSum_() const {
  return frwrdLwrBound_ + bkwrdLwrBound_;
}

InstCount SchedRange::GetDeadline() const {
  return lastCycle_ - bkwrdLwrBound_;
}

bool SchedRange::IsFsbl_() const { return GetLwrBoundSum_() <= lastCycle_; }

void SchedRange::UnTightnLwrBounds() {
  assert(IsFsbl_());
  assert(isFrwrdTightnd_ || isBkwrdTightnd_);

  if (isFrwrdTightnd_) {
    assert(frwrdLwrBound_ != prevFrwrdLwrBound_);
    frwrdLwrBound_ = prevFrwrdLwrBound_;
    isFrwrdTightnd_ = false;
  }

  if (isBkwrdTightnd_) {
    assert(bkwrdLwrBound_ != prevBkwrdLwrBound_);
    bkwrdLwrBound_ = prevBkwrdLwrBound_;
    isBkwrdTightnd_ = false;
  }

  if (isFxd_)
    isFxd_ = false;
}

void SchedRange::CmtLwrBoundTightnng() {
  assert(isFrwrdTightnd_ || isBkwrdTightnd_);
  isFrwrdTightnd_ = false;
  isBkwrdTightnd_ = false;
}

InstCount SchedRange::GetLwrBound(DIRECTION dir) const {
  return (dir == DIR_FRWRD) ? frwrdLwrBound_ : bkwrdLwrBound_;
}

bool SchedRange::IsFxd() const { return lastCycle_ == GetLwrBoundSum_(); }

void SchedRange::SetLwrBound(DIRECTION dir, InstCount bound) {
  InstCount &crntBound = (dir == DIR_FRWRD) ? frwrdLwrBound_ : bkwrdLwrBound_;
  bool &isTightnd = (dir == DIR_FRWRD) ? isFrwrdTightnd_ : isBkwrdTightnd_;

  if (isFxd_ && bound != crntBound) {
    assert(bound < crntBound);
    isFxd_ = false;
  }

  crntBound = bound;
#ifdef IS_DEBUG
  InstCount crntBoundPtr = (dir == DIR_FRWRD) ? frwrdLwrBound_ : bkwrdLwrBound_;
  assert(crntBoundPtr == bound);
#endif
  isTightnd = false;
}

bool SchedRange::IsTightnd(DIRECTION dir) const {
  return (dir == DIR_FRWRD) ? isFrwrdTightnd_ : isBkwrdTightnd_;
}
