#include "opt-sched/Scheduler/enumerator.h"
#include "opt-sched/Scheduler/bb_thread.h"
#include "opt-sched/Scheduler/hist_table.h"
#include "opt-sched/Scheduler/logger.h"
#include "opt-sched/Scheduler/random.h"
#include "opt-sched/Scheduler/stats.h"
#include "opt-sched/Scheduler/utilities.h"
#include <algorithm>
#include <iterator>
#include <memory>
#include <sstream>
#include <stack>

using namespace llvm::opt_sched;

class InstPool4;

int HalfNode::getAndRemoveNextPrefixInst() {
  int temp = prefix_.front();
  prefix_.pop();
  return temp;
}

HalfNode::HalfNode() {
  heuristic_ = nullptr;
}

HalfNode::HalfNode(std::queue<int> prefix, unsigned long *heuristic, InstCount cost) :
  prefix_ {prefix},
  heuristic_ {heuristic},
  cost_ {cost} {
  
}

HalfNode::~HalfNode() {
  if (heuristic_ != nullptr) {
    delete heuristic_;
  }
}




EnumTreeNode::EnumTreeNode() {
  isCnstrctd_ = false;
  isClean_ = true;
  rdyLst_ = NULL;
  diversityNum_ = INVALID_VALUE;
}
/*****************************************************************************/

EnumTreeNode::~EnumTreeNode() {
  assert(isCnstrctd_ || isClean_);
  assert(isCnstrctd_ || rdyLst_ == NULL);

  if (isCnstrctd_) {
      assert(frwrdLwrBounds_ != NULL);
      delete[] frwrdLwrBounds_;

      assert(exmndInsts_ != NULL);
      for (ExaminedInst *exmndInst = exmndInsts_->GetFrstElmnt();
          exmndInst != NULL; exmndInst = exmndInsts_->GetNxtElmnt()) {
        delete exmndInst;
      }
      exmndInsts_->Reset();
      delete exmndInsts_;

      assert(chldrn_ != NULL);
      delete chldrn_;


    if (rdyLst_ != NULL)
      delete rdyLst_;
    if (rsrvSlots_ != NULL)
      delete[] rsrvSlots_;
#ifdef INSTPREFIX    
    delete prefixInstNums_;
#endif
  } else {
    assert(isClean_);
  }
}
/*****************************************************************************/
void EnumTreeNode::Init_() {
  assert(isClean_);
  brnchCnt_ = 0;
  crntBrnchNum_ = 0;
  fsblBrnchCnt_ = 0;
  legalInstCnt_ = 0;
  hstry_ = NULL;
  rdyLst_ = NULL;
  dmntdNode_ = NULL;
  isArchivd_ = false;
  isFsbl_ = true;
  isLngthFsbl_ = true;
  lngthFsblBrnchCnt_ = 0;
  isLeaf_ = false;
  cost_ = INVALID_VALUE;
  costLwrBound_ = INVALID_VALUE;
  crntCycleBlkd_ = false;
  rsrvSlots_ = NULL;
  totalCostIsActualCost_ = false;
  totalCost_ = -1;
  suffix_.clear();
}
/*****************************************************************************/

void EnumTreeNode::Construct(EnumTreeNode *prevNode, SchedInstruction *inst,
                             Enumerator *enumrtr, bool fullNode, bool allocStructs,
                             InstCount instCnt) {
  
  if (isCnstrctd_) {
    if (isClean_ == false) {
      Clean();
    }
  }

  Init_();

  prevNode_ = prevNode;
  
  inst_ = inst;
  enumrtr_ = enumrtr;
  time_ = prevNode_ == NULL ? 0 : prevNode_->time_ + 1;

  InstCount instCnt_ = enumrtr_->totInstCnt_;

  assert(instCnt_ != INVALID_VALUE);
  
   if (isCnstrctd_ == false) {
    exmndInsts_ = new LinkedList<ExaminedInst>(instCnt_);
    chldrn_ = new LinkedList<HistEnumTreeNode>(instCnt_);
    frwrdLwrBounds_ = new InstCount[instCnt_];
  }

  if (enumrtr && fullNode) {
    if (enumrtr_->IsHistDom()) {
      CreateTmpHstry_();
    }
  }

  FormPrtilSchedSig_();

  dmntdNode_ = NULL;

  isCnstrctd_ = true;
  isClean_ = false;
  pushedToLocalPool_ = false;
  wasChildStolen_ = false;
}
/*****************************************************************************/

void EnumTreeNode::Reset() {
  assert(isCnstrctd_);

  if (rdyLst_ != NULL) {
    rdyLst_->Reset();
  }

  if (exmndInsts_ != NULL) {
    for (ExaminedInst *exmndInst = exmndInsts_->GetFrstElmnt();
         exmndInst != NULL; exmndInst = exmndInsts_->GetNxtElmnt()) {
      delete exmndInst;
    }
    exmndInsts_->Reset();
  }

  if (chldrn_ != NULL) {
    chldrn_->Reset();
  }

  suffix_.clear();
}
/*****************************************************************************/

void EnumTreeNode::Clean() {
  assert(isCnstrctd_);
  Reset();

  if (rdyLst_ != NULL) {
    delete rdyLst_;
    rdyLst_ = NULL;
  }

  if (rsrvSlots_ != NULL) {
    delete[] rsrvSlots_;
    rsrvSlots_ = NULL;
  }

  isClean_ = true;
}
/*****************************************************************************/

void EnumTreeNode::FormPrtilSchedSig_() {
  SchedInstruction *inst = inst_;
  EnumTreeNode *prevNode = prevNode_;

  if (prevNode != NULL) {
    prtilSchedSig_ = prevNode->GetSig();
  } else { // if this is the root node
    prtilSchedSig_ = 0;
  }

  if (inst != NULL) {
    InstSignature instSig = inst->GetSig();
    prtilSchedSig_ ^= instSig;
  }
}
/*****************************************************************************/

void EnumTreeNode::SetLwrBounds() { SetLwrBounds(DIR_FRWRD); }
/*****************************************************************************/

void EnumTreeNode::SetLwrBounds(DIRECTION dir) {
  assert(dir == DIR_FRWRD);
  InstCount *&nodeLwrBounds = frwrdLwrBounds_;
  assert(nodeLwrBounds != NULL);
  DataDepGraph *dataDepGraph = enumrtr_->dataDepGraph_;
  dataDepGraph->GetCrntLwrBounds(dir, nodeLwrBounds, enumrtr_->getSolverID());
}
/*****************************************************************************/

void EnumTreeNode::SetRsrvSlots(int16_t rsrvSlotCnt, ReserveSlot *rsrvSlots) {
  assert(rsrvSlots_ == NULL);
  rsrvSlots_ = NULL;

  if (rsrvSlotCnt == 0) { // If no unpipelined instrs are scheduled
    return;
  }

  int issuRate = enumrtr_->machMdl_->GetIssueRate();

  rsrvSlots_ = new ReserveSlot[issuRate];

  for (int i = 0; i < issuRate; i++) {
    rsrvSlots_[i].strtCycle = rsrvSlots[i].strtCycle;
    rsrvSlots_[i].endCycle = rsrvSlots[i].endCycle;
  }
}
/*****************************************************************************/

bool EnumTreeNode::DoesPartialSchedMatch(EnumTreeNode *othr) {
  EnumTreeNode *thisNode, *othrNode;

  if (othr->time_ != time_) {
    return false;
  }

  for (thisNode = this, othrNode = othr;
       thisNode->IsRoot() != true && othrNode->IsRoot() != true;
       thisNode = thisNode->GetParent(), othrNode = othrNode->GetParent()) {
    if (thisNode->GetInst() != othrNode->GetInst()) {
      return false;
    }
  }

  return true;
}
/****************************************************************************/

void EnumTreeNode::PrntPartialSched(std::ostream &out) {
  out << "\nPartial sched. at time " << time_ << ": ";

  for (EnumTreeNode *node = this; node->IsRoot() != true;
       node = node->GetParent()) {
    out << node->GetInstNum() << ", ";
  }
}
/*****************************************************************************/

void EnumTreeNode::NewBranchExmnd(SchedInstruction *inst, bool isLegal,
                                  bool isNodeDmntd, bool wasRlxInfsbl,
                                  bool isBrnchFsbl, DIRECTION dir,
                                  bool isLngthFsbl) {
  if (inst != NULL) {
    InstCount deadline = inst->GetCrntDeadline(enumrtr_->getSolverID());
    InstCount cycleNum = enumrtr_->GetCycleNumFrmTime_(time_ + 1);
    InstCount slotNum = enumrtr_->GetSlotNumFrmTime_(time_ + 1);

    if (dir == DIR_FRWRD && cycleNum == deadline &&
        slotNum == enumrtr_->issuRate_ - 1) {
      // If that was the last issue slot in the instruction's deadline
      // then this instruction has just missed its deadline
      // and we don't need to consider this tree node any further
      isFsbl_ = false;
    }

    if (isLegal) {
      legalInstCnt_++;

      if (enumrtr_->prune_.nodeSup) {
        if (!isNodeDmntd) {
          ExaminedInst *exmndInst;
          exmndInst =
              new ExaminedInst(inst, wasRlxInfsbl, enumrtr_->dirctTightndLst_);
          exmndInsts_->InsrtElmnt(exmndInst);
        }
      }
    }
  }

  if (isLngthFsbl == false) {
    lngthFsblBrnchCnt_--;

    if (lngthFsblBrnchCnt_ == 0) {
      isLngthFsbl_ = false;
    }
  }

  crntBrnchNum_++;

  if (isBrnchFsbl == false) {
    ChildInfsbl();
  }
}
/*****************************************************************************/

void EnumTreeNode::SetBranchCnt(InstCount rdyLstSize, bool isLeaf) {
  assert(isLeaf == false || rdyLstSize == 0);
  isLeaf_ = isLeaf;

  if (isLeaf_) {
    isLngthFsbl_ = true;
  }

  brnchCnt_ = rdyLstSize + 1;
  isEmpty_ = rdyLstSize == 0;

  if (isLeaf_) {
    brnchCnt_ = 0;
  }

  fsblBrnchCnt_ = brnchCnt_;
  lngthFsblBrnchCnt_ = brnchCnt_;
}
/*****************************************************************************/

bool EnumTreeNode::ChkInstRdndncy(SchedInstruction *, int) {
  // Since we are optimizing spill cost, different permutations of the
  // same set of instructions within a certain cycle may have different
  // spill costs
  return false;
}
/*****************************************************************************/

bool EnumTreeNode::IsNxtSlotStall() {
  if (IsNxtCycleNew_() == false) {
    // If a stall has been scheduled in the current cycle then all slots in
    // this cycle must be stalls
    if (inst_ == NULL && time_ > 0) {
      return true;
    }
  }
  return false;
}
/*****************************************************************************/

bool EnumTreeNode::WasSprirNodeExmnd(SchedInstruction *cnddtInst) {
  if (cnddtInst == NULL)
    return false;

  Entry<ExaminedInst> *srchPtr;
  exmndInsts_->GetFrstElmntInPtr(srchPtr);
  for (ExaminedInst *exmndInst = srchPtr->element; srchPtr != nullptr && exmndInst != NULL;
       srchPtr = srchPtr->GetNext()) {
    SchedInstruction *inst = exmndInst->GetInst();
    assert(inst != cnddtInst);

    if (inst->GetIssueType() == cnddtInst->GetIssueType() &&
        inst->BlocksCycle() == cnddtInst->BlocksCycle() &&
        inst->IsPipelined() == cnddtInst->IsPipelined()) {
      if (cnddtInst->IsScsrDmntd(inst, enumrtr_->getSolverID())) {
        return true;
      } else {
#ifdef IS_DEBUG
        assert(!cnddtInst->IsScsrEquvlnt(inst, enumrtr_->getSolverID()));
#ifdef IS_DEBUG_NODEDOM
        if (inst->IsScsrDmntd(cnddtInst)) {
          stats::negativeNodeDominationHits++;
        }
#endif
#endif
      }
    }
  }

  return false;
}
/*****************************************************************************/

bool EnumTreeNode::WasRsrcDmnntNodeExmnd(SchedInstruction *cnddtInst) {
  if (cnddtInst == NULL) {
    return false;
  }

  SchedInstruction *inst;
  ExaminedInst *exmndInst;

  for (exmndInst = exmndInsts_->GetFrstElmnt(); exmndInst != NULL;
       exmndInst = exmndInsts_->GetNxtElmnt()) {
    inst = exmndInst->GetInst();
    assert(inst != cnddtInst);

    if (inst->GetIssueType() == cnddtInst->GetIssueType()) {
      if (exmndInst->wasRlxInfsbl()) {
        if (exmndInst->IsRsrcDmntd(cnddtInst)) {
          return true;
        }
      }
    }
  }
  return false;
}
/*****************************************************************************/

bool EnumTreeNode::IsBranchDominated(SchedInstruction *cnddtInst) {
  // Check if the given instruction can be feasibly replaced by a previously
  // examined instruction, which was found to be infeasible, thus proving by
  // contradiction that the given instruction is infeasible for this slot
  ExaminedInst *exmndInst = exmndInsts_->GetFrstElmnt();
  if (exmndInst == NULL)
    return false;

  SchedInstruction *inst = exmndInst->GetInst();
  assert(inst->IsSchduld(enumrtr_->getSolverID()) == false);

  if (cnddtInst->GetIssueType() != inst->GetIssueType())
    return false;

  InstCount deadline = inst->GetCrntDeadline(enumrtr_->getSolverID());

  // If one of the successors of the given instruction will get delayed if
  // this instruction was replaced by the examined instruction
  // then the swapping won't be possible and the domination checking fails.
  if (cnddtInst->ProbeScsrsCrntLwrBounds(deadline, enumrtr_->getSolverID()))
    return false;

  return true;
}
/*****************************************************************************/

void EnumTreeNode::Archive() {
  assert(isArchivd_ == false);

  if (enumrtr_->IsCostEnum()) {
    hstry_->SetCostInfo(this, false, enumrtr_);
  }

  isArchivd_ = true;
}
/**************************************************************************/

ExaminedInst::ExaminedInst(SchedInstruction *inst,
                                         bool wasRlxInfsbl,
                                         LinkedList<SchedInstruction> *) {
  inst_ = inst;
  wasRlxInfsbl_ = wasRlxInfsbl;
  tightndScsrs_ = NULL;
}
/****************************************************************************/

ExaminedInst::~ExaminedInst() {
  if (tightndScsrs_ != NULL) {
    for (TightndInst *inst = tightndScsrs_->GetFrstElmnt(); inst != NULL;
         inst = tightndScsrs_->GetNxtElmnt()) {
      delete inst;
    }
    tightndScsrs_->Reset();
    delete tightndScsrs_;
  }
}
/****************************************************************************/
void EnumTreeNode::removeNextPriorityInst()
{
  SchedInstruction *inst = rdyLst_->GetNextPriorityInst();
  while (inst->GetNum() != GetInstNum()) 
    inst = rdyLst_->GetNextPriorityInst();
  //Logger::Info("Removing inst # %d from enumTreeNodeReadyList", inst->GetNum());
  rdyLst_->RemoveNextPriorityInst();
}

void EnumTreeNode::printRdyLst() 
{
  int rdyLstSize = rdyLst_->GetInstCnt();
  SchedInstruction *inst;
  Logger::Info("ReadyList contains: ");
  for (int i = 0; i < rdyLstSize; i++) {
    inst = rdyLst_->GetNextPriorityInst();
    Logger::Info("%d", inst->GetNum());
  }
  rdyLst_->ResetIterator();
}

/****************************************************************************/
EnumTreeNode *EnumTreeNode::getAndRemoveNextPrefixInst() {
  EnumTreeNode *temp = prefix_.front();
  prefix_.pop();
  return temp;
}
/****************************************************************************/
void EnumTreeNode::pushToPrefix(EnumTreeNode *inst) {
  prefix_.push(inst);
}
/****************************************************************************/

Enumerator::Enumerator(DataDepGraph *dataDepGraph, MachineModel *machMdl,
                       InstCount schedUprBound, int16_t sigHashSize,
                       SchedPriorities prirts, Pruning PruningStrategy,
                       bool SchedForRPOnly, bool enblStallEnum,
                       Milliseconds timeout, int SolverID, int NumSolvers, std::mutex *AllocatorLock,
                       int timeoutToMemblock,
                       bool isSecondPass, InstCount preFxdInstCnt, SchedInstruction *preFxdInsts[])
    : ConstrainedScheduler(dataDepGraph, machMdl, schedUprBound, SolverID) {

  //#ifndef IS_CORRECT_LOCALPOOL
  //  #define IS_CORRECT_LOCALPOOL
  //#endif

  //#ifndef IS_DEBUG_SEARCH_ORDER
  //  #define IS_DEBUG_SEARCH_ORDER
  //#endif

  //#ifndef DEBUG_GP_HISTORY
  //  #define DEBUG_GP_HISTORY
  //#endif

  //ifndef WORK_STEAL
  //  #define WORK_STEAL
  //#endif

  //#ifndef INSERT_ON_BACKTRACK
  //  #define INSERT_ON_BACKTRACK
  //#endif

  //#ifndef INSERT_ON_STEPFRWRD
  //  #define INSERT_ON_STEPFRWRD
  //#endif

  //#ifndef IS_DEBUG_METADATA
  //  #define IS_DEBUG_METADATA
  //#endif

  //#ifndef IS_SYNCH_ALLOC
  //  #define IS_SYNCH_ALLOC
  //#endif

  NumSolvers_ = NumSolvers;
  
  timeoutToMemblock_ = timeoutToMemblock;

  memAllocBlkSize_ = (int)timeout / timeoutToMemblock_;// TIMEOUT_TO_MEMBLOCK_RATIO;
  assert(preFxdInstCnt >= 0);

  if (memAllocBlkSize_ > MAX_MEMBLOCK_SIZE) {
    memAllocBlkSize_ = MAX_MEMBLOCK_SIZE;
  }

  if (memAllocBlkSize_ == 0) {
    memAllocBlkSize_ = 1;
  }

  isCnstrctd_ = false;

  IsSecondPass_ = isSecondPass;

  rdyLst_ = NULL;
  prirts_ = prirts;
  prune_ = PruningStrategy;
  SchedForRPOnly_ = SchedForRPOnly;
  enblStallEnum_ = enblStallEnum;

  isEarlySubProbDom_ = true;

  rlxdSchdulr_ = new RJ_RelaxedScheduler(dataDepGraph, machMdl,
                                         schedUprBound_ + SCHED_UB_EXTRA,
                                         DIR_FRWRD, RST_DYNMC, SolverID, INVALID_VALUE);

  for (int16_t i = 0; i < issuTypeCnt_; i++) {
    neededSlots_[i] = instCntPerIssuType_[i];
#ifdef IS_DEBUG_ISSUE_TYPES
    Logger::Info("#of insts. of type %d is %d", i, instCntPerIssuType_[i]);
#endif
  }

  dataDepGraph_->EnableBackTracking();

  maxNodeCnt_ = 0;
  createdNodeCnt_ = 0;
  exmndNodeCnt_ = 0;
  fxdInstCnt_ = 0;
  minUnschduldTplgclOrdr_ = 0;
  backTrackCnt_ = 0;
  fsblSchedCnt_ = 0;
  imprvmntCnt_ = 0;
  prevTrgtLngth_ = INVALID_VALUE;
  bbt_ = NULL;
  AllocatorLock_ = AllocatorLock;

  int16_t sigSize = 8 * sizeof(InstSignature) - 1;

  Milliseconds histTableInitTime = Utilities::GetProcessorTime();

  exmndSubProbs_ = NULL;

  // Dont bother constructing if its a worker
  if (IsHistDom() && SolverID <= 1) {
    exmndSubProbs_ =
        new BinHashTable<HistEnumTreeNode>(sigSize, sigHashSize, true, NumSolvers_);
  }

  histTableInitTime = Utilities::GetProcessorTime() - histTableInitTime;
  stats::historyTableInitializationTime.Record(histTableInitTime);

  tightndLst_ = NULL;
  bkwrdTightndLst_ = NULL;
  dirctTightndLst_ = NULL;
  fxdLst_ = NULL;

  //TODO -- why do we need to increase max size?
  tightndLst_ = new LinkedList<SchedInstruction>(totInstCnt_);
  fxdLst_ = new LinkedList<SchedInstruction>(totInstCnt_);
  dirctTightndLst_ = new LinkedList<SchedInstruction>(totInstCnt_);
  bkwrdTightndLst_ = new LinkedList<SchedInstruction>(totInstCnt_);
  tmpLwrBounds_ = new InstCount[totInstCnt_];

  // need to be the same for all workers
  if (SolverID <= 1) {
    SetInstSigs_();
  }
  iterNum_ = 0;
  preFxdInstCnt_ = preFxdInstCnt;
  preFxdInsts_ = preFxdInsts;

  isCnstrctd_ = true;

  SolverID_ = SolverID;


  //RootExmndInsts_ = new LinkedList<ExaminedInst>(totInstCnt_);
  //RootChldrn_ = new LinkedList<HistEnumTreeNode>(totInstCnt_);
  //RootFrwrdLwrBounds_ = new InstCount[totInstCnt_];

}
/****************************************************************************/

Enumerator::~Enumerator() {
  //Logger::Info("SolverID %d deleting enum", SolverID_);
  // double free if workers try to delete hist table -- refers to same object
  if (SolverID_ <= 1) {
    //Logger::Info("starting deletion of history table");
    delete exmndSubProbs_;
    //Logger::Info("deleted history table");
  }

  for (InstCount i = 0; i < schedUprBound_; i++) {
    if (frstRdyLstPerCycle_[i] != NULL) {
      delete frstRdyLstPerCycle_[i];
      frstRdyLstPerCycle_[i] = NULL;
    }
  }

  
  delete tightndLst_;
  delete dirctTightndLst_;
  delete fxdLst_;
  delete bkwrdTightndLst_;
  delete[] tmpLwrBounds_;
  tmpHstryNode_->Clean();
  delete tmpHstryNode_;

  //Logger::Info("SolverID %d finished deleting enum", SolverID_);
}
/****************************************************************************/

void Enumerator::SetupAllocators_() {
  int memAllocBlkSize = memAllocBlkSize_;
  int lastInstsEntryCnt = issuRate_ * (dataDepGraph_->GetMaxLtncy());
  
  int maxNodeCnt = issuRate_ * schedUprBound_ + 1;
  int additionalNodes = bbt_->isWorker() ? bbt_->getLocalPoolMaxSize(SolverID_ - 2) * 4 : 0;
  maxNodeCnt += additionalNodes;
  //int multiplier = bbt_->isWorker() ? 15 : 1;
  //maxNodeCnt *= multiplier;
  int maxSize = INVALID_VALUE;

  //maxNodeCnt = (SolverID_ = 1) ? maxNodeCnt * 15 : maxNodeCnt;
  nodeAlctr_ = new EnumTreeNodeAlloc(maxNodeCnt, maxSize);

  if (IsHistDom()) {
    hashTblEntryAlctr_ =
        new MemAlloc<BinHashTblEntry<HistEnumTreeNode>>(memAllocBlkSize);

    bitVctr1_ = new BitVector(totInstCnt_);
    bitVctr2_ = new BitVector(totInstCnt_);

    lastInsts_ = new SchedInstruction *[lastInstsEntryCnt];
    othrLastInsts_ = new SchedInstruction *[totInstCnt_];
  }

  alctrsFreed_ = false;
}
/****************************************************************************/

void Enumerator::ResetAllocators_() {

  if (SolverID_ <= 1 || IsSecondPass_) {
    //Logger::Info("resetting allocator");
    if (IsHistDom()) {
      hashTblEntryAlctr_->Reset();
    }
    nodeAlctr_->Reset();
  }
}

/****************************************************************************/

void Enumerator::FreeAllocators_(){
  // For master thread, EnumTreeNodes will be deleted out of scope of this enumerator
  // Therefore only delete for workers (SolverID > 1) or nonparallel (isSecondPass)
  /*if (SolverID_ > 1 || IsSecondPass_) {
    delete nodeAlctr_;
    //Logger::Info("SolverID %d just deleted nodeAlctr", SolverID_);
  }*/

  if (!alctrsFreed_) {
    //Logger::Info("SolverID %d freeing enum::alctr", SolverID_);
    if (nodeAlctr_ != NULL)
      delete nodeAlctr_;
    //Logger::Info("SolverID %d just deleted nodeAlctr", SolverID_);
    nodeAlctr_ = NULL;
    if (rlxdSchdulr_ != NULL)
      delete rlxdSchdulr_;
    rlxdSchdulr_ = NULL;

    //Logger::Info("SolverID %d freeing enum::histAlctr", SolverID_);
    if (IsHistDom()) {
      if (hashTblEntryAlctr_ != NULL)
        delete hashTblEntryAlctr_;
      hashTblEntryAlctr_ = NULL;
      if (bitVctr1_ != NULL)
        delete bitVctr1_;
      if (bitVctr2_ != NULL)
        delete bitVctr2_;
      if (lastInsts_ != NULL)
        delete[] lastInsts_;
      if (othrLastInsts_ != NULL)
        delete[] othrLastInsts_;

      bitVctr1_ = NULL;
      bitVctr2_ = NULL;
      lastInsts_ = NULL;
      othrLastInsts_ = NULL;
    }
    //Logger::Info("finished freeing enum::histAcltr");

    alctrsFreed_ = true;
  }
}

/****************************************************************************/

void Enumerator::deleteNodeAlctr() {
  delete nodeAlctr_;
}
/****************************************************************************/

void Enumerator::freeEnumTreeNode(EnumTreeNode *node) {
  nodeAlctr_->Free(node);
}

void Enumerator::Reset() {
  schduldInstCnt_ = 0;
  crntSlotNum_ = 0;
  crntRealSlotNum_ = 0;
  crntCycleNum_ = 0;
  exmndNodeCnt_ = 0;

  if (IsHistDom() && SolverID_ <= 1) {
    Logger::Info("resetting hist table");
    exmndSubProbs_->Clear(false, hashTblEntryAlctr_);
  }

  ResetAllocators_();

  for (InstCount i = 0; i < schedUprBound_; i++) {
    if (frstRdyLstPerCycle_[i] != NULL) {
      frstRdyLstPerCycle_[i]->Reset();
    }
  }

  // potentially reset all the DDG data -- why isn't it doing it by itself?

  fxdLst_->Reset();
  tightndLst_->Reset();
  dirctTightndLst_->Reset();
  bkwrdTightndLst_->Reset();
  dataDepGraph_->SetSttcLwrBounds(SolverID_);
}
/****************************************************************************/

void Enumerator::resetEnumHistoryState()
{
  assert(IsHistDom());

  int lastInstsEntryCnt = issuRate_ * (dataDepGraph_->GetMaxLtncy());

  //delete bitVctr1_;
  //delete bitVctr2_;

  //bitVctr1_ = new BitVector(totInstCnt_);
  //bitVctr2_ = new BitVector(totInstCnt_);  
  
  delete[] lastInsts_;
  delete[] othrLastInsts_;

  lastInsts_ = new SchedInstruction *[lastInstsEntryCnt];
  othrLastInsts_ = new SchedInstruction *[totInstCnt_];
}
/****************************************************************************/

bool Enumerator::Initialize_(InstSchedule *sched, InstCount trgtLngth, int SolverID,
                             bool ScheduleRoot) {
  assert(trgtLngth <= schedUprBound_);
  assert(fxdLst_);
  trgtSchedLngth_ = trgtLngth;
  fsblSchedCnt_ = 0;
  imprvmntCnt_ = 0;
  crntSched_ = sched;
  minUnschduldTplgclOrdr_ = 0;
  backTrackCnt_ = 0;
  iterNum_++;
  SolverID_ = SolverID;


  if (ConstrainedScheduler::Initialize_(trgtSchedLngth_, fxdLst_) == false) {
    return false;
  }

  rlxdSchdulr_->Initialize(false);

  if (preFxdInstCnt_ > 0) {
    if (InitPreFxdInsts_() == false) {
      return false;
    }

    dataDepGraph_->SetDynmcLwrBounds(SolverID);
  }

  if (FixInsts_(NULL) == false) {
    return false;
  }

  // For each issue slot the total number of options is equal to the total
  // number of instructions plus the option of scheduling a stall
  // This establishes an upper bound on the number of tree nodes
  // InstCount maxSlotCnt = trgtSchedLngth_ * issuRate_;
  exmndNodeCnt_ = 0;
  maxNodeCnt_ = 0;

  int i;

  for (i = 0; i < issuTypeCnt_; i++) {
    avlblSlots_[i] = slotsPerTypePerCycle_[i] * trgtSchedLngth_;

    if (avlblSlots_[i] < neededSlots_[i]) {
#ifdef IS_DEBUG_FLOW
      Logger::Info("Length %d is infeasible; %d slots of type %d are needed.",
                   trgtLngth, neededSlots_[i], i);
#endif
      return false;
    }
  }

  rlxdSchdulr_->SetupPrirtyLst();

  createdNodeCnt_ = 0;
  fxdInstCnt_ = 0;
  rdyLst_ = NULL;
  
  /*
  if (!bbt_->isWorker())
  {
    CreateRootNode_();
    crntNode_ = rootNode_;
  }
  else
  {
    createWorkerRootNode_();
  }*/
  CreateRootNode_();
  crntNode_ = rootNode_;

  ClearState_();
  return true;
}
/*****************************************************************************/

bool Enumerator::InitPreFxdInsts_() {
  for (InstCount i = 0; i < preFxdInstCnt_; i++) {
    bool fsbl = preFxdInsts_[i]->ApplyPreFxng(tightndLst_, fxdLst_, SolverID_);
    if (!fsbl)
      return false;
  }
  return true;
}
/*****************************************************************************/

void Enumerator::SetInstSigs_() {
  InstCount i;
  int16_t bitsForInstNum = Utilities::clcltBitsNeededToHoldNum(totInstCnt_ - 1);
  Logger::Info("solverID %d setting inst sigs", SolverID_);

  for (i = 0; i < totInstCnt_; i++) {
    SchedInstruction *inst = dataDepGraph_->GetInstByIndx(i);
    //InstSignature sig = RandomGen::GetRand32();
    InstSignature sig = inst->GetNum() * 12345678;

    // ensure it is not zero
    if (sig == 0) {
      sig += 1;
    }

    // left shift by the number of bits needed to encode the instruction number
    sig <<= bitsForInstNum;

    // now, place the instruction number in the least significant bits
    sig |= i;

    //    sig &= 0x7fffffffffffffff;
    sig &= 0x7fffffff;

    assert(sig != 0);

    inst->SetSig(sig);
  }
}


SchedInstruction *Enumerator::GetInstByIndx(InstCount index) {
  return dataDepGraph_->GetInstByIndx(index);
}


/*****************************************************************************/

void Enumerator::CreateRootNode_() {
  //Logger::Info("creating enum root");
#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorLock();
#endif
  rootNode_ = nodeAlctr_->Alloc(NULL, NULL, this);
#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorUnlock();
#endif
  
  CreateNewRdyLst_();
  rootNode_->SetRdyLst(rdyLst_);
  rootNode_->SetLwrBounds(DIR_FRWRD);
  assert(rsrvSlotCnt_ == 0);
  rootNode_->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);
  InitNewNode_(rootNode_);
  CmtLwrBoundTightnng_();
}
/*****************************************************************************/

void Enumerator::setHistTable(BinHashTable<HistEnumTreeNode> *exmndSubProbs) {
  assert(IsHistDom());

  //if (exmndSubProbs_)
  //  delete exmndSubProbs_;

  exmndSubProbs_ = exmndSubProbs;
}
// virtual in Enumerator class
/*
void Enumerator::createWorkerRootNode_() {
  crntNode_ = nodeAlctr_->Alloc(NULL, NULL, this);
  CreateNewRdyLst_();
  crntNode_->SetRdyLst(rdyLst_);
  crntNode_->SetLwrBounds(DIR_FRWRD);

  assert(rsrvSlotCnt_ == 0);
  crntNode_->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);

  bbt_->setSttcLwrBounds(crntNode_);

  // InitNewNode_(rootNode_) innards

  crntNode_->SetCrntCycleBlkd(isCrntCycleBlkd_);
  crntNode_->SetRealSlotNum(crntRealSlotNum_);

  if (IsHistDom()) {
    crntNode_->CreateHistory();
    assert(crntNode_->GetHistory() != tmpHstryNode_);
  }

  crntNode_->SetSlotAvlblty(avlblSlots_, avlblSlotsInCrntCycle_);

  // not sure if we need this
  // UpdtRdyLst_(crntCycleNum_, crntSlotNum_) innards
  //if (prirts_.isDynmc)
  //  rdyLst_->UpdatePriorities();


  bool isLeaf = schduldInstCnt_ == totInstCnt_;

  crntNode_->SetBranchCnt(rdyLst_->GetInstCnt(), isLeaf);

  createdNodeCnt_++;
  crntNode_->SetNum(createdNodeCnt_);



  CmtLwrBoundTightnng_();
}
*/

/*****************************************************************************/


namespace {

// TODO: Add unit tests to replace this style of debug code.
#if defined(IS_DEBUG_SUFFIX_SCHED)

void CheckHistNodeMatches(EnumTreeNode *const node,
                          HistEnumTreeNode *const histNode,
                          const char *loc = "CheckHistNodeMatches") {
  auto histPrefix = histNode->GetPrefix();
  auto currPrefix = [&]() {
    std::vector<InstCount> prefix;
    for (auto n = node; n != nullptr; n = n->GetParent()) {
      if (n->GetInstNum() != SCHD_STALL)
        prefix.push_back(n->GetInstNum());
    }
    return prefix;
  }();
  std::sort(histPrefix.begin(), histPrefix.end());
  std::sort(currPrefix.begin(), currPrefix.end());
  if (histPrefix.size() != currPrefix.size()) {
    printVector(histPrefix, "HistPrefix");
    printVector(currPrefix, "CurrPrefix");
    Logger::Fatal(
        "%s: Hist prefix size %llu doesn't match current prefix %llu!", loc,
        histPrefix.size(), currPrefix.size());
  }
  if (histPrefix != currPrefix) {
    printVector(histPrefix, "HistPrefix");
    printVector(currPrefix, "CurrPrefix");
    Logger::Fatal("%s: Hist prefix and current prefix are not permutations of "
                  "each other!",
                  loc, histPrefix.size(), currPrefix.size());
  }
}

void PrintSchedule(InstSchedule *const sched,
                   Logger::LOG_LEVEL level = Logger::INFO) {
  InstCount cycle, slot;
  std::stringstream s;
  for (auto inst = sched->GetFrstInst(cycle, slot); inst != INVALID_VALUE;
       inst = sched->GetNxtInst(cycle, slot)) {
    s << inst << ' ';
  }
  Logger::Log(level, false, "Schedule: %s", s.str().c_str());
}

#endif // IS_DEBUG_SUFFIX_SCHED

void AppendAndCheckSuffixSchedules(
    HistEnumTreeNode *const matchingHistNodeWithSuffix, BBThread *const bbt_,
    InstSchedule *const crntSched_, InstCount trgtSchedLngth_,
    LengthCostEnumerator *const thisAsLengthCostEnum,
    EnumTreeNode *const crntNode_, DataDepGraph *const dataDepGraph_) {

  Logger::Info("in appendandcheck");

  assert(matchingHistNodeWithSuffix != nullptr && "Hist node is null");
  assert(matchingHistNodeWithSuffix->GetSuffix() != nullptr &&
         "Hist node suffix is null");
  assert(matchingHistNodeWithSuffix->GetSuffix()->size() > 0 &&
         "Hist node suffix size is zero");
  // For each matching history node, concatenate the suffix with the
  // current schedule and check to see if it's better than the best
  // schedule found so far.
  auto concatSched = std::unique_ptr<InstSchedule>(bbt_->allocNewSched_());
  // Get the prefix.
  concatSched->Copy(crntSched_);

#if defined(IS_DEBUG_SUFFIX_SCHED)
  {
    auto prefix = matchingHistNodeWithSuffix->GetPrefix();
    if (prefix.size() != crntSched_->GetCrntLngth()) {
      PrintSchedule(crntSched_, Logger::ERROR);
      std::stringstream s;
      for (auto j : prefix)
        s << j << ' ';
      Logger::Error("Prefix: %s", s.str().c_str());
      s.str("");
      for (auto j : *matchingHistNodeWithSuffix->GetSuffix())
        s << (j == nullptr ? SCHD_STALL : j->GetNum()) << ' ';
      Logger::Error("SUffix: %s", s.str().c_str());
      Logger::Fatal(
          "Hist node prefix size %llu doesn't match current sched length %d!",
          prefix.size(), crntSched_->GetCrntLngth());
    }
  }
#endif

  // Concatenate the suffix.
  for (auto inst : *matchingHistNodeWithSuffix->GetSuffix())
    concatSched->AppendInst((inst == nullptr) ? SCHD_STALL : inst->GetNum());

    // Update and check.

#if defined(IS_DEBUG_SUFFIX_SCHED)
  if (concatSched->GetCrntLngth() != trgtSchedLngth_) {
    PrintSchedule(concatSched.get(), Logger::ERROR);
    PrintSchedule(crntSched_, Logger::ERROR);
    std::stringstream s;
    auto prefix = matchingHistNodeWithSuffix->GetPrefix();
    for (auto j : prefix)
      s << j << ' ';
    Logger::Error("Prefix: %s", s.str().c_str());
    s.str("");
    for (auto j : *matchingHistNodeWithSuffix->GetSuffix())
      s << (j == nullptr ? SCHD_STALL : j->GetNum()) << ' ';
    Logger::Error("SUffix: %s", s.str().c_str());
    Logger::Fatal("Suffix Scheduling: Concatenated schedule length %d "
                  "does not meet target length %d!",
                  concatSched->GetCrntLngth(), trgtSchedLngth_);
  }
#endif
  auto oldCost = thisAsLengthCostEnum->GetBestCost();
  auto newCost = bbt_->UpdtOptmlSched(concatSched.get(), thisAsLengthCostEnum);
#if defined(IS_DEBUG_SUFFIX_SCHED)
  Logger::Info("Found a concatenated schedule with node instruction %d",
               crntNode_->GetInstNum());
#endif
  if (newCost < oldCost) {
    Logger::Info("in append and check suffix sched, updating node costs");

#if defined(IS_DEBUG_SUFFIX_SCHED)
    Logger::Info("Suffix Scheduling: Concatenated schedule has better "
                 "cost %d than best schedule %d!",
                 newCost, oldCost);
#endif
    // Don't forget to update the total cost and suffix for this node,
    // because we intentionally backtrack without visiting its
    // children.
    crntNode_->SetTotalCost(newCost);
    crntNode_->SetTotalCostIsActualCost(true);
    if (newCost == 0) {
      Logger::Info(
          "Suffix Scheduling: ***GOOD*** Schedule of cost 0 was found!");
    }
  } else {
#if defined(IS_DEBUG_SUFFIX_SCHED)
    Logger::Info("Suffix scheduling: Concatenated schedule does not have "
                 "better cost %d than best schedule %d.",
                 newCost, oldCost);
#endif
  }

  // Before backtracking, reset the SchedRegion state to where it was before
  // concatenation.
  bbt_->InitForSchdulngBBThread();
  InstCount cycleNum, slotNum;
  for (auto instNum = crntSched_->GetFrstInst(cycleNum, slotNum);
       instNum != INVALID_VALUE;
       instNum = crntSched_->GetNxtInst(cycleNum, slotNum)) {
    bbt_->SchdulInstBBThread(dataDepGraph_->GetInstByIndx(instNum), cycleNum, slotNum,
                     false);
  }
}
} // namespace

FUNC_RESULT Enumerator::FindFeasibleSchedule_(InstSchedule *sched,
                                              InstCount trgtLngth,
                                              Milliseconds deadline) {

  EnumTreeNode *nxtNode = NULL;
  bool allNodesExplrd = false;
  bool foundFsblBrnch = false;
  bool isCrntNodeFsbl = true;
  bool isTimeout = false;

  if (!isCnstrctd_)
    return RES_ERROR;

  assert(trgtLngth <= schedUprBound_);

  // workers initialize the enumerator before calling FindFeasibleSched
  if (!bbt_->isWorker()) {
    if (Initialize_(sched, trgtLngth) == false) 
      return RES_FAIL;
    Logger::Info("worker id %d has root isnt %d", SolverID_, rootNode_->GetInstNum());
  }

#ifdef IS_DEBUG_NODES
  uint64_t prevNodeCnt = exmndNodeCnt_;
#endif

  while (!(allNodesExplrd || WasObjctvMet_())) {
    if (deadline != INVALID_VALUE && Utilities::GetProcessorTime() > deadline) {
      isTimeout = true;
      //Logger::Info("timed out");
      break;
    }

    mostRecentMatchingHistNode_ = nullptr;

    if (isCrntNodeFsbl) {
      #ifdef IS_DEBUG_METADATA
      Milliseconds startTime = Utilities::GetProcessorTime();
      #endif
      foundFsblBrnch = FindNxtFsblBrnch_(nxtNode);
      
      #ifdef IS_DEBUG_METADATA
      findBranchTime += Utilities::GetProcessorTime() - startTime;
      #endif
    } else {
      foundFsblBrnch = false;
    }

    if (foundFsblBrnch) {
      // (Chris): It's possible that the node we just determined to be feasible
      // dominates a history node with a suffix schedule. If this is the case,
      // then instead of continuing the search, we should generate schedules by
      // concatenating the best known suffix.
      #ifdef IS_DEBUG_METADATA
      Milliseconds startTime = Utilities::GetProcessorTime();
      #endif

      StepFrwrd_(nxtNode);

      #ifdef IS_DEBUG_METADATA
      moveForwardTime += Utilities::GetProcessorTime() - startTime;
      #endif

      // Find matching history nodes with suffixes.
      auto matchingHistNodesWithSuffix = mostRecentMatchingHistNode_;

      // If there are no such matches, continue the search. Else,
      // generate concatenated schedules.
      if (!IsHistDom() || matchingHistNodesWithSuffix == nullptr) {
        // If a branch from the current node that leads to a feasible node has
        // been found, move on down the tree to that feasible node.
        isCrntNodeFsbl = true;
      } else {
        assert(this->IsCostEnum() && "Not a LengthCostEnum instance!");
        crntNode_->GetHistory()->SetSuffix(
            matchingHistNodesWithSuffix->GetSuffix());
        AppendAndCheckSuffixSchedules(matchingHistNodesWithSuffix, bbt_,
                                      crntSched_, trgtSchedLngth_,
                                      static_cast<LengthCostEnumerator *>(this),
                                      crntNode_, dataDepGraph_);
        isCrntNodeFsbl = BackTrack_();
      }

      
    } else {
      // All branches from the current node have been explored, and no more
      // branches that lead to feasible nodes have been found.
      //Logger::Info("SolverID %d going to bracktrack", SolverID_);
      if (crntNode_ == rootNode_) {
        if (bbt_->isWorker()) BackTrackRoot_();
        allNodesExplrd = true;
      } else {
        isCrntNodeFsbl = BackTrack_();
      }
    }

#ifdef IS_DEBUG_FLOW
    crntNode_->PrntPartialSched(Logger::GetLogStream());
#endif
#ifdef IS_DEBUG
// Logger::PeriodicLog();
#endif
  }

#ifdef IS_DEBUG_NODES
  uint64_t crntNodeCnt = exmndNodeCnt_ - prevNodeCnt;
  stats::nodesPerLength.Record(crntNodeCnt);
#endif

  if (isTimeout) {
    //Logger::Info("SolverID %d returning timeout from inside enum", SolverID_);
    return RES_TIMEOUT;
  }
  // Logger::Info("\nEnumeration at length %d done\n", trgtLngth);
  return fsblSchedCnt_ > 0 ? RES_SUCCESS : RES_FAIL;
}
/****************************************************************************/

bool Enumerator::FindNxtFsblBrnch_(EnumTreeNode *&newNode) {
  assert(crntNode_);
  InstCount i;
  bool isEmptyNode;
  SchedInstruction *inst;
  InstCount brnchCnt = crntNode_->GetBranchCnt(isEmptyNode);
  InstCount crntBrnchNum = crntNode_->GetCrntBranchNum();
  bool isNodeDmntd, isRlxInfsbl;
  bool enumStall = false;
  bool isLngthFsbl = true;


#if defined(IS_DEBUG) || defined(IS_DEBUG_READY_LIST)
  InstCount rdyInstCnt = rdyLst_->GetInstCnt();;
  assert(crntNode_->IsLeaf() || (brnchCnt != rdyInstCnt) ? 1 : rdyInstCnt);
#endif

#ifdef IS_DEBUG_READY_LIST
  Logger::Info("Ready List Size is %d", rdyInstCnt);
  // Warning! That will reset the instruction iterator!
  // rdyLst_->Print(Logger::GetLogStream());

  stats::maxReadyListSize.SetMax(rdyInstCnt);
#endif

  if (crntBrnchNum == 0 && SchedForRPOnly_)
    crntNode_->SetFoundInstWithUse(IsUseInRdyLst_());

  for (i = crntBrnchNum; i < brnchCnt && crntNode_->IsFeasible(); i++) {
#ifdef IS_CORRECT_LOCALPOOL
    //Logger::Info("probing branch %d out of %d", i, brnchCnt);
#endif
#ifdef IS_DEBUG_FLOW
    Logger::Info("SolverID %d Probing branch %d out of %d", SolverID_, i, brnchCnt);
#endif

    if (i == brnchCnt - 1) {
      assert(i == rdyLst_->GetInstCnt());
      //Logger::Info("SolverID %d no more branches", SolverID_);
      if (!bbt_->isSecondPass()) {
        //Logger::Info("SolverID %d out of insts", SolverID_);
        return false;
      }
#ifdef IS_DEBUG_SEARCH_ORDER
      //Logger::Log((Logger::LOG_LEVEL) 4, false, "Out of instructions, stalling");
#endif
      // then we only have the option of scheduling a stall
      assert(isEmptyNode == false || brnchCnt == 1);
      inst = NULL;
      enumStall = EnumStall_();

      if (isEmptyNode || crntNode_->GetLegalInstCnt() == 0 || enumStall) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
        stats::stalls++;
#endif
      } else {
        crntNode_->NewBranchExmnd(inst, false, false, false, false, DIR_FRWRD,
                                  false);
        continue;
      }
    } else {
      //Logger::Info("SolverID %d attempting to probe next inst", SolverID_);
      assert(rdyLst_);

if (bbt_->isWorkStealOn()) {
    if (!bbt_->isWorker()) {
      inst = rdyLst_->GetNextPriorityInst();
    }

    if (bbt_->isWorker()) {
      if (crntNode_->getPushedToLocalPool()) {
        bbt_->localPoolLock(SolverID_ - 2);
        inst = rdyLst_->GetNextPriorityInst();
        if (!inst) {
          bbt_->localPoolUnlock(SolverID_ - 2);
          return false;
        }
        if (crntNode_->wasInstStolen(inst)) {
          //Logger::Info("inst %d (parent inst %d) was stolen from solver %d", inst->GetNum(), crntNode_->GetInstNum(), SolverID_);
          bbt_->localPoolUnlock(SolverID_ - 2);
          continue;
        }


        EnumTreeNode *removed = nullptr;
        bbt_->localPoolRemoveSpecificElement(SolverID_ - 2, inst, crntNode_, removed);
        bbt_->localPoolUnlock(SolverID_ - 2);
        if (removed != nullptr) {
#ifdef IS_CORRECT_LOCALPOOL
          Logger::Info("SolverID %d removed element from pool at depth %d", SolverID_, removed->GetTime());
#endif
          nodeAlctr_->Free(removed);
        }
        /*else {
#ifdef IS_CORRECT_LOCALPOOL
        Logger::Info("SolverID %dfailed to find a matching node in rdyLst_", SolverID_);
#endif
        }*/
      }

      else { //we haven't pushed this nodes rdylst to local pool
        //if (crntNode_ && bbt_->getLocalPoolSize(SolverID_ - 2) > 0)
        //  Logger::Info("havent pushed this node rdyLst to pool (node time %d, localPool time %d", crntNode_->GetTime(), bbt_->viewLocalPoolFront(SolverID_-2)->GetTime());
        inst = rdyLst_->GetNextPriorityInst();
        if (!inst) {
          return false;
        }
      }
    }
}

else {
      inst = rdyLst_->GetNextPriorityInst();
}

      //inst = rdyLst_->GetNextPriorityInst();

#ifdef IS_DEBUG_SEARCH_ORDER
        Logger::Log((Logger::LOG_LEVEL) 4, false, "SolverID %d Probing inst %d", SolverID_, inst->GetNum());
#endif
      assert(inst != NULL);
      bool isLegal = ChkInstLglty_(inst);
      isLngthFsbl = isLegal;

      if (isLegal == false || crntNode_->ChkInstRdndncy(inst, i)) {
#ifdef IS_DEBUG_FLOW
        Logger::Info("Inst %d is illegal or redundant in cyc%d/slt%d",
                     inst->GetNum(), crntCycleNum_, crntSlotNum_);
#endif
        exmndNodeCnt_++;
        crntNode_->NewBranchExmnd(inst, false, false, false, false, DIR_FRWRD,
                                  isLngthFsbl);
        continue;
      }
    }

    exmndNodeCnt_++;

#ifdef IS_DEBUG_INFSBLTY_TESTS
    stats::feasibilityTests++;
#endif
    isNodeDmntd = isRlxInfsbl = false;
    isLngthFsbl = true;

    

    if (ProbeBranch_(inst, newNode, isNodeDmntd, isRlxInfsbl, isLngthFsbl)) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
      stats::feasibilityHits++;
#endif
      return true;
    } else {
      RestoreCrntState_(inst, newNode);
      crntNode_->NewBranchExmnd(inst, true, isNodeDmntd, isRlxInfsbl, false,
                                DIR_FRWRD, isLngthFsbl);
    }

    
  }
  return false; // No feasible branch has been found at the current node
}
/*****************************************************************************/

bool Enumerator::ProbeBranch_(SchedInstruction *inst, EnumTreeNode *&newNode,
                              bool &isNodeDmntd, bool &isRlxInfsbl,
                              bool &isLngthFsbl) {

  bool fsbl = true;
  newNode = NULL;
  isLngthFsbl = false;

  assert(IsStateClear_());
  assert(inst == NULL || inst->IsSchduld(SolverID_) == false);

#ifdef IS_DEBUG_FLOW
  InstCount instNum = inst == NULL ? -2 : inst->GetNum();
  Logger::Info("Probing inst %d in cycle %d / slot %d", instNum, crntCycleNum_,
               crntSlotNum_);
#endif


  //startTime = Utilities::GetProcessorTime();
  // If this instruction is prefixed, it cannot be scheduled earlier than its
  // prefixed cycle
  if (inst != NULL)
    if (inst->GetPreFxdCycle() != INVALID_VALUE)
      if (inst->GetPreFxdCycle() != crntCycleNum_) {
#ifdef IS_DEBUG_SEARCH_ORDER
        Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: prefix fail");
#endif
        //endTime = Utilities::GetProcessorTime();
        //prefixTime += endTime - startTime;
        return false;
      }
  //endTime = Utilities::GetProcessorTime();
  //prefixTime += endTime - startTime;


  //startTime = Utilities::GetProcessorTime();
  if (inst != NULL) {
    if (inst->GetCrntLwrBound(DIR_FRWRD, SolverID_) > crntCycleNum_) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
      stats::forwardLBInfeasibilityHits++;
#endif
      frwrdLBInfsbl++;
#ifdef IS_DEBUG_SEARCH_ORDER
      Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: LB fail");
#endif
      //endTime = Utilities::GetProcessorTime();
      //lbTime += endTime - startTime;
      return false;
    }
    if (inst->GetCrntDeadline(SolverID_) < crntCycleNum_) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
      stats::backwardLBInfeasibilityHits++;
#endif
      bkwrdLBInfsbl++;

#ifdef IS_DEBUG_SEARCH_ORDER
      Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: deadline fail");
#endif
      //endTime = Utilities::GetProcessorTime();
      //lbTime += endTime - startTime;
      return false;
    }
  }
  //endTime = Utilities::GetProcessorTime();
  //lbTime += endTime - startTime;

  //startTime = Utilities::GetProcessorTime();
  // If we are scheduling for register pressure only, and this branch
  // defines a register but does not use any, we can prune this branch
  // if another instruction in the ready list does use a register.
  if (SchedForRPOnly_) {
    if (inst != NULL && crntNode_->FoundInstWithUse() &&
        inst->GetAdjustedUseCnt() == 0 && !dataDepGraph_->DoesFeedUser(inst)) { 
          //endTime = Utilities::GetProcessorTime();
          //useCntTime += endTime - startTime;
          return false;
        }
  }
  //endTime = Utilities::GetProcessorTime();
  //useCntTime += endTime - startTime;

  //startTime = Utilities::GetProcessorTime();
  if (prune_.nodeSup) {
    if (inst != NULL) {
      if (crntNode_->WasSprirNodeExmnd(inst)) {
        stats::nodeSuperiorityInfeasibilityHits++;
      nodeSupInfsbl++;
        isNodeDmntd = true;
#ifdef IS_DEBUG_SEARCH_ORDER
        Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: history fail");
#endif

        //endTime = Utilities::GetProcessorTime();
        //nodeSupTime += endTime - startTime;
        return false;
      }
    }
  }
  //endTime = Utilities::GetProcessorTime();
  //nodeSupTime += endTime - startTime;

  //startTime = Utilities::GetProcessorTime();

  if (inst != NULL) {
    inst->Schedule(crntCycleNum_, crntSlotNum_, SolverID_);
    DoRsrvSlots_(inst);
    state_.instSchduld = true;
  }
  //endTime = Utilities::GetProcessorTime();
  //instSchedulingTime += endTime - startTime;

  //startTime = Utilities::GetProcessorTime();
  fsbl = ProbeIssuSlotFsblty_(inst);
  state_.issuSlotsProbed = true;

  if (!fsbl) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
    stats::slotCountInfeasibilityHits++;
#endif
  slotCntInfsbl++;
#ifdef IS_DEBUG_SEARCH_ORDER
    Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: issue slot fail");
#endif
    return false;
  }
  //endTime = Utilities::GetProcessorTime();
  //issueSlotTime += endTime - startTime;

  if (bbt_->isSecondPass()) {
    fsbl = TightnLwrBounds_(inst);
    state_.lwrBoundsTightnd = true;
  }

  if (fsbl == false) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
    stats::rangeTighteningInfeasibilityHits++;
#endif
  rangeTightInfsbl++;

#ifdef IS_DEBUG_SEARCH_ORDER
    Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: tightn LB fail");
#endif
    return false;
  }

  state_.instFxd = true;

  //startTime = Utilities::GetProcessorTime();
#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorLock();
#endif
  newNode = nodeAlctr_->Alloc(crntNode_, inst, this);
#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorUnlock();
#endif
  newNode->SetLwrBounds(DIR_FRWRD);
  newNode->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);
  //endTime = Utilities::GetProcessorTime();

  //nodeAllocTime += endTime - startTime;

  // If a node (sub-problem) that dominates the candidate node (sub-problem)
  // has been examined already and found infeasible

  //startTime = Utilities::GetProcessorTime();
  //if (inst != NULL)
    //Logger::Info("Solver %d checking HistDom for inst %d", SolverID_, inst->GetNum());
  if (prune_.histDom && IsHistDom()) {
    if (isEarlySubProbDom_) {
      //Logger::Info("calling wasdmntSubProb from reg Enum");
      if (WasDmnntSubProbExmnd_(inst, newNode)) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
        stats::historyDominationInfeasibilityHits++;
#endif
  histDomInfsbl++;
#ifdef IS_DEBUG_SEARCH_ORDER
        Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: histDom fail");
#endif
  //endTime = Utilities::GetProcessorTime();
  //histDomTime += endTime - startTime;
        return false;
      }
      //endTime = Utilities::GetProcessorTime();
      //histDomTime += endTime - startTime;
    }
  }

  //Milliseconds startTime = Utilities::GetProcessorTime();
  // Try to find a relaxed schedule for the unscheduled instructions
  if (prune_.rlxd && bbt_->isSecondPass()) {
    fsbl = RlxdSchdul_(newNode);
    state_.rlxSchduld = true;
  //relaxedTime += Utilities::GetProcessorTime() - startTime;

    if (fsbl == false) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
      stats::relaxedSchedulingInfeasibilityHits++;
#endif

  relaxedSchedInfsbl++;

      isRlxInfsbl = true;
#ifdef IS_DEBUG_SEARCH_ORDER
      Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: relaxed fail");
#endif
        return false;
    }
  }

  isLngthFsbl = true;
  assert(newNode != NULL);

  return true;
}
/****************************************************************************/

bool Enumerator::ProbeIssuSlotFsblty_(SchedInstruction *inst, bool trueProbe) {
  bool endOfCycle = crntSlotNum_ == issuRate_ - 1;
  IssueType issuType = inst == NULL ? ISSU_STALL : inst->GetIssueType();

  if (issuType != ISSU_STALL) {
    //Logger::Info("avlblSlotsinCrntCycle_[issuType] %d issuType %d", avlblSlotsInCrntCycle_[issuType], issuType);
    //Logger::Info("avblSlots[issuType] %d", avlblSlots_[issuType]);
    assert(avlblSlotsInCrntCycle_[issuType] > 0);
    assert(avlblSlots_[issuType] > 0);
    avlblSlotsInCrntCycle_[issuType]--;
    avlblSlots_[issuType]--;
    //Logger::Info("before decrementing, neededSlots_ %d", neededSlots_[issuType]);
    neededSlots_[issuType]--;

    //Logger::Info("avlblSlots_[issuType] %d, needeSlots_[issuType] %d", avlblSlots_[issuType], neededSlots_[issuType]); 
    if (trueProbe) assert(avlblSlots_[issuType] >= neededSlots_[issuType]);
  }

  int16_t i;

  for (i = 0; i < issuTypeCnt_; i++) {
    // Due to stalls in the cycle that has just completed, the available
    // slots for each type that did not get filled can never be used.
    // This could not have been decremented when the stalls were
    // scheduled, because it was not clear then which type was affected
    // by each stall
    if (endOfCycle) {
      avlblSlots_[i] -= avlblSlotsInCrntCycle_[i];
      avlblSlotsInCrntCycle_[i] = 0;
    }

    if (avlblSlots_[i] < neededSlots_[i]) {
      return false;
    }
  }

  return true;
}
/*****************************************************************************/

void Enumerator::RestoreCrntState_(SchedInstruction *inst,
                                   EnumTreeNode *newNode) {
  #ifdef IS_DEBUG_METADATA
  Milliseconds startTime = Utilities::GetProcessorTime();
  #endif

  if (newNode != NULL) {
    if (newNode->IsArchived() == false) {
      nodeAlctr_->Free(newNode);
    }
  }

  if (state_.lwrBoundsTightnd && bbt_->isSecondPass()) {
    UnTightnLwrBounds_(inst);
  }

  if (state_.instSchduld) {
    assert(inst != NULL);
    UndoRsrvSlots_(inst);
    inst->UnSchedule(SolverID_);
  }

  if (state_.issuSlotsProbed) {
    crntNode_->GetSlotAvlblty(avlblSlots_, avlblSlotsInCrntCycle_);

    if (inst != NULL) {
      IssueType issuType = inst->GetIssueType();
      neededSlots_[issuType]++;
    }
  }

  ClearState_();

  #ifdef IS_DEBUG_METADATA
  restoreTime += Utilities::GetProcessorTime() - startTime;
  #endif

}
/*****************************************************************************/

void Enumerator::StepFrwrd_(EnumTreeNode *&newNode) {
  SchedInstruction *instToSchdul = newNode->GetInst();
  InstCount instNumToSchdul;
#ifdef IS_CORRECT_LOCALPOOL
  Logger::Info("SolverID %d stepping frwrd to time %d", SolverID_, newNode->GetTime());
#endif
#ifdef IS_DEBUG_SEARCH_ORDER
  if (instToSchdul)
    Logger::Log((Logger::LOG_LEVEL) 4, false, "Stepping forward to inst %d", instToSchdul->GetNum());
#endif

  // TODO: toggle work stealing on-off

if (bbt_->isWorkStealOn()) {
  if (bbt_->isWorker()) {
    bool pushedToLocal = false;
    if (!crntNode_->getPushedToLocalPool()) {
      if (bbt_->getLocalPoolSize(SolverID_ - 2) < bbt_->getLocalPoolMaxSize(SolverID_ - 2)) {
        //rdyLst_->ResetIterator();
        LinkedList<SchedInstruction> fillList;
        //Logger::Info("rdyLst has %d insts",rdyLst_->GetInstCnt());
        rdyLst_->GetUnscheduledInsts(&fillList);
        //Logger::Info("fillList has %d insts", fillList.GetElmntCnt());
        EnumTreeNode *pushNode;
        if (fillList.GetElmntCnt() > 0) {
#ifdef IS_CORRECT_LOCALPOOL
          Logger::Info("SolverID %d added %d elements to local pool with time %d", SolverID_, fillList.GetElmntCnt(), crntNode_->GetTime());
#endif
          pushedToLocal = true;
          fillList.ResetIterator();
          SchedInstruction *temp = fillList.GetFrstElmnt();
          bbt_->localPoolLock(SolverID_ - 2);
          while (temp != NULL) {
  #ifdef IS_SYNCH_ALLOC
            bbt_->allocatorLock();
  #endif
            pushNode = nodeAlctr_->Alloc(crntNode_, temp, this, false);
  #ifdef IS_SYNCH_ALLOC
            bbt_->allocatorUnlock();
  #endif
          //rdyLst_->RemoveNextPriorityInst();

            bbt_->localPoolPushFront(SolverID_ - 2, pushNode);
            temp = fillList.GetNxtElmnt();
          //Logger::Info("Solver %d pushed inst into localpool", SolverID_);
          //Logger::Info("Solver %d localPoolSize %d", SolverID_, bbt_->getLocalPoolSize(SolverID_ - 2));
          }
          //TEST CODE
          

          // need to find a way to make this nonatomic
          // relying on it being atomic is fine for the specific inst we are 
          // stepping fwrd to, but the remaining insts will not see
          // the stolen inst

          //CreateNewRdyLst_();
          //newNode->SetRdyLst(rdyLst_);
          
          bbt_->localPoolUnlock(SolverID_ - 2);
        }
      }
      crntNode_->setPushedToLocalPool(pushedToLocal);

    }
  }  
}

//TEST CODE
/*
if (!crntNode_->getPushedToLocalPool() || !bbt_->isWorker() || isSecondPass()) {
  CreateNewRdyLst_();
  newNode->SetRdyLst(rdyLst_);
}
*/

  //if (!crntNode_->getPushedToLocalPool()) CreateNewRdyLst_();
  //if (crntNode_->getPushedToLocalPool()) bbt_->localPoolLock(SolverID_ - 2);
  CreateNewRdyLst_();
  //if (crntNode_->getPushedToLocalPool()) bbt_->localPoolUnlock(SolverID_ - 2);

  // correctness issue
  // the new node should inherit the parents ready list before any thread potentiallly
  // removes an instruction from it. If we dont account for this then there is a 
  // potential correctness issue.

  // The solution should have createRdyLst in critical section, and
  // only perform the following lines if the crntNode hasnot pushed to local

  // Note: only CreateNewRdyLst_ needs to be in critical section (not setting the rdyLst)
  // code in current form attempts to solve these problems



  // this code compiles and runs if we remove createNEwRdyLst in crit seciton above
  //if (crntNode_->getPushedToLocalPool()) bbt_->localPoolLock(SolverID_ - 2);
  //CreateNewRdyLst_();
  // Let the new node inherit its parent's ready list before we update it
  newNode->SetRdyLst(rdyLst_);
  //if (crntNode_->getPushedToLocalPool()) bbt_->localPoolUnlock(SolverID_ - 2);

  if (instToSchdul == NULL) {
    instNumToSchdul = SCHD_STALL;
  } else {
    instNumToSchdul = instToSchdul->GetNum();
    SchdulInst_(instToSchdul, crntCycleNum_);
    if (crntNode_->getPushedToLocalPool()) bbt_->localPoolLock(SolverID_ - 2);
    rdyLst_->RemoveNextPriorityInst();
    if (crntNode_->getPushedToLocalPool()) bbt_->localPoolUnlock(SolverID_ - 2);
    if (instToSchdul->GetTplgclOrdr() == minUnschduldTplgclOrdr_) {
      minUnschduldTplgclOrdr_++;
    }
  }

  crntSched_->AppendInst(instNumToSchdul);

  MovToNxtSlot_(instToSchdul);
  assert(crntCycleNum_ <= trgtSchedLngth_);


  if (crntSlotNum_ == 0) {
    InitNewCycle_();
  }

  InitNewNode_(newNode);

#ifdef INSERT_ON_STEPFRWRD
  if (!isSecondPass()) {
    if (IsHistDom()) {
      assert(!crntNode_->IsArchived());
        UDT_HASHVAL key = exmndSubProbs_->HashKey(crntNode_->GetSig());

      if (bbt_->isWorker()) {
        bbt_->histTableLock(key);
          HistEnumTreeNode *crntHstry = crntNode_->GetHistory();
          /*if (crntHstry->GetParent() != NULL) {
            Logger::Info("parent inst %d", crntHstry->GetParent()->GetInstNum());
          }*/
          //crntHstry->Copy(crntNode_->GetHistory());
  #ifdef IS_SYNCH_ALLOC
          bbt_->allocatorLock();
  #endif
          exmndSubProbs_->InsertElement(crntNode_->GetSig(), crntHstry,
                                    hashTblEntryAlctr_, bbt_);
  #ifdef IS_SYNCH_ALLOC
          bbt_->allocatorUnlock();
  #endif
          SetTotalCostsAndSuffixes(crntNode_, crntNode_->GetParent(), trgtSchedLngth_,
                              prune_.useSuffixConcatenation);
          crntNode_->Archive();
        bbt_->histTableUnlock(key);
      }

      else {
        HistEnumTreeNode *crntHstry = crntNode_->GetHistory();
        exmndSubProbs_->InsertElement(crntNode_->GetSig(), crntHstry,
                                    hashTblEntryAlctr_, bbt_);
        SetTotalCostsAndSuffixes(crntNode_, crntNode_->GetParent(), trgtSchedLngth_,
                              prune_.useSuffixConcatenation);
        crntNode_->Archive();
      }
        

    } else {
      assert(crntNode_->IsArchived() == false);
    }
  }
#endif


#ifdef IS_DEBUG_FLOW
  Logger::Info("Stepping forward from node %lld to node %lld by scheduling "
               "inst. #%d in cycle #%d. CostLB=%d",
               crntNode_->GetParent()->GetNum(), crntNode_->GetNum(),
               instNumToSchdul, crntCycleNum_, crntNode_->GetCostLwrBound());
#endif

  CmtLwrBoundTightnng_();
  ClearState_();
}
/*****************************************************************************/

void Enumerator::InitNewNode_(EnumTreeNode *newNode) {
#ifdef INSTPREFIX
  LinkedList<int> *temp = crntNode_->getInstPrefix();
  newNode->copyInstPrefix(temp);
  newNode->pushToInstPrefix(crntNode_->GetInstNum());
#endif
  crntNode_ = newNode;

  crntNode_->SetCrntCycleBlkd(isCrntCycleBlkd_);
  crntNode_->SetRealSlotNum(crntRealSlotNum_);

  if (IsHistDom()) {
    crntNode_->CreateHistory();
    assert(crntNode_->GetHistory() != tmpHstryNode_);
  }

  crntNode_->SetSlotAvlblty(avlblSlots_, avlblSlotsInCrntCycle_);

  UpdtRdyLst_(crntCycleNum_, crntSlotNum_);
  bool isLeaf = schduldInstCnt_ == totInstCnt_;

  crntNode_->SetBranchCnt(rdyLst_->GetInstCnt(), isLeaf);

  createdNodeCnt_++;
  crntNode_->SetNum(createdNodeCnt_);

  /*if (crntNode_->GetParent() == rootNode_) {
    Logger::Info("first level node has time %d", crntNode_->GetTime());
    Logger::Info("zeroth level node has time %d", rootNode_->GetTime());
  }*/
}

/*****************************************************************************/
void Enumerator::InitNewGlobalPoolNode_(EnumTreeNode *newNode) {
  crntNode_ = newNode;

  
  crntNode_->SetCrntCycleBlkd(isCrntCycleBlkd_);
  crntNode_->SetRealSlotNum(crntRealSlotNum_);
  
  crntNode_->SetSlotAvlblty(avlblSlots_, avlblSlotsInCrntCycle_);
  

  UpdtRdyLst_(crntCycleNum_, crntSlotNum_);
  bool isLeaf = schduldInstCnt_ == totInstCnt_;

  crntNode_->SetBranchCnt(rdyLst_->GetInstCnt(), isLeaf);
  
  createdNodeCnt_++;
  crntNode_->SetNum(createdNodeCnt_);
  
}

/*****************************************************************************/

void Enumerator::SetTotalCostsAndSuffixes(EnumTreeNode *const currentNode,
                              EnumTreeNode *const parentNode,
                              const InstCount targetLength,
                              const bool suffixConcatenationEnabled) {
  // (Chris): Before archiving, set the total cost info of this node. If it's a
  // leaf node, then the total cost is the current cost. If it's an inner node,
  // then the total cost either has already been set (if one of its children had
  // a real cost), or hasn't been set, which means the total cost right now is
  // the dynamic lower bound of this node.


  //Logger::Info("in setTotalCostsAndsuxxi");

  if (currentNode->IsLeaf()) {
#if defined(IS_DEBUG_ARCHIVE)
    Logger::Info("Leaf node total cost %d", currentNode->GetCost());
#endif
    currentNode->SetTotalCost(currentNode->GetCost());
    currentNode->SetTotalCostIsActualCost(true);
  } else {
    if (!currentNode->GetTotalCostIsActualCost() &&
        (currentNode->GetTotalCost() == -1 ||
         currentNode->GetCostLwrBound() < currentNode->GetTotalCost())) {
#if defined(IS_DEBUG_ARCHIVE)
      Logger::Info("Inner node doesn't have a real cost yet. Setting total "
                   "cost to dynamic lower bound %d",
                   currentNode->GetCostLwrBound());
#endif
      currentNode->SetTotalCost(currentNode->GetCostLwrBound());
    }
  }

#if defined(IS_DEBUG_SUFFIX_SCHED)
  if (currentNode->GetTotalCostIsActualCost() &&
      currentNode->GetTotalCost() == -1) {
    Logger::Fatal("Actual cost was not set even though its flag was!");
  }
#endif

  // (Chris): If this node has an actual cost associated with the best schedule,
  // we want to propagate it backward only if this node's cost is less than the
  // parent node's cost.
  std::vector<SchedInstruction *> parentSuffix;
  if (parentNode != nullptr) {
    if (currentNode->GetTotalCostIsActualCost()) {
      if (suffixConcatenationEnabled &&
          (currentNode->IsLeaf() ||
           (!currentNode->IsLeaf() && currentNode->GetSuffix().size() > 0))) {
        parentSuffix.reserve(currentNode->GetSuffix().size() + 1);
        parentSuffix.push_back(currentNode->GetInst());
        parentSuffix.insert(parentSuffix.end(),
                            currentNode->GetSuffix().begin(),
                            currentNode->GetSuffix().end());
      }
      if (!parentNode->GetTotalCostIsActualCost()) {
#if defined(IS_DEBUG_ARCHIVE)
        Logger::Info("Current node has a real cost, but its parent doesn't. "
                     "Settings parent's total cost to %d",
                     currentNode->GetTotalCost());
#endif
        parentNode->SetTotalCost(currentNode->GetTotalCost());
        parentNode->SetTotalCostIsActualCost(true);
        parentNode->SetSuffix(std::move(parentSuffix));
      } else if (currentNode->GetTotalCost() < parentNode->GetTotalCost()) {
#if defined(IS_DEBUG_ARCHIVE)
        Logger::Info(
            "Current node has a real cost (%d), and so does parent. (%d)",
            currentNode->GetTotalCost(), parentNode->GetTotalCost());
#endif
        parentNode->SetTotalCost(currentNode->GetTotalCost());
        parentNode->SetSuffix(std::move(parentSuffix));
      }
    }
  }

// (Chris): Ensure that the prefix and the suffix of the current node contain
// no common instructions. This can be compiled out once the code is working.
#if defined(IS_DEBUG_SUFFIX_SCHED)
  if (suffixConcatenationEnabled) {

    void printVector(const std::vector<InstCount> &v, const char *label) {
      std::stringstream s;
      for (auto i : v)
        s << i << ' ';
      Logger::Info("%s: %s", label, s.str().c_str());
    }

    std::vector<InstCount> prefix;
    for (auto n = currentNode; n != nullptr; n = n->GetParent()) {
      if (n->GetInstNum() != SCHD_STALL)
        prefix.push_back(n->GetInstNum());
    }
    auto sortedPrefix = prefix;
    std::sort(sortedPrefix.begin(), sortedPrefix.end());

    std::vector<InstCount> suffix;
    for (auto i : currentNode->GetSuffix()) {
      suffix.push_back(i->GetNum());
    }
    auto sortedSuffix = suffix;
    std::sort(sortedSuffix.begin(), sortedSuffix.end());

    std::vector<InstCount> intersection;
    std::set_intersection(sortedPrefix.begin(), sortedPrefix.end(),
                          sortedSuffix.begin(), sortedSuffix.end(),
                          std::back_inserter(intersection));

    auto printVector = [](const std::vector<InstCount> &v, const char *prefix) {
      std::stringstream s;
      for (auto i : v)
        s << i << ' ';
      Logger::Error("SetTotalCostsAndSuffixes: %s: %s", prefix,
                    s.str().c_str());
    };
    if (intersection.size() != 0) {
      printVector(prefix, "prefix");
      printVector(suffix, "suffix");
      printVector(intersection, "intersection");
      Logger::Error("SetTotalCostsAndSuffixes: Error occurred when archiving "
                    "node with InstNum %d",
                    currentNode->GetInstNum());
      Logger::Fatal(
          "Prefix schedule and suffix schedule contain common instructions!");
    }
    if (suffix.size() > 0 && suffix.size() + prefix.size() != targetLength) {
      printVector(prefix, "prefix");
      printVector(suffix, "suffix");
      Logger::Fatal("Sum of suffix (%llu) and prefix (%llu) sizes doesn't "
                    "match target length %d!",
                    suffix.size(), prefix.size(), targetLength);
    }
    CheckHistNodeMatches(currentNode, currentNode->GetHistory(),
                         "SetTotalCostsAndSuffixes: CheckHistNodeMatches");
  }
#endif
}


bool Enumerator::BackTrack_(bool trueState) {
  bool fsbl = true;
  SchedInstruction *inst = crntNode_->GetInst();
  EnumTreeNode *trgtNode = crntNode_->GetParent();
#ifdef IS_CORRECT_LOCALPOOL
  Logger::Info("SolverID %d backtracking to time %d", SolverID_, trgtNode->GetTime());
#endif
  if (crntNode_->GetInst()) {
#ifdef IS_DEBUG_SEARCH_ORDER
    Logger::Log((Logger::LOG_LEVEL) 4, false, "SolverID %d Back tracking fron inst %d to inst %d", SolverID_, inst->GetNum(), trgtNode->GetInstNum());
#endif
}
  rdyLst_->RemoveLatestSubList();

#ifdef INSERT_ON_BACKTRACK
  if (IsHistDom() && trueState) {
    assert(!crntNode_->IsArchived());
      UDT_HASHVAL key = exmndSubProbs_->HashKey(crntNode_->GetSig());

    if (bbt_->isWorker()) {
      bbt_->histTableLock(key);
        HistEnumTreeNode *crntHstry = crntNode_->GetHistory();
#ifdef IS_SYNCH_ALLOC
        bbt_->allocatorLock();
#endif
        exmndSubProbs_->InsertElement(crntNode_->GetSig(), crntHstry,
                                  hashTblEntryAlctr_, bbt_);
#ifdef IS_SYNCH_ALLOC
        bbt_->allocatorUnlock();
#endif
        SetTotalCostsAndSuffixes(crntNode_, trgtNode, trgtSchedLngth_,
                             prune_.useSuffixConcatenation);
        crntNode_->Archive();
      bbt_->histTableUnlock(key);
    }

    else {
      HistEnumTreeNode *crntHstry = crntNode_->GetHistory();
      exmndSubProbs_->InsertElement(crntNode_->GetSig(), crntHstry,
                                  hashTblEntryAlctr_, bbt_);
      SetTotalCostsAndSuffixes(crntNode_, trgtNode, trgtSchedLngth_,
                             prune_.useSuffixConcatenation);
      crntNode_->Archive();
    }
      

  } else {
    assert(crntNode_->IsArchived() == false);
  }
#endif
#ifndef INSERT_ON_BACKTRACK
if (isSecondPass()) {
    if (IsHistDom() && trueState) {
      assert(!crntNode_->IsArchived());
        UDT_HASHVAL key = exmndSubProbs_->HashKey(crntNode_->GetSig());

      if (bbt_->isWorker()) {
        bbt_->histTableLock(key);
          HistEnumTreeNode *crntHstry = crntNode_->GetHistory();
  #ifdef IS_SYNCH_ALLOC
          bbt_->allocatorLock();
  #endif
          exmndSubProbs_->InsertElement(crntNode_->GetSig(), crntHstry,
                                    hashTblEntryAlctr_, bbt_);
  #ifdef IS_SYNCH_ALLOC
          bbt_->allocatorUnlock();
  #endif
          SetTotalCostsAndSuffixes(crntNode_, trgtNode, trgtSchedLngth_,
                              prune_.useSuffixConcatenation);
          crntNode_->Archive();
        bbt_->histTableUnlock(key);
      }

      else {
        HistEnumTreeNode *crntHstry = crntNode_->GetHistory();
        exmndSubProbs_->InsertElement(crntNode_->GetSig(), crntHstry,
                                    hashTblEntryAlctr_, bbt_);
        SetTotalCostsAndSuffixes(crntNode_, trgtNode, trgtSchedLngth_,
                              prune_.useSuffixConcatenation);
        crntNode_->Archive();
      }
        

    } else {
      assert(crntNode_->IsArchived() == false);
    }
  }
  //TODOJEFF
  /*
  else {
    SetTotalCostsAndSuffixes(crntNode_, trgtNode, trgtSchedLngth_,
                           prune_.useSuffixConcatenation);
    crntNode_->Archive();
  }*/
#endif
 
  if (!crntNode_->wasChildStolen())
    nodeAlctr_->Free(crntNode_);
  else {
    trgtNode->setChildStolen(true);
  }

  EnumTreeNode *prevNode = crntNode_;
  crntNode_ = trgtNode;

  //Logger::Info("backtracking to time %d", crntNode_->GetTime());
  rdyLst_ = crntNode_->GetRdyLst();
/*
#ifdef WORK_STEAL
  if (bbt_->isWorker()) {
    bbt_->localPoolLock(SolverID_ - 2);
    rdyLst_ = crntNode_->GetRdyLst();
    bbt_->localPoolUnlock(SolverID_ - 2);
  }
  else {
    rdyLst_ = crntNode_->GetRdyLst();
  }
#endif
*/
/*
#ifdef WORK_STEAL
  // we need to synchronize on trgNode->rdyLst as well since
  // the stealing thread modifies the trgtNodes ready list when stealing
  bbt_->localPoolLock(SolverID_ - 2);
  rdyLst_ = crntNode_->GetRdyLst(); 
  if (bbt_->getLocalPoolSize(SolverID_ - 2) > 0 && bbt_->isWorker()) {
    //Logger::Info("SolverID %d checking its own local pool", SolverID_);

    EnumTreeNode *popNode = bbt_->localPoolPopFront(SolverID_ - 2);
    assert(popNode);
    //Logger::Info("popNode has time %d, prevNode has time %d", popNode->GetTime(), prevNode->GetTime());
    assert(popNode->GetTime() <= prevNode->GetTime());

    while (popNode->GetTime() == prevNode->GetTime()) {
      assert(popNode->GetParent() == crntNode_);
      nodeAlctr_->Free(popNode);
      if (bbt_->getLocalPoolSize(SolverID_ - 2) == 0) break;
      popNode = bbt_->localPoolPopFront(SolverID_ - 2);
    }

    if (popNode->GetTime() != prevNode->GetTime()) {
      bbt_->localPoolPushFront(SolverID_- 2,popNode);
    }
    
    else {
      //Logger::Info("making allocated node available on free list");
    }
   
    /*
    int size = bbt_->getLocalPoolSize(SolverID_ - 2);
    for (int i = 0; i < size; i++) {
      EnumTreeNode *popNode = bbt_->localPoolPopTail(SolverID_ - 2);
      assert(popNode->GetTime() <= prevNode->GetTime());
      if (popNode->GetTime() > prevNode->GetTime()) {
        Logger::Info("assumption invalidated, popNode time %d prevNode time %d", popNode->GetTime(), prevNode->GetTime());
        Logger::Info("is match? %d", popNode->GetParent() == crntNode_);
      }
      if (popNode->GetParent() == crntNode_) {
        //Logger::Info("found a match, popnode time %d prevNodew time %d", popNode->GetTime(), prevNode->GetTime());
      }
      else {
        bbt_->localPoolPushFront(SolverID_ - 2, popNode);
      }
    }
    /


    //size = bbt_->getLocalPoolSize(SolverID_ - 2)
  }

  bbt_->localPoolUnlock(SolverID_ - 2);
#endif
*/

  MovToPrevSlot_(crntNode_->GetRealSlotNum());

  trgtNode->NewBranchExmnd(inst, true, false, false, crntNode_->IsFeasible(),
                           DIR_BKWRD, prevNode->IsLngthFsbl());

#ifdef IS_DEBUG_FLOW
  InstCount instNum = inst == NULL ? SCHD_STALL : inst->GetNum();
  Logger::Info("Backtracking from node %lld to node %lld by unscheduling inst. "
               "#%d in cycle #%d. CostLB=%d",
               prevNode->GetNum(), trgtNode->GetNum(), instNum, crntCycleNum_,
               trgtNode->GetCostLwrBound());
#endif

  crntNode_->GetSlotAvlblty(avlblSlots_, avlblSlotsInCrntCycle_);
  isCrntCycleBlkd_ = crntNode_->GetCrntCycleBlkd();

  if (inst != NULL) {
    IssueType issuType = inst->GetIssueType();
    neededSlots_[issuType]++;
  }

  crntSched_->RemoveLastInst();
  RestoreCrntLwrBounds_(inst, trueState);

  if (inst != NULL) {
    // int hitCnt;
    // assert(rdyLst_->FindInst(inst, hitCnt) && hitCnt == 1);
    assert(inst->IsInReadyList(SolverID_));

    UndoRsrvSlots_(inst);
    UnSchdulInst_(inst);
    inst->UnSchedule(SolverID_);

    if (inst->GetTplgclOrdr() == minUnschduldTplgclOrdr_ - 1) {
      minUnschduldTplgclOrdr_--;
    }
  }


  backTrackCnt_++;
  return fsbl;
}
/*****************************************************************************/

bool Enumerator::WasDmnntSubProbExmnd_(SchedInstruction *,
                                       EnumTreeNode *&newNode) {

  //Logger::Info("in wasDmntSubProbExmnd");
#ifdef IS_DEBUG_SPD
  stats::signatureDominationTests++;
#endif
  HistEnumTreeNode *exNode;
  int listSize = exmndSubProbs_->GetListSize(newNode->GetSig());
  //UDT_HASHVAL key = exmndSubProbs_->HashKey(newNode->GetSig());
  stats::historyListSize.Record(listSize);
  //Logger::Info("bucket has size of %d and key %d", listSize, key);
  //Logger::Log((Logger::LOG_LEVEL)4, false, "there are %d nodes in the history bucket", listSize);
  if (listSize == 0) return false;
  mostRecentMatchingHistNode_ = nullptr;
  bool mostRecentMatchWasSet = false;
  bool wasDmntSubProbExmnd = false;
  int trvrsdListSize = 0;

  // lock table for syncrhonized iterator
  
  //bbt_->histTableLock(key);
  //Logger::Info("Solver %d, made it through door %d", SolverID_, key);
  //Logger::Info("Solver %d inside lock key %d, instNum %d", SolverID_, key, newNode->GetInstNum());
  //Logger::Info("histTable has GetEntryCnt of %d", exmndSubProbs_->GetEntryCnt());
  HashTblEntry<HistEnumTreeNode> *srchPtr = nullptr;
  /*if (isGenerateState_) {
    Logger::Info("getting last match for key %d", key);
    Logger::Info("crnt node is %d", newNode->GetInstNum());
  }*/
  exNode = exmndSubProbs_->GetLastMatch(srchPtr,newNode->GetSig());
  /*if (isGenerateState_) {
    Logger::Info("found last match, it has address %p", exNode);
    Logger::Info("last match has insts: ");
    HistEnumTreeNode *crntNode2 = exNode;
    while (crntNode2 != NULL) {
      Logger::Info("%d", crntNode2->GetInstNum());
      crntNode2 = crntNode2->GetParent();
    }
  }*/
  for (; trvrsdListSize < listSize; trvrsdListSize++) {
    if (exNode == NULL || exNode == nullptr) break;

#ifdef IS_DEBUG_SPD
    stats::signatureMatches++;
#endif
    //Logger::Log((Logger::LOG_LEVEL)4, false, "checking to see if history node with inst %d matches", exNode->getInstNum());
    if (exNode->DoesMatch(newNode, this, bbt_->isWorker(), isGenerateState_)) {
      //Logger::Log((Logger::LOG_LEVEL)4, false, "checking against matching history node with inst %d", exNode->getInstNum());
      if (!mostRecentMatchWasSet) {
        mostRecentMatchingHistNode_ =
            (exNode->GetSuffix() != nullptr) ? exNode : nullptr;
        mostRecentMatchWasSet = true;
      }
      bool doesDominate = exNode->DoesDominate(newNode, this);
      if (doesDominate) {
        
#ifdef IS_DEBUG_SPD
        Logger::Info("Node %d is dominated. Partial scheds:",
                     newNode->GetNum());
        Logger::Info("Current node:");
        newNode->PrntPartialSched(Logger::GetLogStream());
        Logger::Info("Hist node:");
        exNode->PrntPartialSched(Logger::GetLogStream());
#endif

        nodeAlctr_->Free(newNode);
        newNode = NULL;
        stats::positiveDominationHits++;
#ifdef IS_DEBUG_SPD
        stats::positiveDominationHits++;
        stats::traversedHistoryListSize.Record(trvrsdListSize);
        stats::historyDominationPosition.Record(trvrsdListSize);
        stats::historyDominationPositionToListSize.Record(
            (trvrsdListSize * 100) / listSize);
#endif

        wasDmntSubProbExmnd = true;
        break;
      } else {
#ifdef IS_DEBUG_SPD
        stats::signatureAliases++;
#endif
      }
    }

    exNode = exmndSubProbs_->GetPrevMatch(srchPtr, newNode->GetSig());
  }
  
  // unlock
  //Logger::Info("Solver %d unlocking key %d", SolverID_, key);
  //bbt_->histTableUnlock(key);  

  stats::traversedHistoryListSize.Record(trvrsdListSize);
  return wasDmntSubProbExmnd;
}
/****************************************************************************/

bool Enumerator::TightnLwrBounds_(SchedInstruction *newInst, bool trueTightn) {
  //if (newInst) Logger::Log((Logger::LOG_LEVEL) 4, false, "Calling TLB for inst %d", newInst->GetNum());
  
  SchedInstruction *inst;
  InstCount newLwrBound = 0;
  InstCount nxtAvlblCycle[MAX_ISSUTYPE_CNT];
  bool fsbl;
  InstCount i;

  assert(fxdLst_->GetElmntCnt() == 0);
  assert(tightndLst_->GetElmntCnt() == 0);

  for (i = 0; i < issuTypeCnt_; i++) {
    // If this slot is filled with a stall then all subsequent slots are
    // going to be filled with stalls
    if (newInst == NULL) {
      nxtAvlblCycle[i] = crntCycleNum_ + 1;
    } else {
      // If the last slot for this type has been taken in this cycle
      // then an inst. of this type cannot issue any earlier than the
      // next cycle
      nxtAvlblCycle[i] =
          avlblSlotsInCrntCycle_[i] == 0 ? crntCycleNum_ + 1 : crntCycleNum_;
    }
  }

  for (i = minUnschduldTplgclOrdr_; i < totInstCnt_; i++) {
    inst = dataDepGraph_->GetInstByTplgclOrdr(i);
    //Logger::Info("inst->GetCrntLwrBound() %d, crntCycleNum_ %d, instNum %d", inst->GetCrntLwrBound(DIR_FRWRD, SolverID_), crntCycleNum_, inst->GetNum());
    if (trueTightn)
      assert(inst != newInst ||
            inst->GetCrntLwrBound(DIR_FRWRD, SolverID_) == crntCycleNum_);
      if (inst->IsSchduld(SolverID_) == false) {
      //Logger::Info("inst->GetNum() %d", inst->GetNum());
      //Logger::Info("SolverID_ = %d, inst->IsSchduld = %d, inst->GetNum() %d", SolverID_, inst->IsSchduld(SolverID_), inst->GetNum());
        //if (SolverID_ == 2) Logger::Log((Logger::LOG_LEVEL) 4, false, "inst %d is not scheduled", inst->GetNum());
        IssueType issuType = inst->GetIssueType();
        newLwrBound = nxtAvlblCycle[issuType];


      if (newLwrBound > inst->GetCrntLwrBound(DIR_FRWRD, SolverID_)) {
        //if ((SolverID_) == 2) Logger::Log((Logger::LOG_LEVEL) 4, false, "tlb for inst %d", inst->GetNum()); 
#ifdef IS_DEBUG_FLOW
        Logger::Info("Tightening LB of inst %d from %d to %d", inst->GetNum(),
                     inst->GetCrntLwrBound(DIR_FRWRD, SolverID_), newLwrBound);
#endif
        fsbl = inst->TightnLwrBoundRcrsvly(DIR_FRWRD, newLwrBound, tightndLst_,
                                           fxdLst_, false, SolverID_);

        if (fsbl == false) {
          return false;
        }
      }

      assert(inst->GetCrntLwrBound(DIR_FRWRD, SolverID_) >= newLwrBound);

      if (inst->GetCrntLwrBound(DIR_FRWRD, SolverID_) > inst->GetCrntDeadline(SolverID_)) {
        return false;
      }
    }
  }

  for (inst = tightndLst_->GetFrstElmnt(); inst != NULL;
       inst = tightndLst_->GetNxtElmnt()) {
    dataDepGraph_->SetCrntFrwrdLwrBound(inst, SolverID_);
  }

  return FixInsts_(newInst);
}
/****************************************************************************/

void Enumerator::UnTightnLwrBounds_(SchedInstruction *newInst) {
  UnFixInsts_(newInst);

  SchedInstruction *inst;

  for (inst = tightndLst_->GetFrstElmnt(); inst != NULL;
       inst = tightndLst_->GetNxtElmnt()) {
    inst->UnTightnLwrBounds(SolverID_);
    dataDepGraph_->SetCrntFrwrdLwrBound(inst, SolverID_);
    assert(inst->IsFxd(SolverID_) == false);
  }

  tightndLst_->Reset();
  dirctTightndLst_->Reset();
}
/*****************************************************************************/

void Enumerator::CmtLwrBoundTightnng_() {
  SchedInstruction *inst;

  for (inst = tightndLst_->GetFrstElmnt(); inst != NULL;
       inst = tightndLst_->GetNxtElmnt()) {
    inst->CmtLwrBoundTightnng(SolverID_);
  }

  tightndLst_->Reset();
  dirctTightndLst_->Reset();
  CmtInstFxng_();
}
/*****************************************************************************/

bool Enumerator::FixInsts_(SchedInstruction *newInst) {
  bool fsbl = true;

  bool newInstFxd = false;

  fxdInstCnt_ = 0;


  for (SchedInstruction *inst = fxdLst_->GetFrstElmnt(); inst != NULL;
       inst = fxdLst_->GetNxtElmnt()) {
    assert(inst->IsFxd(SolverID_));
    assert(inst->IsSchduld(SolverID_) == false || inst == newInst);
    fsbl = rlxdSchdulr_->FixInst(inst, inst->GetFxdCycle(SolverID_));

    if (inst == newInst) {
      newInstFxd = true;
      assert(inst->GetFxdCycle(SolverID_) == crntCycleNum_);
    }

    if (fsbl == false) {
#ifdef IS_DEBUG_FLOW
      Logger::Info("Can't fix inst %d in cycle %d", inst->GetNum(),
                   inst->GetFxdCycle());
#endif
      break;
    }

    fxdInstCnt_++;
  }

  if (fsbl)
    if (!newInstFxd && newInst != NULL) {
      if (newInst->IsFxd(SolverID_) == false)
      // We need to fix the new inst. only if it has not been fixed before
      {
        fsbl = rlxdSchdulr_->FixInst(newInst, crntCycleNum_);

        if (fsbl) {
          fxdLst_->InsrtElmnt(newInst);
          fxdInstCnt_++;
        }
      }
    }

  return fsbl;
}
/*****************************************************************************/

void Enumerator::UnFixInsts_(SchedInstruction *newInst) {
  InstCount unfxdInstCnt = 0;
  SchedInstruction *inst;

  for (inst = fxdLst_->GetFrstElmnt(), unfxdInstCnt = 0;
       inst != NULL && unfxdInstCnt < fxdInstCnt_;
       inst = fxdLst_->GetNxtElmnt(), unfxdInstCnt++) {
    assert(inst->IsFxd(SolverID_) || inst == newInst);
    InstCount cycle = inst == newInst ? crntCycleNum_ : inst->GetFxdCycle(SolverID_);
    rlxdSchdulr_->UnFixInst(inst, cycle);
  }

  assert(unfxdInstCnt == fxdInstCnt_);
  fxdLst_->Reset();
  fxdInstCnt_ = 0;
}
/*****************************************************************************/

void Enumerator::printTplgclOrder() {
  SchedInstruction *inst;
  for (int i = minUnschduldTplgclOrdr_; i < totInstCnt_; i++) {
    Logger::Info("getting the %dth instruction", i);
    inst = dataDepGraph_->GetInstByTplgclOrdr(i);
    if (inst)
      Logger::Info("from prntTplgclOrder, inst->getNUm() %d", inst->GetNum());
  }
}

void Enumerator::printRdyLst() {
  rdyLst_->ResetIterator();
  int sizeOfList = rdyLst_->GetInstCnt();
  Logger::Info("ReadyList Contains: ");
  for (int i = 0; i < sizeOfList; i++) {
    Logger::Info("%d", rdyLst_->GetNextPriorityInst()->GetNum());
  }
  rdyLst_->ResetIterator();
}


void Enumerator::CmtInstFxng_() {
  fxdLst_->Reset();
  fxdInstCnt_ = 0;
}
/*****************************************************************************/

void Enumerator::RestoreCrntLwrBounds_(SchedInstruction *unschduldInst, bool trueState) {
  //Logger::Info("in restoreCLB");
  InstCount *frwrdLwrBounds = crntNode_->GetLwrBounds(DIR_FRWRD);
  bool unschduldInstDone = false;

  for (InstCount i = 0; i < totInstCnt_; i++) {
    SchedInstruction *inst = dataDepGraph_->GetInstByIndx(i);
    InstCount fxdCycle = 0;
    bool preFxd = inst->IsFxd(SolverID_);

    if (preFxd) {
      fxdCycle = inst->GetFxdCycle(SolverID_);
    }

    inst->SetCrntLwrBound(DIR_FRWRD, frwrdLwrBounds[i], SolverID_);
    dataDepGraph_->SetCrntFrwrdLwrBound(inst,SolverID_);
    bool postFxd = inst->IsFxd(SolverID_);

    if (preFxd && !postFxd) { // if got untightened and unfixed
      rlxdSchdulr_->UnFixInst(inst, fxdCycle);

      if (inst == unschduldInst) {
        unschduldInstDone = true;
      }
    }
  }

  if (unschduldInst != NULL && !unschduldInstDone) {
    // Assume that the instruction has not been unscheduled yet
    // i.e. lower bound restoration occurs before unscheduling

    //Logger::Info("inst %d is schduld? %d", unschduldInst->GetNum(), unschduldInst->IsSchduld(SolverID_));
    assert(unschduldInst->IsSchduld(SolverID_));

    if (unschduldInst->IsFxd(SolverID_) == false)
    // only if the untightening got it unfixed
    {
      rlxdSchdulr_->UnFixInst(unschduldInst, unschduldInst->GetSchedCycle(SolverID_));
    }
  }
}
/*****************************************************************************/

bool Enumerator::RlxdSchdul_(EnumTreeNode *newNode) {
  assert(newNode != NULL);
  LinkedList<SchedInstruction> *rsrcFxdLst = new LinkedList<SchedInstruction>;

  bool fsbl =
      rlxdSchdulr_->SchdulAndChkFsblty(crntCycleNum_, trgtSchedLngth_ - 1);

  for (SchedInstruction *inst = rsrcFxdLst->GetFrstElmnt(); inst != NULL;
       inst = rsrcFxdLst->GetNxtElmnt()) {
    assert(inst->IsSchduld(SolverID_) == false);
    fsbl = rlxdSchdulr_->FixInst(inst, inst->GetCrntLwrBound(DIR_FRWRD,SolverID_));

    if (fsbl == false) {
      return false;
    }

    fxdLst_->InsrtElmnt(inst);
    fxdInstCnt_++;
#ifdef IS_DEBUG_FIX
    Logger::Info("%d [%d], ", inst->GetNum(), inst->GetFxdCycle());
#endif
  }

  assert(rsrcFxdLst->GetElmntCnt() == 0);
  rsrcFxdLst->Reset();
  delete rsrcFxdLst;
  return fsbl;
}
/*****************************************************************************/

bool Enumerator::IsUseInRdyLst_() {
  assert(rdyLst_ != NULL);
  bool isEmptyNode = false;
  InstCount brnchCnt = crntNode_->GetBranchCnt(isEmptyNode);
  SchedInstruction *inst;
  bool foundUse = false;

#ifdef IS_DEBUG_RP_ONLY
  Logger::Info("Looking for a use in the ready list with nodes:");
  for (int i = 0; i < brnchCnt - 1; i++) {
    inst = rdyLst_->GetNextPriorityInst();
    assert(inst != NULL);
    Logger::Info("#%d:%d", i, inst->GetNum());
  }
  rdyLst_->ResetIterator();
#endif

  for (int i = 0; i < brnchCnt - 1; i++) {
    inst = rdyLst_->GetNextPriorityInst();
    assert(inst != NULL);
    if (inst->GetAdjustedUseCnt() != 0 || dataDepGraph_->DoesFeedUser(inst)) {
      foundUse = true;
#ifdef IS_DEBUG_RP_ONLY
      Logger::Info("Inst %d uses a register", inst->GetNum());
#endif
      break;
    }
#ifdef IS_DEBUG_RP_ONLY
    Logger::Info("Inst %d does not use a register", inst->GetNum());
#endif
  }

  rdyLst_->ResetIterator();
  return foundUse;
}


 void Enumerator::removeInstFromRdyLst_(InstCount instructionNumber) {
    //Logger::Info("attempting to remove %d from readyLst", instructionNumber);
    InstCount rdyLstSize = rdyLst_->GetInstCnt();
    rdyLst_->ResetIterator();

    if (rdyLstSize > 0) {
      //Logger::Info("readyLst contains");
      //printRdyLst();
      for (int i = 0; i < rdyLstSize; i++) {
        SchedInstruction *tempInst = rdyLst_->GetNextPriorityInst();
        //Logger::Info("iterating on %d in rdyLst", tempInst->GetNum());
        if (tempInst->GetNum() == instructionNumber) {
          rdyLst_->RemoveNextPriorityInst();
          break;
        }
      }
      rdyLst_->ResetIterator();
      //Logger::Info("readyLst contains");
      //printRdyLst();
    }
 }
/*****************************************************************************/

void Enumerator::PrintLog_() {
  Logger::Info("--------------------------------------------------\n");

  Logger::Info("Total nodes examined: %lld\n", GetNodeCnt());
  Logger::Info("History table includes %d entries.\n",
               exmndSubProbs_->GetEntryCnt());
  Logger::GetLogStream() << stats::historyEntriesPerIteration;
  Logger::Info("--------------------------------------------------\n");
}
/*****************************************************************************/

bool Enumerator::EnumStall_() { return enblStallEnum_; }


void Enumerator::printInfsbltyHits() {
  
  Logger::Info("Cost Infeasibility Hits = %d",costInfsbl);
  Logger::Info("Relaxed Infeasibility Hits = %d",rlxdInfsbl);
  Logger::Info("Backward LB Infeasibility Hits = %d",bkwrdLBInfsbl);
  Logger::Info("Forward LB Infeasibility Hits = %d",frwrdLBInfsbl);
  Logger::Info("Node Superiority Infeasibility Hits = %d",nodeSupInfsbl);
  Logger::Info("History Domination Infeasibility Hits = %d",histDomInfsbl);
  Logger::Info("Range Tightening Infeasibility Hits = %d",rangeTightInfsbl);
  Logger::Info("Slot Count Infeasibility Hits = %d",slotCntInfsbl);
  
}

void Enumerator::printMetadata() {
  Logger::Info("time spent finding branch %d", findBranchTime);
  Logger::Info("time spent probing %d",probeTime);
  Logger::Info("time spent restoring %d", restoreTime);
  Logger::Info("time spent moving forward %d", moveForwardTime);
  Logger::Info("time spent backtracking %d", backtrackTime);
  Logger::Info("time spent checking soln %d", checkSolnTime);
  Logger::Info("time spent checking cost fsblt for probe %d", tightnLBTime);
}

void Enumerator::printProbeTiming() {
  Logger::Info("Time for prefix %d", prefixTime);
  Logger::Info("Time for LB %d", lbTime);
  Logger::Info("Time for deadline %d", deadlineTime);
  Logger::Info("Time for useCnt %d", useCntTime);
  Logger::Info("Time for nodeSup %d", nodeSupTime);
  Logger::Info("Time for instScheduling %d", instSchedulingTime);
  Logger::Info("Time for issueSlot %d", issueSlotTime);
  Logger::Info("Time for tightnLB %d", tightnLBTime);
  Logger::Info("Time for nodeAlloc %d", nodeAllocTime);
  Logger::Info("Time for histDom %d", histDomTime);
  Logger::Info("Time for relaxedSched %d", relaxedTime);
}

/*****************************************************************************/

LengthEnumerator::LengthEnumerator(
    DataDepGraph *dataDepGraph, MachineModel *machMdl, InstCount schedUprBound,
    int16_t sigHashSize, SchedPriorities prirts, Pruning PruningStrategy,
    bool SchedForRPOnly, bool enblStallEnum, Milliseconds timeout, bool IsSecondPass,
    InstCount preFxdInstCnt, SchedInstruction *preFxdInsts[])
    : Enumerator(dataDepGraph, machMdl, schedUprBound, sigHashSize, prirts,
                 PruningStrategy, SchedForRPOnly, enblStallEnum, timeout, 0, 1, nullptr, 1, IsSecondPass,
                 preFxdInstCnt, preFxdInsts) {
  SetupAllocators_();
  tmpHstryNode_ = new HistEnumTreeNode;
}
/*****************************************************************************/

LengthEnumerator::~LengthEnumerator() {
  Reset();
  FreeAllocators_();
}
/*****************************************************************************/

void LengthEnumerator::SetupAllocators_() {
  int memAllocBlkSize = memAllocBlkSize_;

  Enumerator::SetupAllocators_();

  if (IsHistDom()) {
    histNodeAlctr_ = new MemAlloc<HistEnumTreeNode>(memAllocBlkSize);
  }
}
/****************************************************************************/

void LengthEnumerator::ResetAllocators_() {
  Enumerator::ResetAllocators_();
  if (IsHistDom() && SolverID_ <= 1)
    histNodeAlctr_->Reset();
}
/****************************************************************************/

void LengthEnumerator::FreeAllocators_(){
  if (IsHistDom() && !alctrsFreed_) {
    delete histNodeAlctr_;
    histNodeAlctr_ = NULL;
  }

  Enumerator::FreeAllocators_();//isMaster);

}
/****************************************************************************/

bool LengthEnumerator::IsCostEnum() { return false; }
/*****************************************************************************/

FUNC_RESULT LengthEnumerator::FindFeasibleSchedule(InstSchedule *sched,
                                                   InstCount trgtLngth,
                                                   Milliseconds deadline) {
  return FindFeasibleSchedule_(sched, trgtLngth, deadline);
}
/*****************************************************************************/

void LengthEnumerator::Reset() { Enumerator::Reset(); }
/*****************************************************************************/

bool LengthEnumerator::WasObjctvMet_() {
  bool wasSlonFound = WasSolnFound_();

  return wasSlonFound;
}
/*****************************************************************************/

HistEnumTreeNode *LengthEnumerator::AllocHistNode_(EnumTreeNode *node) {
  HistEnumTreeNode *histNode = histNodeAlctr_->GetObject();
  histNode->Construct(node, false, isGenerateState_);
  return histNode;
}
/*****************************************************************************/

HistEnumTreeNode *LengthEnumerator::AllocTempHistNode_(EnumTreeNode *node) {
  HistEnumTreeNode *histNode = tmpHstryNode_;
  histNode->Construct(node, true, isGenerateState_);
  return histNode;
}
/*****************************************************************************/

void LengthEnumerator::FreeHistNode_(HistEnumTreeNode *histNode) {
  histNode->Clean();
  histNodeAlctr_->FreeObject(histNode);
}
/*****************************************************************************/

LengthCostEnumerator::LengthCostEnumerator(BBThread *bbt,
    DataDepGraph *dataDepGraph, MachineModel *machMdl, InstCount schedUprBound,
    int16_t sigHashSize, SchedPriorities prirts, Pruning PruningStrategy,
    bool SchedForRPOnly, bool enblStallEnum, Milliseconds timeout,
    SPILL_COST_FUNCTION spillCostFunc, bool IsSecondPass, int NumSolvers,  int timeoutToMemblock, std::mutex *AllocatorLock,
    int SolverID, InstCount preFxdInstCnt, SchedInstruction *preFxdInsts[])
    : Enumerator(dataDepGraph, machMdl, schedUprBound, sigHashSize, prirts,
                 PruningStrategy, SchedForRPOnly, enblStallEnum, timeout,
                 SolverID, NumSolvers, AllocatorLock, timeoutToMemblock, IsSecondPass, preFxdInstCnt, preFxdInsts) {
  bbt_ = bbt;
  SolverID_ = SolverID;
  SetupAllocators_();

  costChkCnt_ = 0;
  costPruneCnt_ = 0;
  isEarlySubProbDom_ = false;
  costLwrBound_ = 0;
  spillCostFunc_ = spillCostFunc;
  tmpHstryNode_ = new CostHistEnumTreeNode;
}
/*****************************************************************************/

LengthCostEnumerator::~LengthCostEnumerator() {
  if (!alctrsFreed_ && SolverID_ <= 1) {
    Reset();
    if (SolverID_ > 1) {
      //Logger::Info("resseting node allocator for solverID %d", SolverID_);
      nodeAlctr_->Reset();
      if (IsHistDom()) {
        hashTblEntryAlctr_->Reset();
      }
    }
    FreeAllocators_();
  }
}
/*****************************************************************************/

void LengthCostEnumerator::destroy() {
  if (!alctrsFreed_) {
    Reset();
    if (SolverID_ > 1) {
      //Logger::Info("resseting node allocator for solverID %d", SolverID_);
      nodeAlctr_->Reset();
      if (IsHistDom()) {
        hashTblEntryAlctr_->Reset();
      }
    }
    //FreeAllocators_();
  }
}



void LengthCostEnumerator::SetupAllocators_() {
  int memAllocBlkSize = memAllocBlkSize_;

  Enumerator::SetupAllocators_();

  if (IsHistDom()) {
    histNodeAlctr_ = new MemAlloc<CostHistEnumTreeNode>(memAllocBlkSize);
  }
}
/****************************************************************************/

void LengthCostEnumerator::ResetAllocators_() {
  if (!alctrsFreed_) {
    Enumerator::ResetAllocators_();
    if (IsHistDom() && SolverID_ <= 1)
      histNodeAlctr_->Reset();
  }
}
/****************************************************************************/

void LengthCostEnumerator::FreeAllocators_(){
  if (IsHistDom() & !alctrsFreed_) {
    Logger::Info("SolverID %d freeing history allocator with %d blocks", SolverID_, histNodeAlctr_->GetSize());
    if (histNodeAlctr_ != NULL)
      delete histNodeAlctr_;
    histNodeAlctr_ = NULL;
  }
  
  Enumerator::FreeAllocators_();


}

void LengthCostEnumerator::deleteNodeAlctr() {
  Enumerator::deleteNodeAlctr();
}
/****************************************************************************/

bool LengthCostEnumerator::IsCostEnum() { return true; }
/*****************************************************************************/

void LengthCostEnumerator::Reset() { Enumerator::Reset(); }
/*****************************************************************************/

bool LengthCostEnumerator::Initialize_(InstSchedule *preSched,
                                       InstCount trgtLngth, int SolverID, bool ScheduleRoot) {
  bool fsbl = Enumerator::Initialize_(preSched, trgtLngth, SolverID, ScheduleRoot);

  if (fsbl == false) {
    return false;
  }

  costChkCnt_ = 0;
  costPruneCnt_ = 0;
  return true;
}
/*****************************************************************************/

/*****************************************************************************/

FUNC_RESULT LengthCostEnumerator::FindFeasibleSchedule(InstSchedule *sched,
                                                       InstCount trgtLngth,
                                                       BBThread *bbt,
                                                       int costLwrBound,
                                                       Milliseconds deadline) {
  
  bbt_ = bbt;
  //costLwrBound_ = costLwrBound;

  //if (bbt_->isWorker()) {
  //  nodeAlctr_->setBlockLock(bbt_->getAllocatorLock());
  //  hashTblEntryAlctr_->setBlockLock(bbt_->getAllocatorLock());
  //  histNodeAlctr_->setBlockLock(bbt_->getAllocatorLock());
  //}

  //Logger::Info("before enumerator FFS, entryCnt %d", getHistTableEntryCnt());
  FUNC_RESULT rslt = FindFeasibleSchedule_(sched, trgtLngth, deadline);
  //Logger::Info("after enumerator FFS, entryCnt %d", getHistTableEntryCnt());

#ifdef IS_DEBUG_TRACE_ENUM
  stats::costChecksPerLength.Record(costChkCnt_);
  stats::costPruningsPerLength.Record(costPruneCnt_);
  stats::feasibleSchedulesPerLength.Record(fsblSchedCnt_);
  stats::improvementsPerLength.Record(imprvmntCnt_);
#endif

  //printInfsbltyHits();
  //printProbeTiming();

  #ifdef IS_DEBUG_METADATA
  Logger::Info("finished enumeration");
  printMetadata();
  #endif

  return rslt;
}
/*****************************************************************************/

bool LengthCostEnumerator::WasObjctvMet_() {
  #ifdef IS_DEBUG_METADATA
  Milliseconds startTime = Utilities::GetProcessorTime();
  #endif
  assert(GetBestCost_() >= 0);

  //Logger::Info("was objctv met: schedulInstCount %d, tot isntCount %d", schduldInstCnt_, totInstCnt_);
  if (WasSolnFound_() == false) {
    #ifdef IS_DEBUG_METADATA
    checkSolnTime += Utilities::GetProcessorTime() - startTime;
    #endif
    return false;
  }

  //Logger::Info("sln found");
  InstCount crntCost = GetBestCost_();
  //Logger::Info("crntCost = %d", crntCost);
  InstCount newCost = bbt_->UpdtOptmlSched(crntSched_, this);
  //Logger::Info("newCost %d", newCost);
  assert(newCost <= GetBestCost_());

  if (newCost < crntCost) {
    imprvmntCnt_++;
    if (bbt_->isWorker()) 
      bbt_->incrementImprvmntCnt();
  }

  #ifdef IS_DEBUG_METADATA
  checkSolnTime += Utilities::GetProcessorTime() - startTime;
  #endif

  if (newCost == costLwrBound_) Logger::Info("objctv met");
  return newCost == costLwrBound_;
}
/*****************************************************************************/

bool LengthCostEnumerator::ProbeBranch_(SchedInstruction *inst,
                                        EnumTreeNode *&newNode,
                                        bool &isNodeDmntd, bool &isRlxInfsbl,
                                        bool &isLngthFsbl) {

  #ifdef IS_DEBUG_METADATA                                          
  Milliseconds startTime = Utilities::GetProcessorTime();
  #endif

  bool isFsbl = true;

  isFsbl = Enumerator::ProbeBranch_(inst, newNode, isNodeDmntd, isRlxInfsbl,
                                    isLngthFsbl);
  
  assert(newNode || !isFsbl);

  if (isFsbl == false) {
    assert(isLngthFsbl == false);
    isLngthFsbl = false;

    #ifdef IS_DEBUG_METADATA
    probeTime += Utilities::GetProcessorTime() - startTime;
    #endif

    return false;
  }

  isLngthFsbl = true;

  isFsbl = ChkCostFsblty_(inst, newNode);


  if (isFsbl == false) {
#ifdef IS_DEBUG_SEARCH_ORDER
    Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: cost fail");
#endif
    #ifdef IS_DEBUG_METADATA
    probeTime += Utilities::GetProcessorTime() - startTime; 
    #endif
    return false;
  }

  if (IsHistDom()) {
//#ifdef IS_DEBUG_SEARCH_ORDER
//    Logger::Info("Solver %d IN LCE HIST DOM", SolverID_);
//#endif
    assert(newNode);
    EnumTreeNode *parent = newNode->GetParent();
    if (WasDmnntSubProbExmnd_(inst, newNode)) {
#ifdef IS_DEBUG_FLOW
      Logger::Info("History domination\n\n");
#endif

#ifdef IS_DEBUG_INFSBLTY_TESTS
      stats::historyDominationInfeasibilityHits++;
#endif
  histDomInfsbl++;
      bbt_->UnschdulInstBBThread(inst, crntCycleNum_, crntSlotNum_, parent);
#ifdef IS_DEBUG_SEARCH_ORDER
      Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: LCE history fail");
#endif
#ifdef IS_DEBUG_METADATA
      probeTime += Utilities::GetProcessorTime() - startTime;
#endif
      isNodeDmntd = true;
      return false;
    }
  }

  assert(newNode);
  #ifdef IS_DEBUG_METADATA
  probeTime += Utilities::GetProcessorTime() - startTime;
  #endif
  return true;
}
/*****************************************************************************/

bool LengthCostEnumerator::ChkCostFsblty_(SchedInstruction *inst,
                                          EnumTreeNode *&newNode,
                                          bool trueState) {
  bool isFsbl = true;

  costChkCnt_++;

  bbt_->SchdulInstBBThread(inst, crntCycleNum_, crntSlotNum_, false);

  if (prune_.spillCost) {
    isFsbl = bbt_->ChkCostFsblty(trgtSchedLngth_, newNode, !trueState);

    if (!isFsbl && trueState) {
      stats::costInfeasibilityHits++;
#ifdef IS_DEBUG_FLOW
      Logger::Info("Detected cost infeasibility of inst %d in cycle %d",
                   inst == NULL ? -2 : inst->GetNum(), crntCycleNum_);
#endif
  costInfsbl++;
      bbt_->UnschdulInstBBThread(inst, crntCycleNum_, crntSlotNum_,
                         newNode->GetParent());
    }
  }

  return isFsbl;
}
/*****************************************************************************/

bool LengthCostEnumerator::BackTrack_(bool trueState) {
  
  //Logger::Info("SolverID %d in LCE Backtrack", SolverID_);
  #ifdef IS_DEBUG_METADATA
  Milliseconds startTime;
  startTime = Utilities::GetProcessorTime();
  #endif
  SchedInstruction *inst = crntNode_->GetInst();

  bbt_->UnschdulInstBBThread(inst, crntCycleNum_, crntSlotNum_, crntNode_->GetParent());

  bool fsbl = Enumerator::BackTrack_(trueState);

  if (trueState) {
    if (prune_.spillCost) {
      if (fsbl) {  
        assert(crntNode_->GetCostLwrBound() >= 0 || inst == rootNode_->GetInst());
        fsbl = crntNode_->GetCostLwrBound() < GetBestCost_();
      }
    }
  }

if (bbt_->isWorkStealOn()) {
  // it is possible that a crntNode becomes infeasible before exploring all its children
  // thus we need to ensure that all children are removed on backtrack
  if (bbt_->isWorker() && !fsbl) {
    bbt_->localPoolLock(SolverID_ - 2);
    if (bbt_->getLocalPoolSize(SolverID_ - 2) > 0) {
      //Logger::Info("SolverID %d checking its own local pool", SolverID_);

      EnumTreeNode *popNode = bbt_->localPoolPopFront(SolverID_ - 2);
      assert(popNode);
      //Logger::Info("popNode has time %d, prevNode has time %d", popNode->GetTime(), prevNode->GetTime());
      assert(popNode->GetTime() <= (crntNode_->GetTime() + 1));

      while (popNode->GetTime() == (crntNode_->GetTime() + 1)) {
#ifdef IS_CORRECT_LOCALPOOL
        Logger::Info("SolverID %d removed element from localPool at time %d", SolverID_, popNode->GetTime());
#endif
        assert(popNode->GetParent() == crntNode_);
        nodeAlctr_->Free(popNode);
        if (bbt_->getLocalPoolSize(SolverID_ - 2) == 0) break;
        popNode = bbt_->localPoolPopFront(SolverID_ - 2);
      }

      if (popNode->GetTime() != (crntNode_->GetTime() + 1)) {
        bbt_->localPoolPushFront(SolverID_- 2,popNode);
      }
    }
    bbt_->localPoolUnlock(SolverID_ - 2);
  }
}

  #ifdef IS_DEBUG_METADATA
  backtrackTime += Utilities::GetProcessorTime() - startTime;
  #endif
  return fsbl;
}
/*****************************************************************************/

void Enumerator::BackTrackRoot_() {

  //TODO JEFF we should be inserting into history here if we insert on backtrack

  if (bbt_->isWorkStealOn()) {
  // it is possible that a crntNode becomes infeasible before exploring all its children
  // thus we need to ensure that all children are removed on backtrack
    bbt_->localPoolLock(SolverID_ - 2);
    if (bbt_->getLocalPoolSize(SolverID_ - 2) > 0) {
      //Logger::Info("SolverID %d checking its own local pool", SolverID_);

      EnumTreeNode *popNode = bbt_->localPoolPopFront(SolverID_ - 2);
      assert(popNode);
      //Logger::Info("popNode has time %d, prevNode has time %d", popNode->GetTime(), prevNode->GetTime());
      assert(popNode->GetTime() <= (crntNode_->GetTime() + 1));

      while (popNode->GetTime() == (crntNode_->GetTime() + 1)) {
#ifdef IS_CORRECT_LOCALPOOL
        Logger::Info("SolverID %d removed element from localPool at time %d", SolverID_, popNode->GetTime());
#endif
        assert(popNode->GetParent() == crntNode_);
        nodeAlctr_->Free(popNode);
        if (bbt_->getLocalPoolSize(SolverID_ - 2) == 0) break;
        popNode = bbt_->localPoolPopFront(SolverID_ - 2);
      }

      if (popNode->GetTime() != (crntNode_->GetTime() + 1)) {
        bbt_->localPoolPushFront(SolverID_- 2,popNode);
      }
    }
    bbt_->localPoolUnlock(SolverID_ - 2);
  }
}

InstCount LengthCostEnumerator::GetBestCost_() { return bbt_->getBestCost(); }
/*****************************************************************************/

void LengthCostEnumerator::CreateRootNode_() {
  //Logger::Info("creating LCE root");
#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorLock();
#endif
  rootNode_ = nodeAlctr_->Alloc(NULL, NULL, this);
#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorUnlock();
#endif
  CreateNewRdyLst_();
  rootNode_->SetRdyLst(rdyLst_);
  rootNode_->SetLwrBounds(DIR_FRWRD);

  assert(rsrvSlotCnt_ == 0);
  rootNode_->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);

  bbt_->setSttcLwrBounds(rootNode_);

  rootNode_->SetCost(0);
  rootNode_->SetCostLwrBound(0);

  InitNewNode_(rootNode_);
  CmtLwrBoundTightnng_();

  //Logger::Info("in create root, rootNode has instNum %d", rootNode_->GetInstNum());

  //Logger::Info("rootNode_->GetCost() in create root %d", rootNode_->GetCost());
  //Logger::Info("solverID is %d", SolverID_);
}
/*****************************************************************************/
/*
void LengthCostEnumerator::createWorkerRootNode_()
{
  crntNode_ = nodeAlctr_->Alloc(NULL, NULL, this);
  CreateNewRdyLst_();
  crntNode_->SetRdyLst(rdyLst_);
  crntNode_->SetLwrBounds(DIR_FRWRD);

  assert(rsrvSlotCnt_ == 0);
  crntNode_->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);

  bbt_->setSttcLwrBounds(crntNode_);

  crntNode_->SetCost(0);
  crntNode_->SetCostLwrBound(0);

  // InitNewNode_(rootNode_) innards

  crntNode_->SetCrntCycleBlkd(isCrntCycleBlkd_);
  crntNode_->SetRealSlotNum(crntRealSlotNum_);

  if (IsHistDom()) {
    crntNode_->CreateHistory();
    assert(crntNode_->GetHistory() != tmpHstryNode_);
  }

  crntNode_->SetSlotAvlblty(avlblSlots_, avlblSlotsInCrntCycle_);

  // not sure if we need this
  // UpdtRdyLst_(crntCycleNum_, crntSlotNum_) innards
  //if (prirts_.isDynmc)
  //  rdyLst_->UpdatePriorities();


  bool isLeaf = schduldInstCnt_ == totInstCnt_;

  crntNode_->SetBranchCnt(rdyLst_->GetInstCnt(), isLeaf);

  createdNodeCnt_++;
  crntNode_->SetNum(createdNodeCnt_);



  CmtLwrBoundTightnng_();
}
*/
/*****************************************************************************/

void LengthCostEnumerator::scheduleInt(int instNum, EnumTreeNode *newNode, bool isPseudoRoot, bool prune)
{
  // what about for the isPseudoRoot case

  //Logger::Info("in scheduleNode, inst %d", node->GetInstNum());
  //scheduleInst_(node->GetInst(), isPseudoRoot);

  newNode = NULL;

  //Logger::Info("attempting to schedule inst %d", instNum);


 
  //PROBEBRANCH
  //out = ProbeBranch_(node->GetInst(), newNode, nodeDom, rlxInfsbl, lngthInfsbl, prune);
 
  SchedInstruction *inst;
  int rdyListSize = rdyLst_->GetInstCnt();

  rdyLst_->ResetIterator();
  for (int i = 0; i < rdyListSize; i++) {
    SchedInstruction *temp = rdyLst_->GetNextPriorityInst();
    if (temp->GetNum() == instNum) {
      inst = temp;
      break;
    }
  }
  rdyLst_->ResetIterator();

  assert(IsStateClear_());
  assert(inst == NULL || inst->IsSchduld(SolverID_) == false);


  if (inst != NULL) {
    inst->Schedule(crntCycleNum_, crntSlotNum_, SolverID_);
    DoRsrvSlots_(inst);
    state_.instSchduld = true;
  }


  ProbeIssuSlotFsblty_(inst);
  state_.issuSlotsProbed = true;


  if (bbt_->isSecondPass()) {
    TightnLwrBounds_(inst, false);
    state_.lwrBoundsTightnd = true;
  }

#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorLock();
#endif
  newNode = nodeAlctr_->Alloc(crntNode_, inst, this);
#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorUnlock();
#endif
  newNode->SetLwrBounds(DIR_FRWRD);
  newNode->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);

  assert(newNode);

  ChkCostFsblty_(inst, newNode, false);
  //END OF PROBEBRANCH


  //STEP FRWRD
  SchedInstruction *instToSchdul = inst;
  InstCount instNumToSchdul;

  assert(newNode);
  CreateNewRdyLst_();
  // Let the new node inherit its parent's ready list before we update it
  newNode->SetRdyLst(rdyLst_);

  if (instToSchdul == NULL) {
    instNumToSchdul = SCHD_STALL;
  } else {
    instNumToSchdul = instToSchdul->GetNum();
    SchdulInst_(instToSchdul, crntCycleNum_);
    
    /*Logger::Info("attempting to remove %d from readyLst", instToSchdul->GetNum());
    Logger::Info("readyLst contains");
    printRdyLst();
    SchedInstruction *tempInst = rdyLst_->GetNextPriorityInst();
    Logger::Info("iterating on %d in rdyLst", tempInst->GetNum());
    while (tempInst->GetNum() != instToSchdul->GetNum()) {
      tempInst = rdyLst_->GetNextPriorityInst();
      Logger::Info("iterating on %d in rdyLst", tempInst->GetNum());
    }
    rdyLst_->RemoveNextPriorityInst();
    rdyLst_->ResetIterator();*/
    
    if (instToSchdul->GetTplgclOrdr() == minUnschduldTplgclOrdr_) {
      minUnschduldTplgclOrdr_++;
    }
  }

  crntSched_->AppendInst(instNumToSchdul);

  MovToNxtSlot_(instToSchdul);
  assert(crntCycleNum_ <= trgtSchedLngth_);

  if (crntSlotNum_ == 0) {
    InitNewCycle_();
  }

  InitNewGlobalPoolNode_(newNode);

  CmtLwrBoundTightnng_();
  ClearState_();

  if (isPseudoRoot)
    rootNode_ = newNode;
}

/*****************************************************************************/


void LengthCostEnumerator::scheduleNode(EnumTreeNode *node, bool isPseudoRoot, bool prune)
{
  // what about for the isPseudoRoot case

  //Logger::Info("in scheduleNode, inst %d", node->GetInstNum());
  //scheduleInst_(node->GetInst(), isPseudoRoot);


  //if (node->GetInst())
  //  Logger::Info("attempting to schedule inst %d", node->GetInstNum());
  //else
  //  Logger::Info("attempting to schedule null inst!!!!");
  EnumTreeNode *newNode = NULL;

 
  //PROBEBRANCH
  //out = ProbeBranch_(node->GetInst(), newNode, nodeDom, rlxInfsbl, lngthInfsbl, prune);
 
  SchedInstruction *inst = node->GetInst();
  assert(IsStateClear_());
  assert(inst == NULL || inst->IsSchduld(SolverID_) == false);


  if (inst != NULL) {
    inst->Schedule(crntCycleNum_, crntSlotNum_, SolverID_);
    DoRsrvSlots_(inst);
    state_.instSchduld = true;
  }


  ProbeIssuSlotFsblty_(inst, false);
  state_.issuSlotsProbed = true;

  if (bbt_->isSecondPass()) {
    TightnLwrBounds_(inst, false);
    state_.lwrBoundsTightnd = true;
  }

#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorLock();
#endif
  newNode = nodeAlctr_->Alloc(crntNode_, inst, this, false);
#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorUnlock();
#endif
  newNode->SetLwrBounds(DIR_FRWRD);
  newNode->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);

  assert(newNode);

  ChkCostFsblty_(inst, newNode, false);
  //END OF PROBEBRANCH


  //STEP FRWRD
  SchedInstruction *instToSchdul = node->GetInst();
  InstCount instNumToSchdul;

  assert(newNode);
  CreateNewRdyLst_();
  // Let the new node inherit its parent's ready list before we update it
  newNode->SetRdyLst(rdyLst_);

  if (instToSchdul == NULL) {
    instNumToSchdul = SCHD_STALL;
  } else {
    instNumToSchdul = instToSchdul->GetNum();
    SchdulInst_(instToSchdul, crntCycleNum_);
    
    /*Logger::Info("attempting to remove %d from readyLst", instToSchdul->GetNum());
    Logger::Info("readyLst contains");
    printRdyLst();
    SchedInstruction *tempInst = rdyLst_->GetNextPriorityInst();
    Logger::Info("iterating on %d in rdyLst", tempInst->GetNum());
    while (tempInst->GetNum() != instToSchdul->GetNum()) {
      tempInst = rdyLst_->GetNextPriorityInst();
      Logger::Info("iterating on %d in rdyLst", tempInst->GetNum());
    }
    rdyLst_->RemoveNextPriorityInst();
    rdyLst_->ResetIterator();*/
    
    if (instToSchdul->GetTplgclOrdr() == minUnschduldTplgclOrdr_) {
      minUnschduldTplgclOrdr_++;
    }
  }

  crntSched_->AppendInst(instNumToSchdul);

  MovToNxtSlot_(instToSchdul);
  assert(crntCycleNum_ <= trgtSchedLngth_);

  if (crntSlotNum_ == 0) {
    InitNewCycle_();
  }

  InitNewGlobalPoolNode_(newNode);

  CmtLwrBoundTightnng_();
  ClearState_();

  if (isPseudoRoot)
    rootNode_ = newNode;
}

bool LengthCostEnumerator::scheduleNodeOrPrune(EnumTreeNode *node,
                                               bool isPseudoRoot) {
  //Logger::Info("beginning enum schedNodeOrPrune, entryCnt %d", getHistTableEntryCnt());

  // shculeding function for state generation


  InstCount i;
  bool isEmptyNode;
  SchedInstruction *inst;
  bool isFsbl = true;
  InstCount brnchCnt = node->GetBranchCnt(isEmptyNode);
  //InstCount crntBrnchNum = node->GetCrntBranchNum();

  // iterate until we find the node
  rdyLst_->ResetIterator();

  // TODO(JEFF) 6/28
  // changed for (i = crntBrnchNum) to for (i = 0) -- to test work stealing

  //Logger::Info("SolverID %d rdyLst has size %d", SolverID_, rdyLst_->GetInstCnt());
  for (i = 0; i < brnchCnt + 1 && node->IsFeasible(); i++) {
    
    //assert(i != brnchCnt - 1);
    //if (i == brnchCnt - 1) Logger::Info("SolverID %d, ALERT i == brnchCnt - 1", SolverID_);
    //if (i == brnchCnt && i != 0) Logger::Info("SolverID %d, DOUBLE ALERT", SolverID_);
    inst = rdyLst_->GetNextPriorityInst();
    //Logger::Info("SolverID_ %d, checking %dth inst (num %d) in rdyLst to find match (against %d)", SolverID_, i, inst->GetNum(), node->GetInstNum());
    if (inst->GetNum() == node->GetInstNum()) {
      //Logger::Info("SolverID_ %d , MATCH (num %d) (against %d)", SolverID_, inst->GetNum(), node->GetInstNum());
      // schedule its instruction
      //Logger::Info("SolverID %d attempting to schedule inst #%d", SolverID_, inst->GetNum());


      //if (!bbt_->isWorker() || SolverID_ == 3)
      //  Logger::Info("attempting to schedule inst %d", inst->GetNum());
      scheduleInst_(inst, isPseudoRoot, isFsbl);
      if (!isFsbl) {
        //nodeAlctr_->Free(node);
        //Logger::Info("ending enum schedNodeOrPrune, entryCnt %d", getHistTableEntryCnt());
        return false;
      }
      break;
    }
  }
  rdyLst_->ResetIterator();
  //Logger::Info("ending enum schedNodeOrPrune, entryCnt %d", getHistTableEntryCnt());
  return true;
  
  // nodes examined? if (fsbl) 
}
/*****************************************************************************/
bool LengthCostEnumerator::scheduleIntOrPrune(int instToSchdul,
                                               bool isPseudoRoot) {
  //Logger::Info("beginning enum schedNodeOrPrune, entryCnt %d", getHistTableEntryCnt());

  // shculeding function for state generation

#ifdef IS_DEBUG_SEARCH_ORDER
    Logger::Log((Logger::LOG_LEVEL)4, false, "SolverID %d probing global pool inst %d", SolverID_, instToSchdul);
#endif

  InstCount i;
  //bool isEmptyNode;
  SchedInstruction *inst;
  bool isFsbl = true;
  InstCount brnchCnt = rdyLst_->GetInstCnt();
  bool flag = false;

  // iterate until we find the node
  rdyLst_->ResetIterator();

  // TODO(JEFF) 6/28
  // changed for (i = crntBrnchNum) to for (i = 0) -- to test work stealing

  for (i = 0; i < brnchCnt; i++) {
    inst = rdyLst_->GetNextPriorityInst();
    if (inst->GetNum() == instToSchdul) {
      flag = true;
      // schedule its instruction
      //Logger::Info("SolverID %d attempting to schedule inst #%d", SolverID_, inst->GetNum());


      //if (!bbt_->isWorker() || SolverID_ == 3)
      //  Logger::Info("attempting to schedule inst %d", inst->GetNum());
      scheduleInst_(inst, isPseudoRoot, isFsbl);
      rdyLst_->ResetIterator();
      if (!isFsbl) {
        //nodeAlctr_->Free(node);
        //Logger::Info("ending enum schedNodeOrPrune, entryCnt %d", getHistTableEntryCnt());
        return false;
      }
      break;
    }
  }
  //Logger::Info("ending enum schedNodeOrPrune, entryCnt %d", getHistTableEntryCnt());

  assert(flag);

#ifdef IS_DEBUG_SEARCH_ORDER
    Logger::Log((Logger::LOG_LEVEL)4, false, "SolverID %d stepping forward global pool inst %d", SolverID_, instToSchdul);
#endif


  return true;
  
  // nodes examined? if (fsbl) 
}
/********************************************************************************/




bool LengthCostEnumerator::isFsbl(EnumTreeNode *node, bool checkHistory) {
  
  
  // This doesnt work with global pool cost
  /*
  if (node->GetCost() >= GetBestCost()) {
#ifdef IS_DEBUG_SEARCH_ORDER
    Logger::Info("GlobalPoolNode %d cost infeasible", node->GetInstNum());
#endif
      costInfsbl++;
    //nodeAlctr_->Free(node);
    return false;
  }
  */

  /*
  if (IsHistDom() && checkHistory) {
    assert(node != NULL);
    if (WasDmnntSubProbExmnd_(node->GetInst(), node)) {
#ifdef IS_DEBUG_FLOW
      Logger::Info("History domination\n\n");
#endif

#ifdef IS_DEBUG_INFSBLTY_TESTS
      stats::historyDominationInfeasibilityHits++;
#endif
  histDomInfsbl++;
#ifdef IS_DEBUG_SEARCH_ORDER
      Logger::Log((Logger::LOG_LEVEL) 4, false, "GlobalPoolNode %d LCE history fail", node->GetInstNum());
#endif
      //nodeAlctr_->Free(node);
      return false;
    }
  }*/

  return true;

  /*
  bool NodeFsbl = true;
  bool fsbl;
  
  SchedInstruction *inst = node->GetInst();
  Logger::Info("checking fsblty for inst %d", inst->GetNum());
  EnumTreeNode *newNode = NULL;

  assert(IsStateClear_());
  assert(inst == NULL || inst->IsSchduld(SolverID_) == false);

#ifdef IS_DEBUG_FLOW
  InstCount instNum = inst == NULL ? -2 : inst->GetNum();
  Logger::Info("Probing inst %d in cycle %d / slot %d", instNum, crntCycleNum_,
               crntSlotNum_);
#endif


  // If this instruction is prefixed, it cannot be scheduled earlier than its
  // prefixed cycle
  if (inst != NULL)
    if (inst->GetPreFxdCycle() != INVALID_VALUE)
      if (inst->GetPreFxdCycle() != crntCycleNum_) {
#ifdef IS_DEBUG_SEARCH_ORDER
        Logger::Info("probe: prefix fail");
#endif
        NodeFsbl = false;
      }

  if (inst != NULL) {
    if (inst->GetCrntLwrBound(DIR_FRWRD, SolverID_) > crntCycleNum_) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
      stats::forwardLBInfeasibilityHits++;
#endif
#ifdef IS_DEBUG_SEARCH_ORDER
      Logger::Info("probe: LB fail");
#endif
      NodeFsbl = false;
    }

    if (inst->GetCrntDeadline(SolverID_) < crntCycleNum_) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
      stats::backwardLBInfeasibilityHits++;
#endif
#ifdef IS_DEBUG_SEARCH_ORDER
      Logger::Info("probe: deadline fail");
#endif
      NodeFsbl = false;
    }
  }

  // If we are scheduling for register pressure only, and this branch
  // defines a register but does not use any, we can prune this branch
  // if another instruction in the ready list does use a register.
  if (SchedForRPOnly_) {
    if (inst != NULL && crntNode_->FoundInstWithUse() &&
        inst->GetAdjustedUseCnt() == 0 && !dataDepGraph_->DoesFeedUser(inst))
      NodeFsbl = false;
  }

  if (prune_.nodeSup) {
    if (inst != NULL)
      if (crntNode_->WasSprirNodeExmnd(inst)) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
        stats::nodeSuperiorityInfeasibilityHits++;
#endif
#ifdef IS_DEBUG_SEARCH_ORDER
        Logger::Info("probe: history fail");
#endif
        NodeFsbl = false;
      }
  }

  if (inst != NULL) {
    inst->Schedule(crntCycleNum_, crntSlotNum_, SolverID_);
    DoRsrvSlots_(inst);
    state_.instSchduld = true;
  }

  fsbl = ProbeIssuSlotFsblty_(inst);
  state_.issuSlotsProbed = true;

  if (!fsbl) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
    stats::slotCountInfeasibilityHits++;
#endif
#ifdef IS_DEBUG_SEARCH_ORDER
    Logger::Info("probe: issue slot fail");
#endif
    NodeFsbl = false;
  }

  Logger::Info("from within isFsbl()");
  fsbl = TightnLwrBounds_(inst);
  state_.lwrBoundsTightnd = true;

  if (fsbl == false) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
    stats::rangeTighteningInfeasibilityHits++;
#endif
#ifdef IS_DEBUG_SEARCH_ORDER
    Logger::Info("probe: tightn LB fail");
#endif
    NodeFsbl = false;
  }

  state_.instFxd = true;

  newNode = nodeAlctr_->Alloc(crntNode_, inst, this);
  newNode->SetLwrBounds(DIR_FRWRD);
  newNode->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);

  // If a node (sub-problem) that dominates the candidate node (sub-problem)
  // has been examined already and found infeasible

  if (prune_.histDom) {
    if (isEarlySubProbDom_)
      if (WasDmnntSubProbExmnd_(inst, newNode)) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
        stats::historyDominationInfeasibilityHits++;
#endif
#ifdef IS_DEBUG_SEARCH_ORDER
        Logger::Info("probe: histDom fail");
#endif
        NodeFsbl = false;
      }
  }


  // Try to find a relaxed schedule for the unscheduled instructions
  if (prune_.rlxd) {
    fsbl = RlxdSchdul_(newNode);
    state_.rlxSchduld = true;


    if (fsbl == false) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
      stats::relaxedSchedulingInfeasibilityHits++;
#endif
#ifdef IS_DEBUG_SEARCH_ORDER
      Logger::Info("probe: relaxed fail");
#endif
      NodeFsbl = false;
    }
  }

  assert(newNode != NULL);
  

  fsbl = ChkCostFsblty_(inst, newNode);

  if (fsbl == false) {
#ifdef IS_DEBUG_SEARCH_ORDER
    Logger::Info("probe: cost fail");
#endif
    NodeFsbl =  false;
  }

  if (IsHistDom()) {
    assert(newNode != NULL);
    EnumTreeNode *parent = newNode->GetParent();
    if (WasDmnntSubProbExmnd_(inst, newNode)) {
#ifdef IS_DEBUG_FLOW
      Logger::Info("History domination\n\n");
#endif

#ifdef IS_DEBUG_INFSBLTY_TESTS
      stats::historyDominationInfeasibilityHits++;
#endif
      bbt_->UnschdulInstBBThread(inst, crntCycleNum_, crntSlotNum_, parent);
#ifdef IS_DEBUG_SEARCH_ORDER
      Logger::Info("probe: LCE history fail");
#endif
      NodeFsbl =  false;
    }
  }

  RestoreCrntState_(inst, newNode);

  return NodeFsbl;
  */
}
/****************************************/

EnumTreeNode *LengthCostEnumerator::scheduleInst_(SchedInstruction *inst, bool isPseudoRoot, bool &isFsbl, bool isRoot, bool prune) {
    // schedule the instruction (e.g. use probeBranch innareds to update state)

  EnumTreeNode *newNode;

  bool isNodeDominated = false, isRlxdFsbl = true, isLngthFsbl = true;
  isFsbl = ProbeBranch_(inst, newNode, isNodeDominated, isRlxdFsbl, isLngthFsbl);

#ifdef DEBUG_GP_HISTORY
  if (isNodeDominated)
    Logger::Info("SolverID %d GlobalPool Node history dominated", SolverID_);
#endif
  if (!isFsbl)
    return nullptr;

  /*
  if (inst != NULL) {
    inst->Schedule(crntCycleNum_, crntSlotNum_, SolverID_);
    DoRsrvSlots_(inst);
    state_.instSchduld = true;
  }

  ProbeIssuSlotFsblty_(inst);
  state_.issuSlotsProbed = true;

  TightnLwrBounds_(inst);
  state_.lwrBoundsTightnd = true;
  state_.instFxd = true;

#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorLock();
#endif
  newNode = nodeAlctr_->Alloc(crntNode_, inst, this);
#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorUnlock();
#endif
  newNode->SetLwrBounds(DIR_FRWRD);
  newNode->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);

  // Try to find a relaxed schedule for the unscheduled instructions
  if (prune_.rlxd) {
    RlxdSchdul_(newNode);
    state_.rlxSchduld = true;
  }
  
  //potentially will be refactored
  bbt_->SchdulInstBBThread(inst, crntCycleNum_, crntSlotNum_, false);
  //Logger::Info("about to chkCostFsblty for inst %d", inst->GetNum());
  
  bool costFsbl = bbt_->ChkCostFsblty(trgtSchedLngth_, newNode);

  if (isFsbl != nullptr)
    *isFsbl = costFsbl;*/

  //START OF STEPFRWRD
  InstCount instNumToSchdul;

  CreateNewRdyLst_();
  assert(newNode);
  newNode->SetRdyLst(rdyLst_);

  instNumToSchdul = inst->GetNum();
  SchdulInst_(inst, crntCycleNum_);

  int rdyLstSize = rdyLst_->GetInstCnt();
  rdyLst_->ResetIterator();
  for (int i = 0; i < rdyLstSize; i++) {
    SchedInstruction *temp = rdyLst_->GetNextPriorityInst();
    if (temp->GetNum() == instNumToSchdul) {
      break;
    }
  }
    
  rdyLst_->RemoveNextPriorityInst();
  rdyLst_->ResetIterator();

  if (inst->GetTplgclOrdr() == minUnschduldTplgclOrdr_) {
    minUnschduldTplgclOrdr_++;
  }

  //if (!isPseudoRoot)
  crntSched_->AppendInst(instNumToSchdul);

  MovToNxtSlot_(inst);
  assert(crntCycleNum_ <= trgtSchedLngth_);

  if (crntSlotNum_ == 0) {
    InitNewCycle_();
  }

  // stepFrwrd calls initNewNode which updates the insts in readyList
  //Logger::Info("initializing new node for inst %d", inst->GetNum());
  InitNewNode_(newNode);

#ifdef INSERT_ON_STEPFRWRD
  if (!isSecondPass()) {
    if (IsHistDom()) {
      assert(!crntNode_->IsArchived());
        UDT_HASHVAL key = exmndSubProbs_->HashKey(crntNode_->GetSig());

      if (bbt_->isWorker()) {
        bbt_->histTableLock(key);
          HistEnumTreeNode *crntHstry = crntNode_->GetHistory();
  #ifdef IS_SYNCH_ALLOC
          bbt_->allocatorLock();
  #endif
          exmndSubProbs_->InsertElement(crntNode_->GetSig(), crntHstry,
                                    hashTblEntryAlctr_, bbt_);
  #ifdef IS_SYNCH_ALLOC
          bbt_->allocatorUnlock();
  #endif
          SetTotalCostsAndSuffixes(crntNode_, crntNode_->GetParent(), trgtSchedLngth_,
                              prune_.useSuffixConcatenation);
          crntNode_->Archive();
        bbt_->histTableUnlock(key);
      }

      else {
        HistEnumTreeNode *crntHstry = crntNode_->GetHistory();
        exmndSubProbs_->InsertElement(crntNode_->GetSig(), crntHstry,
                                    hashTblEntryAlctr_, bbt_);
        SetTotalCostsAndSuffixes(crntNode_, crntNode_->GetParent(), trgtSchedLngth_,
                              prune_.useSuffixConcatenation);
        crntNode_->Archive();
      }
        

    } else {
      assert(crntNode_->IsArchived() == false);
    }
  }
#endif

  if (isPseudoRoot) {
    rootNode_ = newNode;
    //Logger::Info("rootNode_ has inst num %d", rootNode_->GetInstNum());
  }

  CmtLwrBoundTightnng_();
  ClearState_();
  

  return newNode;
}
/*****************************************************************************/
bool LengthCostEnumerator::scheduleArtificialRoot(bool setAsRoot)
{
  //Logger::Info("SolverID_ %d Scheduling artificial root", SolverID_);

  SchedInstruction *inst = rdyLst_->GetNextPriorityInst();
  //Logger::Info("SolverID_ %d Scheduling artificial root inst %d", SolverID_, inst->GetNum());
  bool isFsbl = true;

  scheduleInst_(inst, setAsRoot, isFsbl, true, false);
#ifdef INSTPREFIX
  crntNode_->allocateInstPrefix();
#endif

  return isFsbl;



  //old function
  /*
  SchedInstruction *inst = rdyLst_->GetNextPriorityInst();

  if (inst != NULL) {
    inst->Schedule(crntCycleNum_, crntSlotNum_, SolverID_);
    DoRsrvSlots_(inst);
    state_.instSchduld = true;
  }

  ProbeIssuSlotFsblty_(inst);
  state_.issuSlotsProbed = true;

  TightnLwrBounds_(inst);
  state_.lwrBoundsTightnd = true;
  state_.instFxd = true;


#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorLock();
#endif
  EnumTreeNode *newNode = nodeAlctr_->Alloc(crntNode_, inst, this);
#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorUnlock();
#endif
  newNode->SetLwrBounds(DIR_FRWRD);
  newNode->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);

  bbt_->SchdulInstBBThread(inst, crntCycleNum_, crntSlotNum_, false);
  bbt_->ChkCostFsblty(trgtSchedLngth_, newNode);

  SchedInstruction *instToSchdul = newNode->GetInst();
    InstCount instNumToSchdul;
  instNumToSchdul = instToSchdul->GetNum();

  CreateNewRdyLst_();

  newNode->SetRdyLst(rdyLst_);


  SchdulInst_(instToSchdul, crntCycleNum_);

  rdyLst_->RemoveNextPriorityInst();


  if (instToSchdul->GetTplgclOrdr() == minUnschduldTplgclOrdr_) {
    minUnschduldTplgclOrdr_++;
  }

  crntSched_->AppendInst(instNumToSchdul);

  
  MovToNxtSlot_(instToSchdul);
  assert(crntCycleNum_ <= trgtSchedLngth_);

  if (crntSlotNum_ == 0) {
    InitNewCycle_();
  }

  InitNewNode_(newNode);

  CmtLwrBoundTightnng_();
  ClearState_();
  */

}
/*****************************************************************************/
void LengthCostEnumerator::scheduleAndSetAsRoot_(SchedInstruction *rootInst,
                                                 LinkedList<SchedInstruction> *frstList,
                                                 LinkedList<SchedInstruction> *scndList)
{
  assert(rootInst != NULL);
  scheduleArtificialRoot();
  EnumTreeNode *newNode;

  // schedule inst, and get the TreeNode
  //Logger::Info("SolverID %d Scheduling inst #%d from GPQ", SolverID_, rootInst->GetNum());
  //newNode = scheduleInst_(rootInst, frstList, scndList);

  // set the root node
  rootNode_ = newNode;
  crntNode_ = rootNode_;
}

/*****************************************************************************/
EnumTreeNode *LengthCostEnumerator::checkTreeFsblty(bool &fsbl) {
  assert(rootNode_ != NULL);

  SchedInstruction *inst = rdyLst_->GetNextPriorityInst();
  // can probably just call scheduleInst
  EnumTreeNode *newNode = scheduleInst_(inst, true, fsbl);
  return newNode;

  /*
  if (inst != NULL) {
    inst->Schedule(crntCycleNum_, crntSlotNum_, SolverID_);
    Logger::Info("scheduling inst %d", inst->GetNum());
    DoRsrvSlots_(inst);
    state_.instSchduld = true;
  }

  ProbeIssuSlotFsblty_(inst);
  state_.issuSlotsProbed = true;


  TightnLwrBounds_(inst);
  state_.lwrBoundsTightnd = true;

  // schedule the artificial root
  EnumTreeNode *newNode;

#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorLock();
#endif
  newNode = nodeAlctr_->Alloc(crntNode_, inst, this);
#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorUnlock();
#endif



  newNode->SetLwrBounds(DIR_FRWRD);
  newNode->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);

  if (prune_.rlxd) {
    RlxdSchdul_(newNode);
    state_.rlxSchduld = true;
  }
  
  //potentially will be refactored
  bbt_->SchdulInstBBThread(inst, crntCycleNum_, crntSlotNum_, false);
  //Logger::Info("about to chkCostFsblty for inst %d", inst->GetNum());
  *fsbl = bbt_->ChkCostFsblty(trgtSchedLngth_, newNode);

  //Logger::Info("Master artificial root cost %d", newNode->GetCost());
  
  // test code 4/14
  rootNode_ = newNode;

  // Test code 4/26
  rdyLst_->RemoveNextPriorityInst();
  CreateNewRdyLst_();
  // Let the new node inherit its parent's ready list before we update it
  newNode->SetRdyLst(rdyLst_);


  return newNode;
  */

}

/*****************************************************************************/

void LengthCostEnumerator::getAndRemoveInstFromRdyLst(int instNum, SchedInstruction *&inst) {
  int rdyLstSize = rdyLst_->GetInstCnt();
  inst = nullptr;
  for (int i = 0; i < rdyLstSize; i++) {
    SchedInstruction *temp = rdyLst_->GetNextPriorityInst();
    if (temp->GetNum() == instNum) {
      rdyLst_->RemoveNextPriorityInst();
      inst = temp;
      rdyLst_->ResetIterator();
      break;
    }
  }

  assert(inst != nullptr);
}


void LengthCostEnumerator::schedulePrefixInst_(SchedInstruction *instToSchdul, std::stack<InstCount> &costStack) {
  instToSchdul->Schedule(crntCycleNum_, crntSlotNum_, SolverID_);
  bbt_->SchdulInstBBThread(instToSchdul, crntCycleNum_, crntSlotNum_, false);
  costStack.push(bbt_->getCrntPeakSpillCost());
  
  ConstrainedScheduler::SchdulInst_(instToSchdul, crntCycleNum_);

  MovToNxtSlot_(instToSchdul);
  if (crntSlotNum_ == 0) {
    InitNewCycle_();
  }

  UpdtRdyLst_(crntCycleNum_, crntSlotNum_);
}

void LengthCostEnumerator::unschedulePrefixInst_(SchedInstruction *instToUnschdul, std::stack<InstCount> &costStack) {
  InstCount tempCost = costStack.top();
  costStack.pop();
  bbt_->UnschdulInstBBThread2(instToUnschdul, crntCycleNum_, crntSlotNum_, tempCost);
  rdyLst_->RemoveLatestSubList();
  rdyLst_->AddInst(instToUnschdul);
  MovToPrevSlot_(crntSlotNum_);
  ConstrainedScheduler::UnSchdulInst_(instToUnschdul);
  instToUnschdul->UnSchedule(SolverID_);

}


void LengthCostEnumerator::splitNode(HalfNode *&ExploreNode, InstPool4 *fillPool, int depth) {
  //if (ExploreNode != nullptr) Logger::Info("splitting node with inst %d at depth %d", ExploreNode->getPrefix().back(), depth);
  std::queue<int> tempPrefix;
  std::stack<SchedInstruction *> tempStack;
  std::stack<InstCount> costStack;
  SchedInstruction *tempInst = nullptr;
  int tempInstNum;
  ReadyList *originalRdyLst;

  originalRdyLst = new ReadyList(dataDepGraph_, prirts_, SolverID_);
  //Logger::Info("new rdyLst has element count of %d", originalRdyLst->GetInstCnt());
  originalRdyLst->CopyList(rdyLst_);

  int prefixLength = 0;
  
  if (ExploreNode != nullptr) prefixLength = ExploreNode->getPrefixSize();

  //Logger::Info("prefix has length %d", prefixLength);

  for (int i = 0; i < prefixLength; i++) {
    tempInstNum = ExploreNode->getAndRemoveNextPrefixInst();
    //Logger::Info("prefix inst %d", tempInstNum);
    //printRdyLst();
    tempPrefix.push(tempInstNum);
    getAndRemoveInstFromRdyLst(tempInstNum, tempInst);
    schedulePrefixInst_(tempInst, costStack);
    removeInstFromRdyLst_(tempInstNum);
    tempStack.push(tempInst);
  }

  int rdyListSize = rdyLst_->GetInstCnt();


  //Logger::Info("after scheduling the prefix, rdyLst --");
  //printRdyLst();
  for (int i = 0; i < rdyListSize; i++) {
    std::queue<int> tempPrefix2 = tempPrefix;
    unsigned long nextKey;
    SchedInstruction *temp = rdyLst_->GetNextPriorityInst(nextKey);
    unsigned long *heur;
    
    if (bbt_->getGlobalPoolSortMethod() == 1) {
      heur = new unsigned long[depth];
      heur[0] = nextKey;
      unsigned long *prevHeur = nullptr;
      if (ExploreNode != nullptr) prevHeur = ExploreNode->getHeuristic();
      for (int i = 1; i < depth; i++) {
        assert(prevHeur != nullptr);
        //if (i >= 1 && i == depth - 1) 
        //  Logger::Info("Setting heur[%d] to %d", i, ExploreNode->second[i-1]);
        heur[i] = prevHeur[i-1];
      }
    }
    else {
      heur = new unsigned long[1];
      heur[0] = nextKey;
    }

    bbt_->UpdateSpillInfoForSchdul_(temp, false);
    tempPrefix2.push(temp->GetNum());
    //Logger::Info("creating halfnode for inst %d", temp->GetNum());
    //Logger::Info("heur is %d", heur[0]);
    fillPool->push(new HalfNode(tempPrefix2, heur, bbt_->getCrntSpillCost()));
    bbt_->UpdateSpillInfoForUnSchdul_(temp);
  }

  for (int i = 0; i < prefixLength; i++) {
    SchedInstruction *temp = tempStack.top();
    unschedulePrefixInst_(temp, costStack);
    tempStack.pop();
  }

  rdyLst_->Reset();
  rdyLst_->CopyList(originalRdyLst);
  delete originalRdyLst;

}



void LengthCostEnumerator::getRdyListAsNodes(std::pair<EnumTreeNode *, unsigned long *> *ExploreNode, InstPool *pool, int depth) {
  std::stack<EnumTreeNode *> prefix;
  std::queue<EnumTreeNode *> subPrefix; 
  int prefixLength = 0;

  EnumTreeNode *node = ExploreNode->first;

  bool flag = false;
  if (node != rootNode_) {
    flag = true;

   /*
    
    //prefix.push(node);
    //Logger::Info("expanding prefix of exploreNode %d", node->GetNum());
    temp = node->GetParent();
    

    while (temp != rootNode_) {
      Logger::Info("adding %d to prefix", temp->GetInstNum());
      prefix.push(temp);

      //hacker hour
      EnumTreeNode *temp2;
      temp2 = temp->GetParent();
      Logger::Info("temp2->getInst %d", temp2->GetInstNum());
      int i = 0;
      while (temp->GetInstNum() == temp2->GetInstNum() && temp2 != rootNode_) {
        Logger::Info("%dth instruction of correcting prefix", i);
        ++i;
        temp2 = temp2->GetParent();
      }

      temp = temp2;
    }
    prefixLength = prefix.Length();

        int j = 0;
    while (!prefix.empty()) {
      ++j;
      temp = prefix.top();
      subPrefix.push(temp->GetInstNum());
      Logger::Info("scheduling the %dth inst of prefix (inst is %d)", j, temp->GetInstNum());
      prefix.pop();
      //Logger::Info("before scheduling prefix");
      //printRdyLst();
      scheduleNode(temp, false, false);
      removeInstFromRdyLst_(temp->GetInstNum());
      // TODO -- delete node
    }
    scheduleNode(node, false, false);
    Logger::Info("attempting to remove %d from readyLst", node->GetInstNum());
    removeInstFromRdyLst_(node->GetInstNum());
    Logger::Info("scheduled prefix of length %d", prefixLength);
    //Logger::Info("finished scheduling the prefix");
    //printRdyLst();
  }
    */



    prefixLength = node->getPrefixSize();
    //EnumTreeNode *parent = node->GetParent();

    //assert(prefixLength >= 1);

    // pop the root
    //node->getAndRemoveNextPrefixInst();
    if (prefixLength >= 1) {  // then we have insts to schedule
      for (int i = 0; i < prefixLength; i++) {
        EnumTreeNode *temp = node->getAndRemoveNextPrefixInst();
        subPrefix.push(temp);
        //Logger::Info("scheduling the %dth inst of prefix (inst is %d)", i, temp->GetInstNum());
        scheduleNode(temp, false, false);
        removeInstFromRdyLst_(temp->GetInstNum());
        //EnumTreeNode *nextParent = parent->GetParent();
        //nodeAlctr_->Free(parent);
        //parent = nextParent;
      }
    }
    scheduleNode(node, false, false);
    removeInstFromRdyLst_(node->GetInstNum());
    subPrefix.push(node);
    //Logger::Info("scheduled prefix of length %d", prefixLength);
  }



  else {
    //Logger::Info("root readyList");
    //printRdyLst();
  }

  // TODO -- delete stack
  
  std::pair<SchedInstruction *, unsigned long> nxtInst;
  int rdyListSize = rdyLst_->GetInstCnt();

  std::queue<std::pair<SchedInstruction *, unsigned long>> firstInstPool;

  for (int i = 0; i < rdyListSize; i++) {
    unsigned long nextKey;
    SchedInstruction *temp = rdyLst_->GetNextPriorityInst(nextKey);
    //Logger::Info("inst with num %d has priority %d", temp->GetNum(), nextKey);
    firstInstPool.push(std::make_pair(temp, nextKey));
  }

  rdyLst_->ResetIterator();

  //SchedPriorities dummyPris;
  //dummyPris.isDynmc = true;

  //ReadyList *prevList = new ReadyList(dataDepGraph_, dummyPris, SolverID_);;
  //prevList->CopyList(rdyLst_);

  for (int i = 0; i < rdyListSize; i++) {
    nxtInst = firstInstPool.front();
    //Logger::Info("getting node for inst %d with heur %d", nxtInst.first->GetNum(), nxtInst.second);
    //Logger::Info("adding inst %d to diversity pool", nxtInst.first->GetNum());
    for (int j = 0 ; j < rdyListSize; j++) {
      SchedInstruction *removeInst = rdyLst_->GetNextPriorityInst();
      if (removeInst->GetNum() == nxtInst.first->GetNum()) {
        rdyLst_->RemoveNextPriorityInst();
        rdyLst_->ResetIterator();
        break;
      }
    }
    firstInstPool.pop();
    EnumTreeNode *pushNode;
    //Logger::Info("Alloc&Init on %d", nxtInst.first->GetNum());
    //if (node == rootNode_) {Logger::Info("before call to alloc&init"); printRdyLst();}


    unsigned long *heur;
    if (bbt_->getGlobalPoolSortMethod() == 1) {
      heur = new unsigned long[depth];
      heur[0] = nxtInst.second;
      for (int i = 1; i < depth; i++) {
        //if (i >= 1 && i == depth - 1) 
        //  Logger::Info("Setting heur[%d] to %d", i, ExploreNode->second[i-1]);
        heur[i] = ExploreNode->second[i-1];
      }
    }
    else {
      heur = new unsigned long[1];
      heur[0] = nxtInst.second;
    }



    pool->push(std::make_pair(allocAndInitNextNode(nxtInst, node, pushNode, node->GetRdyLst(), subPrefix), heur));
    //if (pushNode) Logger::Info("retreived and pushed node with inst %d", pushNode->GetInstNum());
    //rdyLst_->RemoveNextPriorityInst();
    //Logger::Info("pushing node with inst %d to firstInsts", pushNode->GetInstNum());
    //if (!pushNode) Logger::Info("pushing null node from getRdyListAsNodes");
    //pool->push(std::make_pair(pushNode, nextKey)); 
    //if (node == rootNode_) {Logger::Info("after call to alloc&init"); printRdyLst();}
    rdyLst_->AddInst(nxtInst.first);
  }

  delete[] ExploreNode->second;

  SchedInstruction *inst = node->GetInst();

  if (flag) {
    bbt_->UnschdulInstBBThread(inst, crntCycleNum_, crntSlotNum_, crntNode_->GetParent());

    EnumTreeNode *trgtNode = crntNode_->GetParent();
    crntNode_ = trgtNode;
    rdyLst_->RemoveLatestSubList();
    rdyLst_ = crntNode_->GetRdyLst();
  }

  if (prefixLength >= 1) {

    MovToPrevSlot_(crntNode_->GetRealSlotNum());

    crntNode_->GetSlotAvlblty(avlblSlots_, avlblSlotsInCrntCycle_);
    isCrntCycleBlkd_ = crntNode_->GetCrntCycleBlkd();

    if (inst  != NULL) {
      	IssueType issuType = inst->GetIssueType();
    	  neededSlots_[issuType]++;
    }

    crntSched_->RemoveLastInst();
  }

  //bbt_->UpdateSpillInfoForUnSchdul_(inst);//, crntCycleNum_, crntSlotNum_, crntNode_->GetParent());
  //RestoreCrntLwrBounds_(inst);
  if (state_.lwrBoundsTightnd && bbt_->isSecondPass()) {
    UnTightnLwrBounds_(inst);
  }

  /*
  if (inst != NULL) {
   	// int hitCnt;
   	// assert(rdyLst_->FindInst(inst, hitCnt) && hitCnt == 1);
   	assert(inst->IsInReadyList(SolverID_));
    */
  
  UndoRsrvSlots_(inst);
  UnSchdulInst_(inst);
  inst->UnSchedule(SolverID_);

    /*
   	if (inst->GetTplgclOrdr() == minUnschduldTplgclOrdr_ - 1) {
     		minUnschduldTplgclOrdr_--;
   	}
  }
    */

 for (int i = 0; i < prefixLength; i++)
    BackTrack_(false);
}

/*****************************************************************************/
ReadyList *LengthCostEnumerator::getGlobalPoolList(EnumTreeNode *newNode)
{
  #ifdef IS_DEBUG_SEARCH_ORDER
    Logger::Log((Logger::LOG_LEVEL) 4, false, "Probing inst %d", newNode->GetInstNum());
  #endif
  StepFrwrd_(newNode);

  assert(newNode->GetRdyLst()->GetInstCnt() > 0);

  // test code
  rootNode_ = newNode;

  return newNode->GetRdyLst();
}
/*****************************************************************************/
EnumTreeNode *LengthCostEnumerator::allocTreeNode(EnumTreeNode *Prev, 
                                                  SchedInstruction *Inst, 
                                                  InstCount InstCnt) {


  EnumTreeNode *temp = nodeAlctr_->Alloc(Prev, Inst, nullptr, InstCnt);

  return temp;
}


EnumTreeNode *LengthCostEnumerator::allocAndInitNextNode(std::pair<SchedInstruction *, unsigned long> InstNode, 
                                                EnumTreeNode *Prev, 
                                                EnumTreeNode *InitNode,
                                                ReadyList *prevLst,
                                                std::queue<EnumTreeNode *> subPrefix) {

  // ProbeBranch  -- generate Inst state
  // StepFrwrd    -- generate Node state and init
  // Backtrack    -- undo state generation
  // TODO: 
    /* Backtrack corrupts the state of *prevNode_ in our InitNode, currently
       we only need the state of *rdyLst within InitNode, however, it is good pracitce
       to not have corrupt data. We need to return *prevNode_ back to its previous state
       for the next item in our GlobalPool generation
    */


  // shouldnt do any pruning-- 
  //PROBEBRANCH
  //  ProbeBranch_(InstNode.first, InitNode, nodeDom, rlxInfsbl, lngthInfsbl);
 
  SchedInstruction *inst = InstNode.first;
  assert(IsStateClear_());
  assert(inst == NULL || inst->IsSchduld(SolverID_) == false);


  if (inst != NULL) {
    inst->Schedule(crntCycleNum_, crntSlotNum_, SolverID_);
    DoRsrvSlots_(inst);
    state_.instSchduld = true;
  }


  ProbeIssuSlotFsblty_(inst, false);
  state_.issuSlotsProbed = true;


  if (bbt_->isSecondPass()) {
    TightnLwrBounds_(inst, false);
    state_.lwrBoundsTightnd = true;
  }




  EnumTreeNode *parent;
  if (crntNode_->GetInstNum() == inst->GetNum()) {
    parent = crntNode_->GetParent();
  }

  else {
    parent = crntNode_;
  }

#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorLock();
#endif
  InitNode = nodeAlctr_->Alloc(parent, inst, this, false);
#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorUnlock();
#endif
  InitNode->setPrefix(subPrefix);
  InitNode->setPrevNode(parent);

  InitNode->SetLwrBounds(DIR_FRWRD);
  InitNode->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);

  assert(InitNode);

  ChkCostFsblty_(inst, InitNode, false);
  //END OF PROBEBRANCH

  //StepFrwrd_(InitNode);  
  EnumTreeNode *newNode = InitNode;
  SchedInstruction *instToSchdul = newNode->GetInst();
  InstCount instNumToSchdul;


  CreateNewRdyLst_();
  // Let the new node inherit its parent's ready list before we update it
  newNode->SetRdyLst(rdyLst_);

  if (instToSchdul == NULL) {
    instNumToSchdul = SCHD_STALL;
  } else {
    instNumToSchdul = instToSchdul->GetNum();
    SchdulInst_(instToSchdul, crntCycleNum_);
    
    /*
    Logger::Info("attempting to remove %d from readyLst", instToSchdul->GetNum());
    Logger::Info("readyLst contains");
    printRdyLst();
    SchedInstruction *tempInst = rdyLst_->GetNextPriorityInst();
    Logger::Info("iterating on %d in rdyLst", tempInst->GetNum());
    while (tempInst != instToSchdul) {
      tempInst = rdyLst_->GetNextPriorityInst();
      Logger::Info("iterating on %d in rdyLst", tempInst->GetNum());
    }
    rdyLst_->RemoveNextPriorityInst();
    rdyLst_->ResetIterator();
    */
    if (instToSchdul->GetTplgclOrdr() == minUnschduldTplgclOrdr_) {
      minUnschduldTplgclOrdr_++;
    }
  }

  crntSched_->AppendInst(instNumToSchdul);

  MovToNxtSlot_(instToSchdul);
  //assert(crntCycleNum_ <= trgtSchedLngth_);

  if (crntSlotNum_ == 0) {
    InitNewCycle_();
  }

  InitNewGlobalPoolNode_(newNode);

  CmtLwrBoundTightnng_();
  ClearState_();
 //ENDOFSTEPFRWRD

  InitNode->setPriorityKey(InstNode.second);

  //BackTrack_();
  //SchedInstruction *inst = crntNode_->GetInst();

  bbt_->UnschdulInstBBThread(inst, crntCycleNum_, crntSlotNum_, crntNode_->GetParent());

  EnumTreeNode *trgtNode = crntNode_->GetParent();

  rdyLst_->RemoveLatestSubList();

  EnumTreeNode *prevNode = crntNode_;
  crntNode_ = trgtNode;
  rdyLst_ = crntNode_->GetRdyLst();
  assert(rdyLst_ != NULL);

  MovToPrevSlot_(crntNode_->GetRealSlotNum());

  trgtNode->NewBranchExmnd(inst, true, false, false, crntNode_->IsFeasible(),
                           DIR_BKWRD, prevNode->IsLngthFsbl());


  crntNode_->GetSlotAvlblty(avlblSlots_, avlblSlotsInCrntCycle_);
  isCrntCycleBlkd_ = crntNode_->GetCrntCycleBlkd();

  if (inst != NULL) {
    IssueType issuType = inst->GetIssueType();
    neededSlots_[issuType]++;
  }

  crntSched_->RemoveLastInst();
  RestoreCrntLwrBounds_(inst);

  if (inst != NULL) {
    // int hitCnt;
    // assert(rdyLst_->FindInst(inst, hitCnt) && hitCnt == 1);
    assert(inst->IsInReadyList(SolverID_));

    UndoRsrvSlots_(inst);
    UnSchdulInst_(inst);
    inst->UnSchedule(SolverID_);

    if (inst->GetTplgclOrdr() == minUnschduldTplgclOrdr_ - 1) {
      minUnschduldTplgclOrdr_--;
    }
  }

  backTrackCnt_++;


  /*  
  if (prune_.spillCost) {
    if (fsbl) {    
      assert(crntNode_->GetCostLwrBound() >= 0 || inst == rootNode_->GetInst());
      fsbl = crntNode_->GetCostLwrBound() < GetBestCost_();
    }
  }
  */

  return InitNode;

  //ENDOFBACKTRACK

  /*
  // ProbeBranch  -- generate Inst state
  if (InstNode.first != NULL) {
    InstNode.first->Schedule(crntCycleNum_, crntSlotNum_, SolverID_);
    DoRsrvSlots_(InstNode.first);
    state_.instSchduld = true;
  }
  

  //Logger::Info("AllocAndInit for inst %d", InstNode.first);


  ProbeIssuSlotFsblty_(InstNode.first);
  state_.issuSlotsProbed = true;

  TightnLwrBounds_(InstNode.first);
  state_.lwrBoundsTightnd = true;

  state_.instFxd = true;

  // settting InitNode enumrtr to nullptr
#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorLock();
#endif

  // 4/30 passing the enumerator object to TreeNode could result in 
  // subtle synchronization issues between master and worker
  InitNode = nodeAlctr_->Alloc(Prev, InstNode.first, this, totInstCnt_);
#ifdef IS_SYNCH_ALLOC
  bbt_->allocatorUnlock();
#endif
  
  // these need enumrtr 
  //InitNode->SetLwrBounds(DIR_FRWRD);
  //InitNode->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_); 

  if (prune_.rlxd) {
    RlxdSchdul_(InitNode);
    state_.rlxSchduld = true;

  }

  //potentially will be refactored
  bbt_->SchdulInstBBThread(InstNode.first, crntCycleNum_, crntSlotNum_, false);
  bbt_->ChkCostFsblty(trgtSchedLngth_, InitNode, true); // stores the cost LB


  // StepFrwrd    -- generate Node state and init
  // Let the new node inherit its parent's ready list before we update it
  
  //InitNode->SetRdyLst(new ReadyList(dataDepGraph_, prirts_, SolverID_));
  //InitNode->cpyRdyLst(prevLst);

  // test code
  CreateNewRdyLst_();
  InitNode->SetRdyLst(rdyLst_);
  

  if (!InstNode.first) Logger::Info("scheduling a null isnt");
  SchdulInst_(InstNode.first, crntCycleNum_);
  if (InstNode.first->GetTplgclOrdr() == minUnschduldTplgclOrdr_) {
      minUnschduldTplgclOrdr_++;
  }

  MovToNxtSlot_(InstNode.first);


  if (crntSlotNum_ == 0) {
    InitNewCycle_();
  }
  //Logger::Info("before call to initNode");
  //printRdyLst();
  InitNewGlobalPoolNode_(InitNode);
  //Logger::Info("after call to initNode");
  //printRdyLst();
  InitNode->setPriorityKey(InstNode.second);
  



  CmtLwrBoundTightnng_();
  ClearState_();


  //Logger::Info("\n\nNode has Rdy Lst");
  //InitNode->printRdyLst();


  // Backtrack    -- undo state generation
  bbt_->UnschdulInstBBThread(InstNode.first, crntCycleNum_, crntSlotNum_, crntNode_->GetParent());
  rdyLst_->RemoveLatestSubList();


  crntNode_ = InitNode->GetParent();
  //rdyLst_ = crntNode_->GetRdyLst();
  rdyLst_ = crntNode_->GetRdyLst();

  MovToPrevSlot_(crntNode_->GetRealSlotNum());

  //Logger::Info("from crnt Node %d", crntNode_->getEnumerator()->getIssuTypeCnt());
  //Logger::Info("allocAndInit backtrack");
  //Logger::Info("target node %d", crntNode_->GetInstNum());
  //Logger::Info("before updating slot avlblty\navlblSlots_ for issue type 0: %d", avlblSlots_[0]);
  
  
  //crntNode_->GetSlotAvlblty(avlblSlots_, avlblSlotsInCrntCycle_);
  
  //TESTCODE
  IssueType issuType = InstNode.first->GetIssueType();
  ++avlblSlots_[issuType];
  ++avlblSlotsInCrntCycle_[issuType];
  //bool endOfCycle = crntSlotNum_ == issuRate_ - 1;
  //avlblSlots_[issuType] += endOfCycle ? 



  //Logger::Info("after call to getSlotAvlblty: %d", avlblSlots_[0]);
  isCrntCycleBlkd_ = crntNode_->GetCrntCycleBlkd();

  if (InstNode.first != NULL) {
    IssueType issuType = InstNode.first->GetIssueType();
    neededSlots_[issuType]++;
  }

  RestoreCrntLwrBounds_(InstNode.first);

  if (InstNode.first != NULL) {
    // int hitCnt;
    // assert(rdyLst_->FindInst(inst, hitCnt) && hitCnt == 1);

    UndoRsrvSlots_(InstNode.first);
    UnSchdulInst_(InstNode.first);
    InstNode.first->UnSchedule(SolverID_);

    if (InstNode.first->GetTplgclOrdr() == minUnschduldTplgclOrdr_ - 1) {
      minUnschduldTplgclOrdr_--;
    }
  }

  //if (crntNode_->GetInst()) {
  //  Logger::Info("backtracked to noide with inst %d", crntNode_->GetInstNum());
  //}

  assert(InitNode != NULL);
  return InitNode;
  */

}


void LengthCostEnumerator::appendToRdyLst(LinkedList<SchedInstruction> *lst)
{
  rdyLst_->AddList(lst);
}
/*****************************************************************************/

void LengthCostEnumerator::setRootRdyLst()
{
  rootNode_->SetRdyLst(rdyLst_);
}
/*****************************************************************************/

bool LengthCostEnumerator::EnumStall_() {
  // Logger::Info("enblStallEnum_ = %d", enblStallEnum_);
  if (!enblStallEnum_)
    return false;
  if (crntNode_->IsNxtSlotStall())
    return true;
  if (crntNode_ == rootNode_)
    return false;
  if (dataDepGraph_->IncludesUnpipelined())
    return true;
  //  return false;
  return true;
}
/*****************************************************************************/

void LengthCostEnumerator::InitNewNode_(EnumTreeNode *newNode) {
  Enumerator::InitNewNode_(newNode);
}
/*****************************************************************************/

void LengthCostEnumerator::InitNewGlobalPoolNode_(EnumTreeNode *newNode) {
  Enumerator::InitNewGlobalPoolNode_(newNode);
}

HistEnumTreeNode *LengthCostEnumerator::AllocHistNode_(EnumTreeNode *node) {
  CostHistEnumTreeNode *histNode = histNodeAlctr_->GetObject();
  histNode->Construct(node, false, isGenerateState_);
  return histNode;
}
/*****************************************************************************/

HistEnumTreeNode *LengthCostEnumerator::AllocTempHistNode_(EnumTreeNode *node) {
  HistEnumTreeNode *histNode = tmpHstryNode_;
  histNode->Construct(node, true, isGenerateState_);
  return histNode;
}
/*****************************************************************************/

void LengthCostEnumerator::FreeHistNode_(HistEnumTreeNode *histNode) {
  histNode->Clean();
  histNodeAlctr_->FreeObject((CostHistEnumTreeNode *)histNode);
}
/*****************************************************************************/


void LengthCostEnumerator::setLCEElements(BBThread *bbt, InstCount costLwrBound)
{
  bbt_ = bbt;
  costLwrBound_ = costLwrBound;
}
