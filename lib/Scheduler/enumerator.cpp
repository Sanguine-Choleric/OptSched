#include "opt-sched/Scheduler/enumerator.h"
#include "opt-sched/Scheduler/bb_thread.h"
#include "opt-sched/Scheduler/logger.h"
#include "opt-sched/Scheduler/random.h"
#include "opt-sched/Scheduler/stats.h"
#include "opt-sched/Scheduler/utilities.h"
#include "opt-sched/Scheduler/hist_table.h"
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
      //assert(frwrdLwrBounds_ != NULL);
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
  cost_= costLwrBound_ = peakSpillCost_ = spillCostSum_ = INVALID_VALUE;
  crntCycleBlkd_ = false;
  rsrvSlots_ = NULL;
  totalCostIsActualCost_ = false;
  totalCost_.store(INVALID_VALUE);
  localBestCost_.store(INVALID_VALUE);
  explordChildren_.store(0);
  isArtRoot_ = false;
  IsInfsblFromBacktrack_ = false;
  pushedToLocalPool_ = false;
  wasChildStolen_ = false;
  recyclesHistNode_ = false;
  IncrementedParent_ = false;

  explordChildren_.store(0);
  prevNode_ = nullptr;
  std::queue<int> empty;
  std::swap( stolenInsts_, empty );

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
  
  isFirstPass_ = enumrtr->IsTwoPass_ && !enumrtr->IsSecondPass_;
  
  prevNode_ = prevNode;

  inst_ = inst;

  #ifdef IS_DEBUG_WORKSTEAL
  prefixInstNums_ = new std::queue<int>();
  if (prevNode_) {
    int prefixTip = inst_ ? inst_->GetNum() : INVALID_VALUE;
    copyInstPrefix(prevNode_->getInstPrefix());
    pushToInstPrefix(prefixTip);
  }
  else pushToInstPrefix(INVALID_VALUE);
  #endif
    

  enumrtr_ = enumrtr;
  time_ = prevNode_ == NULL ? 0 : prevNode_->time_ + 1;

  InstCount instCnt_ = enumrtr_->totInstCnt_;

  assert(instCnt_ != INVALID_VALUE);
  
   if (isCnstrctd_ == false) {
    exmndInsts_ = new LinkedList<ExaminedInst>(INVALID_VALUE);
    chldrn_ = new LinkedList<HistEnumTreeNode>(INVALID_VALUE);
    frwrdLwrBounds_ = new InstCount[instCnt_];
    allocSize_ = instCnt_;
  }

    if (instCnt_ != allocSize_) {
      delete[] frwrdLwrBounds_;
      frwrdLwrBounds_ = new InstCount[instCnt_];
      allocSize_ = instCnt_;
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
  IncrementedParent_ = false;

  instCnt_ = enumrtr->getTotalInstCnt();
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

  cost_= costLwrBound_ = peakSpillCost_ = spillCostSum_ = INVALID_VALUE;
  totalCost_.store(INVALID_VALUE);
  localBestCost_.store(INVALID_VALUE);

  isArtRoot_ = false;
  totalCostIsActualCost_ = false;
  IsInfsblFromBacktrack_ = false;
  pushedToLocalPool_ = false;
  wasChildStolen_.store(false);
  recyclesHistNode_ = false;
  isArchivd_ = false;  
  IncrementedParent_ = false;

  explordChildren_.store(0);
  prevNode_ = nullptr;
  std::queue<int> empty;
  std::swap( stolenInsts_, empty );

  isClean_ = true;

#ifdef IS_DEBUG_WORKSTEAL
  if (prefixInstNums_ != nullptr) delete prefixInstNums_;
#endif
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
    InstCount deadline = enumrtr_->bbt_->isSecondPass() ? inst->GetCrntDeadline(enumrtr_->getSolverID()) : -1;
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
  numChildren_ = rdyLstSize;
  explordChildren_.store(0);
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
/*
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
*/
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

void EnumTreeNode::Archive(bool fullyExplored) {
  if (fullyExplored) {
    if (!wasChildStolen_) assert(isArchivd_ == false || getRecyclesHistNode());

    if (enumrtr_->IsCostEnum()) {
      hstry_->SetCostInfo(this, false, enumrtr_);
    }

    isArchivd_ = true;
  }
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
void EnumTreeNode::removeNextPriorityInst() {
  SchedInstruction *inst = rdyLst_->GetNextPriorityInst();
  while (inst->GetNum() != GetInstNum()) 
    inst = rdyLst_->GetNextPriorityInst();
  rdyLst_->RemoveNextPriorityInst();
}

/****************************************************************************/
void EnumTreeNode::printRdyLst()  {
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
                       Milliseconds timeout, int SolverID, int NumSolvers,
                       int timeoutToMemblock, MemAlloc<EnumTreeNode> *EnumNodeAlloc,
                       MemAlloc<BinHashTblEntry<HistEnumTreeNode>> *HashTablAlloc,
                       bool isSecondPass, InstCount preFxdInstCnt, SchedInstruction *preFxdInsts[])
    : ConstrainedScheduler(dataDepGraph, machMdl, schedUprBound, SolverID) {

  //#ifndef IS_DEBUG_WORKSTEAL
  //  #define IS_DEBUG_WORKSTEAL
  //#endif

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

  NumSolvers_ = NumSolvers;
  
  timeoutToMemblock_ = timeoutToMemblock;

  memAllocBlkSize_ = (int)timeout / timeoutToMemblock_;
  assert(preFxdInstCnt >= 0);

  if (memAllocBlkSize_ > MAX_MEMBLOCK_SIZE) {
    memAllocBlkSize_ = MAX_MEMBLOCK_SIZE;
  }

  if (memAllocBlkSize_ == 0) {
    memAllocBlkSize_ = 1;
  }

  isCnstrctd_ = false;

  IsSecondPass_ = isSecondPass;
  timeoutToMemblock_ *= IsSecondPass_ ? 100 : 1;


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

  nodeAlctr_ = new TreeNodeAllocWrapper(EnumNodeAlloc);
  hashTblEntryAlctr_ = HashTablAlloc;

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

  int16_t sigSize = 8 * sizeof(InstSignature) - 1;

  Milliseconds histTableInitTime = Utilities::GetProcessorTime();

  exmndSubProbs_ = NULL;

  // Dont bother constructing if its a worker
  if (IsHistDom() && SolverID <= 1) {
    UDT_HASHTBL_CPCTY maxSize = 24000000000; // 32GB * 3/4
    maxSize /= (sizeof(BinHashTblEntry<CostHistEnumTreeNode>) + sizeof(CostHistEnumTreeNode));
    Logger::Info("max size fo hist table %llu", maxSize);
    exmndSubProbs_ =
        new BinHashTable<HistEnumTreeNode>(sigSize, sigHashSize, true, NumSolvers_, maxSize);
  }

  histTableInitTime = Utilities::GetProcessorTime() - histTableInitTime;
  //stats::historyTableInitializationTime.Record(histTableInitTime);

  tightndLst_ = NULL;
  bkwrdTightndLst_ = NULL;
  dirctTightndLst_ = NULL;
  fxdLst_ = NULL;

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
}
/****************************************************************************/

Enumerator::~Enumerator() {
  // double free if workers try to delete hist table -- refers to same object
  // TODO -- replace SolverID_ <= 1 with inline isWorker()
  if (SolverID_ <= 1) {
    delete exmndSubProbs_;
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
  delete nodeAlctr_;
}
/****************************************************************************/

void Enumerator::SetupAllocators_() {
  //int memAllocBlkSize = memAllocBlkSize_;
  int lastInstsEntryCnt = issuRate_ * (dataDepGraph_->GetMaxLtncy());
  
  /*int maxNodeCnt = issuRate_ * schedUprBound_ + 1;
  int additionalNodes = (bbt_->isWorker() && IsFirstPass_) ? bbt_->getLocalPoolMaxSize(SolverID_ - 2) * 4 : 0;
  maxNodeCnt += additionalNodes;
  int maxSize = INVALID_VALUE;


  nodeAlctr_ = new EnumTreeNodeAlloc(maxNodeCnt, maxSize);
*/
  if (IsHistDom()) {
  //  hashTblEntryAlctr_ =
  //      new MemAlloc<BinHashTblEntry<HistEnumTreeNode>>(memAllocBlkSize);

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
    if (IsHistDom()) {
      hashTblEntryAlctr_->Reset();
    }
    nodeAlctr_->Reset();
  }
}

/****************************************************************************/

void Enumerator::FreeAllocators_(){
  if (!alctrsFreed_) {


    if (IsHistDom()) {
      hashTblEntryAlctr_->Reset();
    }
    nodeAlctr_->Reset();

    if (rlxdSchdulr_ != NULL)
      delete rlxdSchdulr_;
    rlxdSchdulr_ = NULL;

    if (IsHistDom()) {
      //if (hashTblEntryAlctr_ != NULL)
      //  delete hashTblEntryAlctr_;
      //hashTblEntryAlctr_ = NULL;
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

    alctrsFreed_ = true;
  }
}

void Enumerator::freeNodeAllocator() {
  //if (nodeAlctr_ != NULL) {
  //  delete nodeAlctr_;
  //}
  //nodeAlctr_ = NULL;
}

/****************************************************************************/

void Enumerator::deleteNodeAlctr() {
  //delete nodeAlctr_;
}
/****************************************************************************/

void Enumerator::freeEnumTreeNode(EnumTreeNode *node) {
  nodeAlctr_->Free(node);
}

/****************************************************************************/
void Enumerator::Reset() {
  schduldInstCnt_ = 0;
  crntSlotNum_ = 0;
  crntRealSlotNum_ = 0;
  crntCycleNum_ = 0;
  exmndNodeCnt_ = 0;

  if (IsHistDom() && SolverID_ <= 1) {
    exmndSubProbs_->Clear(false, hashTblEntryAlctr_);
  }

  ResetAllocators_();

  for (InstCount i = 0; i < schedUprBound_; i++) {
    if (frstRdyLstPerCycle_[i] != NULL) {
      frstRdyLstPerCycle_[i]->Reset();
    }
  }

  fxdLst_->Reset();
  tightndLst_->Reset();
  dirctTightndLst_->Reset();
  bkwrdTightndLst_->Reset();
  dataDepGraph_->SetSttcLwrBounds();
}
/****************************************************************************/

void Enumerator::resetEnumHistoryState()
{

  // TODO -- can this be removed?
  assert(IsHistDom());

  int lastInstsEntryCnt = issuRate_ * (dataDepGraph_->GetMaxLtncy());
  
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

  // TODO -- disable this stuff for first pass
  rlxdSchdulr_->Initialize(false);

  if (preFxdInstCnt_ > 0) {
    if (InitPreFxdInsts_() == false) {
      return false;
    }

    dataDepGraph_->SetDynmcLwrBounds();
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
  CreateRootNode_();
  crntNode_ = rootNode_;

  ClearState_();
  return true;
}
/*****************************************************************************/

bool Enumerator::InitPreFxdInsts_() {
  // TODO -- parameterize
  if (true) return true;
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

  for (i = 0; i < totInstCnt_; i++) {
    SchedInstruction *inst = dataDepGraph_->GetInstByIndx(i);
    InstSignature sig = RandomGen::GetRand32();

    // ensure it is not zero
    if (sig == 0) {
      sig += 1;
    }

    // left shift by the number of bits needed to encode the instruction number
    sig <<= bitsForInstNum;

    // now, place the instruction number in the least significant bits
    sig |= i;

    // sig &= 0x7fffffffffffffff;
    sig &= 0x7fffffff;

    assert(sig != 0);

    inst->SetSig(sig);
  }
}
/*****************************************************************************/

SchedInstruction *Enumerator::GetInstByIndx(InstCount index) {
  return dataDepGraph_->GetInstByIndx(index);
}

/*****************************************************************************/

void Enumerator::CreateRootNode_() {
  rootNode_ = nodeAlctr_->Alloc(NULL, NULL, this);
#ifdef IS_CORRECT_LOCALPOOL
   if (SolverID_  >= 2) {
   bbt_->GlobalPoolLock_->lock();
   Logger::Info("SolverID %d created node %d", SolverID_, rootNode_);
   bbt_->GlobalPoolLock_->unlock();
}
#endif
  CreateNewRdyLst_();
  rootNode_->SetRdyLst(rdyLst_);
  if (bbt_->isSecondPass())
    rootNode_->SetLwrBounds(DIR_FRWRD);
  assert(rsrvSlotCnt_ == 0);
  rootNode_->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);
  bool setCost = true;
  if (SolverID_ == 1) setCost = false;
  InitNewNode_(rootNode_, setCost);
  CmtLwrBoundTightnng_();
}
/*****************************************************************************/

void Enumerator::setHistTable(BinHashTable<HistEnumTreeNode> *exmndSubProbs) {
  assert(IsHistDom());
  exmndSubProbs_ = exmndSubProbs;
}
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
  auto concatSched = std::unique_ptr<InstSchedule>(bbt_->allocNewSched());
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
    assert(false && "in append and check suffix");
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
  bbt_->initForSchdulng();
  InstCount cycleNum, slotNum;
  for (auto instNum = crntSched_->GetFrstInst(cycleNum, slotNum);
       instNum != INVALID_VALUE;
       instNum = crntSched_->GetNxtInst(cycleNum, slotNum)) {
    bbt_->schdulInst(dataDepGraph_->GetInstByIndx(instNum), cycleNum, slotNum,
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

  isWorker_ = bbt_->isWorker();

  // workers initialize the enumerator before calling FindFeasibleSched
  if (!bbt_->isWorker()) {
    if (Initialize_(sched, trgtLngth) == false) 
      return RES_FAIL;
  }

#ifdef IS_DEBUG_NODES
  uint64_t prevNodeCnt = exmndNodeCnt_;
#endif

  // TODO -- exit on solution found (second pass) instead of next iteration of this loop
  while (!(allNodesExplrd || WasObjctvMet_())) {
    if (deadline != INVALID_VALUE && Utilities::GetProcessorTime() > deadline) {
      isTimeout = true;
      break;
    }

    mostRecentMatchingHistNode_ = nullptr;

    if (isCrntNodeFsbl) {
      foundFsblBrnch = FindNxtFsblBrnch_(nxtNode);
    } else {
      foundFsblBrnch = false;
    }

    if (foundFsblBrnch) {
      // (Chris): It's possible that the node we just determined to be feasible
      // dominates a history node with a suffix schedule. If this is the case,
      // then instead of continuing the search, we should generate schedules by
      // concatenating the best known suffix.


      StepFrwrd_(nxtNode);
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
      if (crntNode_ == rootNode_) {
        if (bbt_->isWorker() && IsFirstPass_) BackTrackRoot_();
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
    return RES_TIMEOUT;
  }
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
    if (SolverID_ >= 2) {
    bbt_->GlobalPoolLock_->lock();
    Logger::Info("SolverID %d, probing branch %d out of %d", SolverID_, i, brnchCnt);
    bbt_->GlobalPoolLock_->unlock();
    }
#endif
#ifdef IS_DEBUG_FLOW
    Logger::Info("SolverID %d Probing branch %d out of %d", SolverID_, i, brnchCnt);
#endif

    if (i == brnchCnt - 1) {
      assert(i == rdyLst_->GetInstCnt());
      if (!bbt_->isSecondPass()) {
        return false;
      }
#ifdef IS_DEBUG_SEARCH_ORDER
      Logger::Log((Logger::LOG_LEVEL) 4, false, "Out of instructions, stalling");
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
      assert(rdyLst_);

      if (bbt_->isWorkStealOn()) {
        if (!bbt_->isWorker() || !IsFirstPass_) {
          inst = rdyLst_->GetNextPriorityInst();
        }

        if (bbt_->isWorker() && IsFirstPass_) {
          if (crntNode_->getPushedToLocalPool()) {
            bbt_->localPoolLock(SolverID_ - 2);
            inst = rdyLst_->GetNextPriorityInst();
            if (!inst) {
              bbt_->localPoolUnlock(SolverID_ - 2);
              return false;
            }
            if (crntNode_->wasInstStolen(inst)) {
              bbt_->localPoolUnlock(SolverID_ - 2);
              continue;
            }


            EnumTreeNode *removed = nullptr;
            bbt_->localPoolRemoveSpecificElement(SolverID_ - 2, inst, crntNode_, removed);
            bbt_->localPoolUnlock(SolverID_ - 2);
            if (removed != nullptr) {
#ifdef IS_CORRECT_LOCALPOOL
	bbt_->GlobalPoolLock_->lock();
		Logger::Info("SolverID %d, Removed element from pool at depth %d\n", SolverID_, removed->GetTime());
	bbt_->GlobalPoolLock_->unlock();
#endif
              nodeAlctr_->Free(removed);
            }
          }

          else { //we haven't pushed this nodes rdylst to local pool
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
                              bool &isLngthFsbl, bool prune) {

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


  // If this instruction is prefixed, it cannot be scheduled earlier than its
  // prefixed cycle
  if (inst != NULL && bbt_->isSecondPass())
    if (inst->GetPreFxdCycle() != INVALID_VALUE)
      if (inst->GetPreFxdCycle() != crntCycleNum_) {
#ifdef IS_DEBUG_SEARCH_ORDER
        Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: prefix fail");
#endif

        return false;
      }

  if (inst != NULL && bbt_->isSecondPass()) {
    if (inst->GetCrntLwrBound(DIR_FRWRD) > crntCycleNum_) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
      stats::forwardLBInfeasibilityHits++;
#endif
      //stats::forwardLBInfeasibilityHits++;
#ifdef IS_DEBUG_SEARCH_ORDER
      Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: LB fail");
#endif
      return false;
    }
    if (inst->GetCrntDeadline(SolverID_) < crntCycleNum_) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
      stats::backwardLBInfeasibilityHits++;
#endif
      //stats::backwardLBInfeasibilityHits++;

#ifdef IS_DEBUG_SEARCH_ORDER
      Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: deadline fail");
#endif
      return false;
    }
  }

  // If we are scheduling for register pressure only, and this branch
  // defines a register but does not use any, we can prune this branch
  // if another instruction in the ready list does use a register.
  if (SchedForRPOnly_) {
    if (inst != NULL && crntNode_->FoundInstWithUse() &&
        inst->GetAdjustedUseCnt() == 0 && !dataDepGraph_->DoesFeedUser(inst)) { 
          return false;
        }
  }

  if (prune_.nodeSup) {
    if (inst != NULL) {
      if (crntNode_->WasSprirNodeExmnd(inst)) {
        //stats::nodeSuperiorityInfeasibilityHits++;
      nodeSupInfsbl++;
        isNodeDmntd = true;
#ifdef IS_DEBUG_SEARCH_ORDER
        Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: history fail");
#endif

        return false;
      }
    }
  }

  if (inst != NULL) {
    if (isSecondPass())
      inst->Schedule(crntCycleNum_, crntSlotNum_);
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
  if (!bbt_->isSecondPass()) Logger::Info("actually pruning due to slot count");
  //stats::slotCountInfeasibilityHits++;
#ifdef IS_DEBUG_SEARCH_ORDER
    Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: issue slot fail");
#endif
    return false;
  }

  if (bbt_->isSecondPass()) {
    fsbl = TightnLwrBounds_(inst);
    state_.lwrBoundsTightnd = true;
  }

  if (!fsbl) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
    stats::rangeTighteningInfeasibilityHits++;
#endif
  if (!bbt_->isSecondPass()) Logger::Info("actually pruning due to rng tightn");
  //stats::rangeTighteningInfeasibilityHits++;

#ifdef IS_DEBUG_SEARCH_ORDER
    Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: tightn LB fail");
#endif
    return false;
  }

  state_.instFxd = true;

  newNode = nodeAlctr_->Alloc(crntNode_, inst, this);
#ifdef IS_CORRECT_LOCALPOOL
if (SolverID_ >= 2) {
   bbt_->GlobalPoolLock_->lock();
   Logger::Info("SolverID %d created node %d", SolverID_, newNode);
   bbt_->GlobalPoolLock_->unlock();
}
#endif
  if (bbt_->isSecondPass())
    newNode->SetLwrBounds(DIR_FRWRD);
  newNode->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);

  // If a node (sub-problem) that dominates the candidate node (sub-problem)
  // has been examined already and found infeasible


  if (prune_.histDom && IsHistDom()) {
    if (isEarlySubProbDom_) {
      assert(false);
      if (WasDmnntSubProbExmnd_(inst, newNode)) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
        stats::historyDominationInfeasibilityHits++;
#endif
  //stats::historyDominationInfeasibilityHits++;
#ifdef IS_DEBUG_SEARCH_ORDER
        Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: histDom fail");
#endif
        nodeAlctr_->Free(newNode);
        newNode = NULL;
        return false;
      }
    }
  }

  // Try to find a relaxed schedule for the unscheduled instructions
  if (prune_.rlxd && bbt_->isSecondPass()) {
    fsbl = RlxdSchdul_(newNode);
    state_.rlxSchduld = true;

    if (fsbl == false) {
#ifdef IS_DEBUG_INFSBLTY_TESTS
      stats::relaxedSchedulingInfeasibilityHits++;
#endif
    if (!bbt_->isSecondPass()) Logger::Info("actually pruning due to rlx schd");
  //stats::relaxedSchedulingInfeasibilityHits++;

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
    assert(avlblSlotsInCrntCycle_[issuType] > 0);
    assert(avlblSlots_[issuType] > 0);
    avlblSlotsInCrntCycle_[issuType]--;
    avlblSlots_[issuType]--;
    neededSlots_[issuType]--;
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
    if (isSecondPass())
      inst->UnSchedule();
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


}
/*****************************************************************************/

void Enumerator::StepFrwrd_(EnumTreeNode *&newNode) {
  ++bbt_->StepFrwrds;
  SchedInstruction *instToSchdul = newNode->GetInst();
  InstCount instNumToSchdul;
#ifdef IS_CORRECT_LOCALPOOL
  if (SolverID_ >= 2) {
bbt_->GlobalPoolLock_->lock();
    Logger::Info("SolverID %d, inst %d, Stepping frwrd to time %d\n", SolverID_, instToSchdul->GetNum(),newNode->GetTime());
bbt_->GlobalPoolLock_->unlock();

  }
#endif
#ifdef IS_DEBUG_SEARCH_ORDER
  if (instToSchdul)
    Logger::Log((Logger::LOG_LEVEL) 4, false, "Stepping forward to inst %d", instToSchdul->GetNum());
#endif

if (bbt_->isWorkStealOn()) {
  if (bbt_->isWorker() && IsFirstPass_) {
    bool pushedToLocal = false;
    if (!crntNode_->getPushedToLocalPool()) {
      if (bbt_->getLocalPoolSize(SolverID_ - 2) < bbt_->getLocalPoolMaxSize(SolverID_ - 2)) {
        LinkedList<SchedInstruction> fillList;
        rdyLst_->GetUnscheduledInsts(&fillList);
        EnumTreeNode *pushNode;
        if (fillList.GetElmntCnt() > 0) {
#ifdef IS_CORRECT_LOCALPOOL
bbt_->GlobalPoolLock_->lock();
	Logger::Info("SolverID %d, Added %d, elements to local pool with time %d\n", SolverID_, fillList.GetElmntCnt(), crntNode_->GetTime());
bbt_->GlobalPoolLock_->unlock();

#endif
          pushedToLocal = true;
          fillList.ResetIterator();
          SchedInstruction *temp = fillList.GetFrstElmnt();
          bbt_->localPoolLock(SolverID_ - 2);
          while (temp != NULL) {
            pushNode = nodeAlctr_->Alloc(crntNode_, temp, this, false);
	    assert(pushNode->GetTime() <= (crntNode_->GetTime() + 1));
            bbt_->localPoolPushFront(SolverID_ - 2, pushNode);
#ifdef IS_CORRECT_LOCALPOOL
bbt_->GlobalPoolLock_->lock();
        Logger::Info("SolverID %d, pushnode %d, time %d\n", SolverID_, pushNode, pushNode->GetTime());
//         auto temp2 = pushNode;
//          Logger::Info("pushnode prefix (reverse)\n");
//          while (temp2 != NULL) {
//           Logger::Info("%d", temp2->GetInstNum());
//           temp2 = temp2->GetParent();
//         }
bbt_->GlobalPoolLock_->unlock();
#endif

            temp = fillList.GetNxtElmnt();
          }
          
          bbt_->localPoolUnlock(SolverID_ - 2);
        }
      }
      crntNode_->setPushedToLocalPool(pushedToLocal);

    }
  }  
}

  CreateNewRdyLst_();

  // Let the new node inherit its parent's ready list before we update it
  newNode->SetRdyLst(rdyLst_);

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
    if (IsHistDom() && !crntNode_->getRecyclesHistNode()) {
      assert(!crntNode_->IsArchived());
        UDT_HASHVAL key = exmndSubProbs_->HashKey(crntNode_->GetSig());

      if (bbt_->isWorker() && IsFirstPass_) {
        HistEnumTreeNode *crntHstry = crntNode_->GetHistory();
        crntHstry->setFullyExplored(false);
        crntHstry->setCostIsUseable(false);
        bbt_->histTableLock(key);
          assert(!crntHstry->isInserted());
          exmndSubProbs_->InsertElement(crntNode_->GetSig(), crntHstry,
                                    hashTblEntryAlctr_, bbt_);
          crntHstry->setInserted(true);
#ifdef DEBUG_TOTAL_COST
          assert(!crntHstry->getFullyExplored());
          assert(!crntHstry->getCostIsUseable());
          CostHistEnumTreeNode *temp = static_cast<CostHistEnumTreeNode *>(crntHstry);
          assert(temp->getPartialCost() == temp->getTotalCost());
#endif
        bbt_->histTableUnlock(key);
      }
      
      else {
        HistEnumTreeNode *crntHstry = crntNode_->GetHistory();
        crntHstry->setFullyExplored(false);
        crntHstry->setCostIsUseable(false);
        crntNode_->Archive(false);
        exmndSubProbs_->InsertElement(crntNode_->GetSig(), crntHstry,
                                    hashTblEntryAlctr_, bbt_);
      }
        

    } else {
      if (!crntNode_->getRecyclesHistNode()) assert(crntNode_->IsArchived() == false);
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

void Enumerator::InitNewNode_(EnumTreeNode *newNode, bool setCost) {
  crntNode_ = newNode;

  crntNode_->SetCrntCycleBlkd(isCrntCycleBlkd_);
  crntNode_->SetRealSlotNum(crntRealSlotNum_);

  if (IsHistDom() && !crntNode_->getRecyclesHistNode()) {
    bool setCost = true;
    if (SolverID_ < 2) setCost = false;
    crntNode_->CreateHistory(setCost);
    assert(crntNode_->GetHistory() != tmpHstryNode_);
  }

  crntNode_->SetSlotAvlblty(avlblSlots_, avlblSlotsInCrntCycle_);

  UpdtRdyLst_(crntCycleNum_, crntSlotNum_);
  bool isLeaf = schduldInstCnt_ == totInstCnt_;

  crntNode_->SetBranchCnt(rdyLst_->GetInstCnt(), isLeaf);

  createdNodeCnt_++;
  crntNode_->SetNum(createdNodeCnt_);

  crntNode_->setIsInfsblFromBacktrack_(false);
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

bool Enumerator::SetTotalCostsAndSuffixes(EnumTreeNode *const currentNode,
                              EnumTreeNode *const parentNode,
                              const InstCount targetLength,
                              const bool suffixConcatenationEnabled,
                              const bool fullyExplored) {
  // (Chris): Before archiving, set the total cost info of this node. If it's a
  // leaf node, then the total cost is the current cost. If it's an inner node,
  // then the total cost either has already been set (if one of its children had
  // a real cost), or hasn't been set, which means the total cost right now is
  // the dynamic lower bound of this node.


  bool changeMade = false;

  if (currentNode->IsLeaf()) {
#if defined(IS_DEBUG_ARCHIVE)
    Logger::Info("Leaf node total cost %d", currentNode->GetCost());
#endif
    currentNode->SetTotalCost(currentNode->GetCost());
    currentNode->SetTotalCostIsActualCost(true);
    currentNode->SetLocalBestCost(currentNode->GetCost());
  } else {
    if (!currentNode->GetTotalCostIsActualCost() &&
        (currentNode->GetTotalCost() == INVALID_VALUE ||
         currentNode->GetCostLwrBound() < currentNode->GetTotalCost())) {
#if defined(IS_DEBUG_ARCHIVE)
      Logger::Info("Inner node doesn't have a real cost yet. Setting total "
                   "cost to dynamic lower bound %d",
                   currentNode->GetCostLwrBound());
#endif
      assert(currentNode->GetCostLwrBound() >= currentNode->GetTotalCost());
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

        // TODO: parrentSuffix is not synchronized -- suffix concatenation has not been 
        // enabled for parallel algorithm
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
        changeMade = true;
      } else if (currentNode->GetTotalCost() < parentNode->GetTotalCost()) {
#if defined(IS_DEBUG_ARCHIVE)
        Logger::Info(
            "Current node has a real cost (%d), and so does parent. (%d)",
            currentNode->GetTotalCost(), parentNode->GetTotalCost());
#endif
        assert(parentNode->GetTotalCostIsActualCost());
        parentNode->SetTotalCost(currentNode->GetTotalCost());
        parentNode->SetSuffix(std::move(parentSuffix));
        changeMade = true;
      }
    }


    if (fullyExplored) {
      if (currentNode->GetLocalBestCost() == INVALID_VALUE) {
        currentNode->SetLocalBestCost(currentNode->GetCostLwrBound());
      }
      changeMade |= parentNode->SetLocalBestCost(currentNode->GetLocalBestCost());
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
  return changeMade;
}


bool Enumerator::BackTrack_(bool trueState) {
  ++bbt_->BackTracks;
  bool fsbl = true;
  SchedInstruction *inst = crntNode_->GetInst();
  EnumTreeNode *trgtNode = crntNode_->GetParent();
  bool fullyExplored = false;

  if (crntNode_->GetInst()) {
#ifdef IS_DEBUG_SEARCH_ORDER
    Logger::Log((Logger::LOG_LEVEL) 4, false, "SolverID %d Back tracking fron inst %d to inst %d", SolverID_, inst->GetNum(), trgtNode->GetInstNum());
#endif
}
  rdyLst_->RemoveLatestSubList();

// TODO cleanup the insert_on_backtrack vs stepfrwrd code
#ifdef INSERT_ON_BACKTRACK
  if (IsHistDom() && trueState) {
    assert(!crntNode_->IsArchived());
    HistEnumTreeNode *crntHstry = crntNode_->GetHistory();
    UDT_HASHVAL key = exmndSubProbs_->HashKey(crntNode_->GetSig());

    if (bbt_->isWorker() && IsFirstPass_) {
      // These may need to be protected by lock
      assert(!crntNode_->GetHistory()->getFullyExplored() || crntNode_->wasChildStolen());
      assert(crntNode_->getExploredChildren() <= crntNode_->getNumChildrn());
      bbt_->histTableLock(key);
        // It is posible we are falling to this backtrack directly from another backtrack
        // in which case, the exploredChild != numChildren but it should be labeled as fully explored
        if (crntNode_->getExploredChildren() == crntNode_->getNumChildrn() || (crntNode_->getIsInfsblFromBacktrack_() && !crntNode_->wasChildStolen())) {
          if (!crntNode_->getIncrementedParent()) {
            trgtNode->incrementExploredChildren();
            crntNode_->setIncrementedParent(true);
          }
          fullyExplored = true;
          if (crntNode_->wasChildStolen()) Logger::Info("$$GOODHIT -- fullyexplored with stolen child");
        }
        // set fully explored to fullyExplored when work stealing
        crntHstry->setFullyExplored(fullyExplored);
        SetTotalCostsAndSuffixes(crntNode_, trgtNode, trgtSchedLngth_,
                             prune_.useSuffixConcatenation, fullyExplored);
        crntNode_->Archive(fullyExplored);
        if (!crntNode_->getRecyclesHistNode()) {
          assert(!crntHstry->isInserted() || isSecondPass());
          exmndSubProbs_->InsertElement(crntNode_->GetSig(), crntHstry,
                                  hashTblEntryAlctr_, bbt_);
          crntHstry->setInserted(true);
        }
      bbt_->histTableUnlock(key);
    }

    else {
      HistEnumTreeNode *crntHstry = crntNode_->GetHistory();
      crntHstry->setFullyExplored(true);
      if (!crntNode_->getRecyclesHistNode()) {
        assert(!crntHstry->isInserted() || isSecondPass());
        exmndSubProbs_->InsertElement(crntNode_->GetSig(), crntHstry,
                                  hashTblEntryAlctr_, bbt_);
        crntHstry->setInserted(true);
      }
      SetTotalCostsAndSuffixes(crntNode_, trgtNode, trgtSchedLngth_,
                             prune_.useSuffixConcatenation, fullyExplored);
      crntNode_->Archive(true);
    }
      

  } else {
    assert(crntNode_->IsArchived() == false);
  }
#endif
#ifdef INSERT_ON_STEPFRWRD
  if (isSecondPass()) {
    assert(!bbt_->isWorker());
    if (IsHistDom() && trueState) {
      if (!crntNode_->getRecyclesHistNode()) assert(!crntNode_->IsArchived());
       // UDT_HASHVAL key = exmndSubProbs_->HashKey(crntNode_->GetSig());

        HistEnumTreeNode *crntHstry = crntNode_->GetHistory();
        assert(!crntHstry->getFullyExplored());
        crntHstry->setFullyExplored(true);
        if (!crntNode_->getRecyclesHistNode()) {
          assert(!crntHstry->isInserted());
          exmndSubProbs_->InsertElement(crntNode_->GetSig(), crntHstry,
                                    hashTblEntryAlctr_, bbt_);
          crntHstry->setInserted(true);
        }
        SetTotalCostsAndSuffixes(crntNode_, trgtNode, trgtSchedLngth_,
                              prune_.useSuffixConcatenation, fullyExplored);
        crntNode_->Archive(true);
    
    } 
    else {
      assert(crntNode_->IsArchived() == false);
    }
  }


 
  else {
    if (IsHistDom() && trueState) {
      UDT_HASHVAL key = exmndSubProbs_->HashKey(crntNode_->GetSig());
      HistEnumTreeNode *crntHstry = crntNode_->GetHistory();
      if (bbt_->isWorker()) {
          // These may need to be protected by lock
#ifdef DEBUG_TOTAL_COST
          assert(!crntHstry->getFullyExplored() || crntNode_->wasChildStolen());
          assert(crntNode_->getExploredChildren() <= crntNode_->getNumChildrn());
#endif
          bbt_->histTableLock(key);

          // It is posible we are falling to this backtrack directly from another backtrack
          // in which case, the exploredChild != numChildren but it should be labeled as fully explored
          if (crntNode_->getExploredChildren() == crntNode_->getNumChildrn() || (crntNode_->getIsInfsblFromBacktrack_() && !crntNode_->wasChildStolen())) {
            if (!crntNode_->getIncrementedParent()) {
            trgtNode->incrementExploredChildren();
            crntNode_->setIncrementedParent(true);
            }
          fullyExplored = true;
#ifdef DEBUG_TOTAL_COST
          if (crntNode_->wasChildStolen()) Logger::Info("$$GOODHIT -- fullyexplored with stolen child");
#endif
          }
          // set fully explored to fullyExplored when work stealing
          // there is a race condition to setFullyExplored when a child has stole
          // from the subspace, thus the fullyExplored assert is only true
          // if the subspace has not been stolen from
          crntHstry->setFullyExplored(fullyExplored);
          SetTotalCostsAndSuffixes(crntNode_, trgtNode, trgtSchedLngth_,
                            prune_.useSuffixConcatenation, fullyExplored);
          crntNode_->Archive(fullyExplored);
          bbt_->histTableUnlock(key);
      }
      else {
        crntHstry->setFullyExplored(true);
        SetTotalCostsAndSuffixes(crntNode_, trgtNode, trgtSchedLngth_,
                            prune_.useSuffixConcatenation, fullyExplored);
        crntNode_->Archive(true);
      }
    }
  }
#endif
 
  if (!crntNode_->wasChildStolen())
   nodeAlctr_->Free(crntNode_);
  else {
    trgtNode->setChildStolen(true);
  }

  EnumTreeNode *prevNode = crntNode_;
  crntNode_ = trgtNode;
  rdyLst_ = crntNode_->GetRdyLst();
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
  if (bbt_->isSecondPass())
    RestoreCrntLwrBounds_(inst, trueState);

  if (inst != NULL) {
    // int hitCnt;
    // assert(rdyLst_->FindInst(inst, hitCnt) && hitCnt == 1);
    assert(inst->IsInReadyList(SolverID_));

    UndoRsrvSlots_(inst);
    UnSchdulInst_(inst);
    if (isSecondPass())
      inst->UnSchedule();
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

#ifdef IS_DEBUG_SPD
  stats::signatureDominationTests++;
#endif
  HistEnumTreeNode *exNode;
  HistEnumTreeNode *lastMatch = nullptr;
  int listSize = exmndSubProbs_->GetListSize(newNode->GetSig());

  UDT_HASHVAL key = exmndSubProbs_->HashKey(newNode->GetSig());
  //stats::historyListSize.Record(listSize);
  if (listSize == 0) return false;
  mostRecentMatchingHistNode_ = nullptr;
  bool mostRecentMatchWasSet = false;
  bool wasDmntSubProbExmnd = false;
  int trvrsdListSize = 0;

  // lock table for syncrhonized iterator
  
  
  HashTblEntry<HistEnumTreeNode> *srchPtr = nullptr;
  exNode = exmndSubProbs_->GetLastMatch(srchPtr,newNode->GetSig());

  for (; trvrsdListSize < listSize; trvrsdListSize++) {
    if (exNode == NULL || exNode == nullptr) break;

#ifdef IS_DEBUG_SPD
    stats::signatureMatches++;
#endif
    if (exNode->DoesMatch(newNode, this, bbt_->isWorker() && IsFirstPass_, isGenerateState_)) {
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

        //stats::positiveDominationHits++;
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
        if (exNode->getFullyExplored()) {
          lastMatch = exNode;
        }
#ifdef IS_DEBUG_SPD
        stats::signatureAliases++;
#endif
      }
    }

    exNode = exmndSubProbs_->GetPrevMatch(srchPtr, newNode->GetSig());
  }

  if (!wasDmntSubProbExmnd && lastMatch != nullptr && IsTwoPass_ && !isSecondPass()) {
    bbt_->histTableLock(key);
    lastMatch->ResetHistFields(newNode);
    lastMatch->setRecycled(true);
    newNode->SetHistory(lastMatch);
    newNode->setRecyclesHistNode(true);
    newNode->setArchived(true);
    bbt_->histTableUnlock(key);
  }

  

  //stats::traversedHistoryListSize.Record(trvrsdListSize);
  return wasDmntSubProbExmnd;
}
/****************************************************************************/

bool Enumerator::TightnLwrBounds_(SchedInstruction *newInst, bool trueTightn) {
  if (bbt_->getIsTwoPass() && !bbt_->isSecondPass()) assert(false);
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
    if (trueTightn)
      assert(inst != newInst ||
            inst->GetCrntLwrBound(DIR_FRWRD) == crntCycleNum_);
      if (inst->IsSchduldSecondPass() == false) {
        IssueType issuType = inst->GetIssueType();
        newLwrBound = nxtAvlblCycle[issuType];


      if (newLwrBound > inst->GetCrntLwrBound(DIR_FRWRD)) {
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

      assert(inst->GetCrntLwrBound(DIR_FRWRD) >= newLwrBound);

      if (inst->GetCrntLwrBound(DIR_FRWRD) > inst->GetCrntDeadlineSecondPass()) {
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
    inst->UnTightnLwrBounds();
    dataDepGraph_->SetCrntFrwrdLwrBound(inst, SolverID_);
    assert(inst->IsFxd() == false);
  }

  tightndLst_->Reset();
  dirctTightndLst_->Reset();
}
/*****************************************************************************/

void Enumerator::CmtLwrBoundTightnng_() {
  SchedInstruction *inst;

  for (inst = tightndLst_->GetFrstElmnt(); inst != NULL;
       inst = tightndLst_->GetNxtElmnt()) {
    inst->CmtLwrBoundTightnng();
  }

  tightndLst_->Reset();
  dirctTightndLst_->Reset();
  CmtInstFxng_();
}
/*****************************************************************************/

bool Enumerator::FixInsts_(SchedInstruction *newInst) {
  if (!bbt_->isSecondPass()) return true;
  bool fsbl = true;

  bool newInstFxd = false;

  fxdInstCnt_ = 0;


  for (SchedInstruction *inst = fxdLst_->GetFrstElmnt(); inst != NULL;
       inst = fxdLst_->GetNxtElmnt()) {
    assert(inst->IsFxd());
    assert(inst->IsSchduld(SolverID_) == false || inst == newInst);
    fsbl = rlxdSchdulr_->FixInst(inst, inst->GetFxdCycle());

    if (inst == newInst) {
      newInstFxd = true;
      assert(inst->GetFxdCycle() == crntCycleNum_);
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
      if (newInst->IsFxd() == false)
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
  if (!bbt_->isSecondPass()) return;
  InstCount unfxdInstCnt = 0;
  SchedInstruction *inst;

  for (inst = fxdLst_->GetFrstElmnt(), unfxdInstCnt = 0;
       inst != NULL && unfxdInstCnt < fxdInstCnt_;
       inst = fxdLst_->GetNxtElmnt(), unfxdInstCnt++) {
    assert(inst->IsFxd() || inst == newInst);
    InstCount cycle = inst == newInst ? crntCycleNum_ : inst->GetFxdCycle();
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
  InstCount *frwrdLwrBounds = crntNode_->GetLwrBounds(DIR_FRWRD);
  bool unschduldInstDone = false;

  for (InstCount i = 0; i < totInstCnt_; i++) {
    SchedInstruction *inst = dataDepGraph_->GetInstByIndx(i);
    InstCount fxdCycle = 0;
    bool preFxd = inst->IsFxd();

    if (preFxd) {
      fxdCycle = inst->GetFxdCycle();
    }

    inst->SetCrntLwrBound(DIR_FRWRD, frwrdLwrBounds[i]);
    dataDepGraph_->SetCrntFrwrdLwrBound(inst, SolverID_);
    bool postFxd = inst->IsFxd();

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

    assert(unschduldInst->IsSchduld(SolverID_));

    if (unschduldInst->IsFxd() == false)
    // only if the untightening got it unfixed
    {
      rlxdSchdulr_->UnFixInst(unschduldInst, unschduldInst->GetSchedCycle(SolverID_));
    }
  }
}
/*****************************************************************************/

bool Enumerator::RlxdSchdul_(EnumTreeNode *newNode) {
  if (bbt_->getIsTwoPass() && !bbt_->isSecondPass()) assert(false);
  assert(newNode != NULL);
  LinkedList<SchedInstruction> *rsrcFxdLst = new LinkedList<SchedInstruction>;

  bool fsbl =
      rlxdSchdulr_->SchdulAndChkFsblty(crntCycleNum_, trgtSchedLngth_ - 1);

  for (SchedInstruction *inst = rsrcFxdLst->GetFrstElmnt(); inst != NULL;
       inst = rsrcFxdLst->GetNxtElmnt()) {
    assert(inst->IsSchduld(SolverID_) == false);
    fsbl = rlxdSchdulr_->FixInst(inst, inst->GetCrntLwrBound(DIR_FRWRD));

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
    InstCount rdyLstSize = rdyLst_->GetInstCnt();
    rdyLst_->ResetIterator();

    if (rdyLstSize > 0) {
      for (int i = 0; i < rdyLstSize; i++) {
        SchedInstruction *tempInst = rdyLst_->GetNextPriorityInst();
        if (tempInst->GetNum() == instructionNumber) {
          rdyLst_->RemoveNextPriorityInst();
          break;
        }
      }
      rdyLst_->ResetIterator();
    }
 }
/*****************************************************************************/

void Enumerator::PrintLog_() {
  Logger::Info("--------------------------------------------------\n");

  Logger::Info("Total nodes examined: %lld\n", GetNodeCnt());
  Logger::Info("History table includes %d entries.\n",
               exmndSubProbs_->GetEntryCnt());
  //Logger::GetLogStream() << stats::historyEntriesPerIteration;
  Logger::Info("--------------------------------------------------\n");
}
/*****************************************************************************/

bool Enumerator::EnumStall_() { return enblStallEnum_; }

// TODO remove
void Enumerator::printInfsbltyHits() {
  
  Logger::Info("Cost Infeasibility Hits = %d",CostInfsbl);
  Logger::Info("Relaxed Infeasibility Hits = %d",rlxdInfsbl);
  Logger::Info("Backward LB Infeasibility Hits = %d",bkwrdLBInfsbl);
  Logger::Info("Forward LB Infeasibility Hits = %d",frwrdLBInfsbl);
  Logger::Info("Node Superiority Infeasibility Hits = %d",nodeSupInfsbl);
  Logger::Info("History Domination Infeasibility Hits = %d",histDomInfsbl);
  Logger::Info("Range Tightening Infeasibility Hits = %d",rangeTightInfsbl);
  Logger::Info("Slot Count Infeasibility Hits = %d",slotCntInfsbl);
  
}

/*****************************************************************************/
// NOT SUPPORTED
LengthEnumerator::LengthEnumerator(
    DataDepGraph *dataDepGraph, MachineModel *machMdl, InstCount schedUprBound,
    int16_t sigHashSize, SchedPriorities prirts, Pruning PruningStrategy,
    bool SchedForRPOnly, bool enblStallEnum, Milliseconds timeout, bool IsSecondPass,
    InstCount preFxdInstCnt, SchedInstruction *preFxdInsts[])
    : Enumerator(dataDepGraph, machMdl, schedUprBound, sigHashSize, prirts,
                 PruningStrategy, SchedForRPOnly, enblStallEnum, timeout, 0, 1, 1, nullptr, nullptr, IsSecondPass, 
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

  Enumerator::FreeAllocators_();
}
/****************************************************************************/

bool LengthEnumerator::IsCostEnum() { return false; }

InstCount LengthEnumerator::GetBestCost() {
  assert(false && "wrong getBestCost");
  return INVALID_VALUE;
}
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

HistEnumTreeNode *LengthEnumerator::AllocHistNode_(EnumTreeNode *node, bool setCost) {
  HistEnumTreeNode *histNode = histNodeAlctr_->GetObject();
  if (histNode != nullptr)
    histNode->Construct(node, false, isGenerateState_);
  return histNode;
}
/*****************************************************************************/

HistEnumTreeNode *LengthEnumerator::AllocTempHistNode_(EnumTreeNode *node) {
  HistEnumTreeNode *histNode = tmpHstryNode_;
  histNode->Construct(node, true, isGenerateState_,false);
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
    SPILL_COST_FUNCTION spillCostFunc, bool IsSecondPass, int NumSolvers,  int timeoutToMemblock,
    MemAlloc<EnumTreeNode> *EnumNodeAlloc,
    MemAlloc<CostHistEnumTreeNode> *HistNodeAlloc, MemAlloc<BinHashTblEntry<HistEnumTreeNode>> *HashTablAlloc, int SolverID, InstCount preFxdInstCnt, SchedInstruction *preFxdInsts[])
    : Enumerator(dataDepGraph, machMdl, schedUprBound, sigHashSize, prirts,
                 PruningStrategy, SchedForRPOnly, enblStallEnum, timeout,
                 SolverID, NumSolvers, timeoutToMemblock, EnumNodeAlloc, HashTablAlloc, IsSecondPass, preFxdInstCnt,  preFxdInsts) {
  bbt_ = bbt;
  SolverID_ = SolverID;
  SetupAllocators_();

  histNodeAlctr_ = HistNodeAlloc;

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
      nodeAlctr_->Reset();
      if (IsHistDom()) {
        hashTblEntryAlctr_->Reset();
        histNodeAlctr_->Reset();
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
      nodeAlctr_->Reset();
      if (IsHistDom()) {
        hashTblEntryAlctr_->Reset();
      }
    }
  }
}


/*****************************************************************************/
void LengthCostEnumerator::SetupAllocators_() {
  //int memAllocBlkSize = memAllocBlkSize_;

  Enumerator::SetupAllocators_();

//  if (IsHistDom()) {
//    histNodeAlctr_ = new MemAlloc<CostHistEnumTreeNode>(memAllocBlkSize);
//  }
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
  //if (IsHistDom() & !alctrsFreed_) {
  //  Logger::Info("SolverID %d freeing history allocator with %d blocks", SolverID_, histNodeAlctr_->GetSize());
  //  if (histNodeAlctr_ != NULL)
  //    delete histNodeAlctr_;
  //  histNodeAlctr_ = NULL;
 // }
  if (IsHistDom() && !alctrsFreed_)
    histNodeAlctr_->Reset();
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

//TODO remove costLwrBound
FUNC_RESULT LengthCostEnumerator::FindFeasibleSchedule(InstSchedule *sched,
                                                       InstCount trgtLngth,
                                                       BBThread *bbt,
                                                       int costLwrBound,
                                                       Milliseconds deadline) {
  
  bbt_ = bbt;
  IsTwoPass_ = bbt_->getIsTwoPass();
  IsFirstPass_ = IsTwoPass_ && !IsSecondPass_;
  BypassLatencyChecking_ = IsSecondPass_ ? false : true;

  FUNC_RESULT rslt = FindFeasibleSchedule_(sched, trgtLngth, deadline);

#ifdef IS_DEBUG_TRACE_ENUM
  stats::costChecksPerLength.Record(costChkCnt_);
  stats::costPruningsPerLength.Record(costPruneCnt_);
  stats::feasibleSchedulesPerLength.Record(fsblSchedCnt_);
  stats::improvementsPerLength.Record(imprvmntCnt_);
#endif

  return rslt;
}

/*****************************************************************************/
bool LengthCostEnumerator::WasObjctvMet_() {
  assert(GetBestCost_() >= 0);

  bool HasLegalSchedule = WasSolnFound_();

  if (!HasLegalSchedule && !bbt_->isWorker()) {
    return false;
  }

  // crntCost is normalizeed (lower bound subtracted)
  InstCount crntCost = GetBestCost_();
  if (HasLegalSchedule) {
    InstCount newCost = bbt_->UpdtOptmlSched(crntSched_, this);
    if (!bbt_->isWorker() || !IsFirstPass_) assert(newCost <= GetBestCost_());

    if (newCost < crntCost) {
      imprvmntCnt_++;
      if (bbt_->isWorker() && IsFirstPass_) 
        bbt_->incrementImprvmntCnt();
    }

    crntCost = newCost;
  }

  // crntCost is normalized, thus it is the true cost - cost lwr bound.
  // This condition is true iff true cost == cost lwr bound
  if (crntCost == 0) Logger::Info("objctv met");

  return crntCost == 0;
}
/*****************************************************************************/

bool LengthCostEnumerator::ProbeBranch_(SchedInstruction *inst,
                                        EnumTreeNode *&newNode,
                                        bool &isNodeDmntd, bool &isRlxInfsbl,
                                        bool &isLngthFsbl, bool prune) {
  bool isFsbl = true;

  isFsbl = Enumerator::ProbeBranch_(inst, newNode, isNodeDmntd, isRlxInfsbl,
                                    isLngthFsbl);
  
  assert(newNode || !isFsbl);

  if (isFsbl == false) {
    ++bbt_->OtherInfsbl;
    assert(isLngthFsbl == false);
    isLngthFsbl = false;

    crntNode_->incrementExploredChildren();
    return false;
  }

  isLngthFsbl = true;

  isFsbl = ChkCostFsblty_(inst, newNode);


  if (isFsbl == false) {
    ++bbt_->CostInfsbl;
#ifdef IS_DEBUG_SEARCH_ORDER
    Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: cost fail");
#endif
    crntNode_->incrementExploredChildren();
    crntNode_->SetLocalBestCost(newNode->GetLocalBestCost());
    return false;
  }

  if (IsHistDom() && prune) {
    ++bbt_->HistInfsbl;

    assert(newNode);
    EnumTreeNode *parent = newNode->GetParent();
    if (WasDmnntSubProbExmnd_(inst, newNode)) {
#ifdef IS_DEBUG_FLOW
      Logger::Info("History domination\n\n");
#endif

#ifdef IS_DEBUG_INFSBLTY_TESTS
      stats::historyDominationInfeasibilityHits++;
#endif
  //stats::historyDominationInfeasibilityHits;
      bbt_->unschdulInst(inst, crntCycleNum_, crntSlotNum_, parent);
#ifdef IS_DEBUG_SEARCH_ORDER
      Logger::Log((Logger::LOG_LEVEL) 4, false, "probe: LCE history fail");
#endif
      isNodeDmntd = true;
      crntNode_->incrementExploredChildren();
      nodeAlctr_->Free(newNode);
      newNode = NULL;
      return false;
    }
  }

  assert(newNode);
  return true;
}

/*****************************************************************************/
bool LengthCostEnumerator::ChkCostFsblty_(SchedInstruction *inst,
                                          EnumTreeNode *&newNode,
                                          bool trueState) {
  bool isFsbl = true;

  costChkCnt_++;

  bbt_->schdulInst(inst, crntCycleNum_, crntSlotNum_, false);

  if (prune_.spillCost) {
    isFsbl = bbt_->chkCostFsblty(trgtSchedLngth_, newNode, !trueState);

    if (!isFsbl && trueState) {
      //stats::costInfeasibilityHits++;
#ifdef IS_DEBUG_FLOW
      Logger::Info("Detected cost infeasibility of inst %d in cycle %d",
                   inst == NULL ? -2 : inst->GetNum(), crntCycleNum_);
#endif
  CostInfsbl++;
      bbt_->unschdulInst(inst, crntCycleNum_, crntSlotNum_,
                         newNode->GetParent());
    }
  }

  return isFsbl;
}
/*****************************************************************************/

bool LengthCostEnumerator::BackTrack_(bool trueState) {
  SchedInstruction *inst = crntNode_->GetInst();

  bbt_->unschdulInst(inst, crntCycleNum_, crntSlotNum_, crntNode_->GetParent());

#ifdef IS_CORRECT_LOCALPOOL
  if (SolverID_ >= 2) {
bbt_->GlobalPoolLock_->lock();
    Logger::Info("SolverID %d, Backtracking to time %d\n", SolverID_, crntNode_->GetTime() - 1);
bbt_->GlobalPoolLock_->unlock();

  }
#endif


if (bbt_->isWorkStealOn()) {
  // it is possible that a crntNode becomes infeasible before exploring all its children
  // thus we need to ensure that all children are removed on backtrack
  if (bbt_->isWorker() && IsFirstPass_ && crntNode_->getIsInfsblFromBacktrack_()) {
#ifdef IS_CORRECT_LOCALPOOL
    bbt_->GlobalPoolLock_->lock();
    Logger::Info("SolverID %d, is infsbl from backtrack\n", SolverID_);
    bbt_->GlobalPoolLock_->unlock();
#endif
    bbt_->localPoolLock(SolverID_ - 2);
    if (bbt_->getLocalPoolSize(SolverID_ - 2) > 0) {
      EnumTreeNode *popNode = bbt_->localPoolPopFront(SolverID_ - 2);
      assert(popNode);
#ifdef IS_CORRECT_LOCALPOOL
      if ((popNode->GetTime() > (crntNode_->GetTime() + 1)) || (popNode->GetTime() == crntNode_->GetTime() + 1 && popNode->GetParent() != crntNode_)) {
          bbt_->GlobalPoolLock_->lock();
          Logger::Info("SolverID: %d\n", SolverID_);
          int i = 0;
          auto temp = popNode;
          Logger::Info("popnode prefix (reverse)\n");
          while (temp != NULL) {
            Logger::Info("%d", temp->GetInstNum());
            ++i;
            temp = temp->GetParent();
          }

          Logger::Info("Popped node has prefix length %d and time %d", i, popNode->GetTime()); 

          i = 0;
          temp = crntNode_;
          Logger::Info("crntnode prefix (reverse)\n");
          while (temp != NULL) {
            Logger::Info("%d", temp->GetInstNum());
            ++i;
            temp = temp->GetParent();
          }

          Logger::Info("Crnt node has prefix length %d and time %d", i, crntNode_->GetTime()); 
         bbt_->GlobalPoolLock_->unlock();
}
#endif

      assert(popNode->GetTime() <= (crntNode_->GetTime() + 1));

      while (popNode->GetTime() >= (crntNode_->GetTime() + 1)) {
#ifdef IS_CORRECT_LOCALPOOL
bbt_->GlobalPoolLock_->lock();
	Logger::Info("SolverID %d, Removed element from localPool at time %d\n", SolverID_, popNode->GetTime());
bbt_->GlobalPoolLock_->unlock();

#endif
        //assert(popNode->GetParent() == crntNode_->GetParent());
        nodeAlctr_->Free(popNode);
        if (bbt_->getLocalPoolSize(SolverID_ - 2) == 0) break;
        popNode = bbt_->localPoolPopFront(SolverID_ - 2);
      }

      if (popNode->GetTime() < (crntNode_->GetTime() + 1)) {
        bbt_->localPoolPushFront(SolverID_- 2,popNode);
      }
    }
    bbt_->localPoolUnlock(SolverID_ - 2);
  }
}

  bool fsbl = Enumerator::BackTrack_(trueState);

  if (trueState) {
    if (prune_.spillCost) {
      if (fsbl) {  
        assert(crntNode_->GetCostLwrBound() >= 0 || inst == rootNode_->GetInst());
        fsbl = crntNode_->GetCostLwrBound() < GetBestCost_();
      }
    }
  }

  if (!fsbl) {
    crntNode_->setIsInfsblFromBacktrack_(true);
    crntNode_->SetLocalBestCost(crntNode_->GetCostLwrBound());
  }

  return fsbl;
}
/*****************************************************************************/

void LengthCostEnumerator::BackTrackRoot_(EnumTreeNode *) {
  EnumTreeNode *tempNode = nullptr;
  if (bbt_->getStolenNode() != nullptr) tempNode = bbt_->getStolenNode();

  // if we have stolen work, need to backtrack against the victim threads active tree
  // use the stolen node to backtrack from
  Enumerator::BackTrackRoot_(tempNode);

  if (bbt_->getStolenNode() != nullptr) {
    // propogate information up the active tree of the victim thread, starting from
    // the parent of stolen node as the "crntNode"
    // note tempNode will never be the artifical root of victim tree, so we are safe to
    // propogate up to its parent
    // TODO(JEFF) it is possible that the work stealing thread has stolen from a work 
    // thread, in which case we will only propogote up to artifical root of the thief 
    // thread and never propogate that information to the master victim thread. To
    // resolve this -- we must have GetParent() return the stolen node when it makes
    // sense
    if (tempNode->GetParent()) {
      propogateExploration_(tempNode->GetParent());
    }
  }
}

void LengthCostEnumerator::propogateExploration_(EnumTreeNode *propNode) {
    EnumTreeNode *tmpTrgtNode = propNode->GetParent();
    EnumTreeNode *tmpCrntNode = propNode;

    assert(tmpCrntNode->wasChildStolen());
    if (tmpTrgtNode) tmpTrgtNode->setChildStolen(true);

    bool needsPropogation = false;
    bool fullyExplored = false;

    // It is possible that the parent node has already marked this child as fully explored even 
    // if the node has not been fully explored (because we chk cost feasibility during backtrack)
    // Therefore, we must be sure to not count the child as having been fully explored multiple
    // times, and ensure we do not increment the explored children of parent node if the crntNode
    // had previously became infeasible during backtracking
    if (IsHistDom()) {
      HistEnumTreeNode *crntHstry = tmpCrntNode->GetHistory();
      UDT_HASHVAL key = exmndSubProbs_->HashKey(tmpCrntNode->GetSig());
      bbt_->histTableLock(key);

      if (tmpCrntNode->getExploredChildren() == tmpCrntNode->getNumChildrn() && !tmpCrntNode->getIsInfsblFromBacktrack_()) {
        fullyExplored = needsPropogation = true;
        if (!tmpCrntNode->getIncrementedParent()) {
          if (tmpTrgtNode) tmpTrgtNode->incrementExploredChildren();      
          tmpCrntNode->setIncrementedParent(true);
        }
      } 
    
      // set fully explored to fullyExplored when work stealing
      crntHstry->setFullyExplored(fullyExplored);
      needsPropogation |= SetTotalCostsAndSuffixes(tmpCrntNode, tmpTrgtNode, trgtSchedLngth_,
                          prune_.useSuffixConcatenation, fullyExplored);
      tmpCrntNode->Archive(fullyExplored);
  #ifdef INSERT_ON_BACKTRACK
      if (!tmpCrntNode->getRecyclesHistNode()) {    
        exmndSubProbs_->InsertElement(tmpCrntNode->GetSig(), crntHstry,
                                hashTblEntryAlctr_, bbt_);
        crntHstry->setInserted(true);
      }
  #endif
      bbt_->histTableUnlock(key);
    }

    if (needsPropogation && !tmpCrntNode->isArtRoot() && tmpCrntNode->GetParent() != nullptr && tmpCrntNode->GetParent() != NULL) {
      propogateExploration_(tmpTrgtNode);
    }
}


void Enumerator::BackTrackRoot_(EnumTreeNode *tmpCrntNode) {

  if (tmpCrntNode == nullptr) {
    tmpCrntNode = crntNode_;
  }
  else {
    if (crntNode_->GetLocalBestCost() != INVALID_VALUE) tmpCrntNode->SetLocalBestCost(crntNode_->GetLocalBestCost());
    if (crntNode_->GetTotalCost() != INVALID_VALUE) tmpCrntNode->SetTotalCost(crntNode_->GetTotalCost());
    tmpCrntNode->SetCostLwrBound(crntNode_->GetCostLwrBound());
    tmpCrntNode->SetCost(crntNode_->GetCost());
    tmpCrntNode->SetHistory(crntNode_->GetHistory());
    tmpCrntNode->SetTotalCostIsActualCost(crntNode_->GetTotalCostIsActualCost());
  }
  //SchedInstruction *inst = tmpCrntNode->GetInst();
  EnumTreeNode *trgtNode = tmpCrntNode->GetParent();
  bool fullyExplored = false;
  

#ifdef INSERT_ON_BACKTRACK
  if (IsHistDom()) {
    if (!tmpCrntNode->getRecyclesHistNode()) assert(!tmpCrntNode->IsArchived());
    UDT_HASHVAL key = exmndSubProbs_->HashKey(tmpCrntNode->GetSig());
    HistEnumTreeNode *crntHstry = tmpCrntNode->GetHistory();

    bbt_->histTableLock(key);
    if (crntNode_->getExploredChildren() == crntNode_->getNumChildrn()) {
      if (trgtNode && !crntNode_->getIncrementedParent()) {
        trgtNode->incrementExploredChildren();
        crntNode_->setIncrementedParent(true);
      }
      fullyExplored = true;
    }
    // set fully explored to fullyExplored when work stealing
    crntHstry->setFullyExplored(fullyExplored);
    SetTotalCostsAndSuffixes(tmpCrntNode, trgtNode, trgtSchedLngth_,
                             prune_.useSuffixConcatenation, fullyExplored);
    tmpCrntNode->Archive(fullyExplored);
    crntNode_->setArchived(true);
    if (!tmpCrntNode->getRecyclesHistNode()) {
      exmndSubProbs_->InsertElement(tmpCrntNode->GetSig(), crntHstry,
                                  hashTblEntryAlctr_, bbt_);
      crntHstry->setInserted(true);
    }
  bbt_->histTableUnlock(key);
  }
  else {
    assert(tmpCrntNode->IsArchived() == false);
  }
#endif
#ifdef INSERT_ON_STEPFRWRD
  if (IsHistDom()) {
    UDT_HASHVAL key = exmndSubProbs_->HashKey(tmpCrntNode->GetSig());
    HistEnumTreeNode *crntHstry = tmpCrntNode->GetHistory();
    bbt_->histTableLock(key);
    if (crntNode_->getExploredChildren() == crntNode_->getNumChildrn()) {
      if (trgtNode && !crntNode_->getIncrementedParent()) {
        crntNode_->setIncrementedParent(true);
        trgtNode->incrementExploredChildren();
      }
      fullyExplored = true;
    }
    // set fully explored to fullyExplored when work stealing
    // TODO(jeff): it is possible that the crntHstry has been recycled and now belongs
    // to a different subspace
    crntHstry->setFullyExplored(fullyExplored);
    SetTotalCostsAndSuffixes(tmpCrntNode, trgtNode, trgtSchedLngth_,
                          prune_.useSuffixConcatenation, fullyExplored);
    tmpCrntNode->Archive(fullyExplored);
    crntNode_->setArchived(true);
    bbt_->histTableUnlock(key);
  }
#endif

  // This node belongs to the victim allocator, freeing it with this allocator makes it
  // available in both in subsequent regions (after victim thread resets its allocator)
  //if (!tmpCrntNode->wasChildStolen())
  //  nodeAlctr_->Free(tmpCrntNode);
  if (tmpCrntNode->wasChildStolen()) {
    if (trgtNode) trgtNode->setChildStolen(true);
  }
  

  if (bbt_->isWorkStealOn()) {
  // it is possible that a crntNode becomes infeasible before exploring all its children
  // thus we need to ensure that all children are removed on backtrack
    bbt_->localPoolLock(SolverID_ - 2);
    if (bbt_->getLocalPoolSize(SolverID_ - 2) > 0) {
      EnumTreeNode *popNode = bbt_->localPoolPopFront(SolverID_ - 2);
      assert(popNode);
#ifdef IS_CORRECT_LOCALPOOL

      if ((popNode->GetTime() > (crntNode_->GetTime() + 1)) || (popNode->GetTime() == crntNode_->GetTime() + 1 && popNode->GetParent() != crntNode_)) {
	  bbt_->GlobalPoolLock_->lock();
          Logger::Info("SolverID: %d\n", SolverID_);
	  int i = 0;
          auto temp = popNode;
          Logger::Info("popnode prefix (reverse)\n");
	  while (temp != NULL) {
            Logger::Info("%d", temp->GetInstNum());
	    ++i;
	    temp = temp->GetParent();
	  }

	  Logger::Info("Popped node has prefix length %d and time %d", i, popNode->GetTime()); 

          i = 0;
          temp = crntNode_;
          Logger::Info("crntnode prefix (reverse)\n");
          while (temp != NULL) {
            Logger::Info("%d", temp->GetInstNum());
            ++i;
            temp = temp->GetParent();
          }

          Logger::Info("Crnt node has prefix length %d and time %d", i, crntNode_->GetTime()); 
         bbt_->GlobalPoolLock_->unlock();
      }
#endif
      assert(popNode->GetTime() <= (crntNode_->GetTime() + 1));

      while (popNode->GetTime() >= (crntNode_->GetTime() + 1)) {
#ifdef IS_CORRECT_LOCALPOOL
bbt_->GlobalPoolLock_->lock();
	Logger::Info("SolverID %d,Removed element from localPool at time %d\n", SolverID_, popNode->GetTime());
bbt_->GlobalPoolLock_->unlock();

#endif
        //assert(popNode->GetParent() == crntNode_);
        // 
	//nodeAlctr_->Free(popNode);
        if (bbt_->getLocalPoolSize(SolverID_ - 2) == 0) break;
        popNode = bbt_->localPoolPopFront(SolverID_ - 2);
      }

      if (popNode->GetTime() < (crntNode_->GetTime() + 1)) {
        bbt_->localPoolPushFront(SolverID_- 2,popNode);
      }
    }
    bbt_->localPoolUnlock(SolverID_ - 2);
  }
}

InstCount LengthCostEnumerator::GetBestCost_() { return bbt_->getBestCost(); }
/*****************************************************************************/

void LengthCostEnumerator::CreateRootNode_() {
  rootNode_ = nodeAlctr_->Alloc(NULL, NULL, this);
#ifdef IS_CORRECT_LOCALPOOL
if (SolverID_ >= 2) {

   bbt_->GlobalPoolLock_->lock();
   Logger::Info("SolverID %d created node %d", SolverID_, rootNode_);
   bbt_->GlobalPoolLock_->unlock();
}
#endif
  CreateNewRdyLst_();
  rootNode_->SetRdyLst(rdyLst_);
  if (bbt_->isSecondPass())
    rootNode_->SetLwrBounds(DIR_FRWRD);

  assert(rsrvSlotCnt_ == 0);
  rootNode_->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);

  bbt_->setSttcLwrBounds(rootNode_);

  rootNode_->SetCost(0);
  rootNode_->SetCostLwrBound(0);
  rootNode_->SetTotalCost(0);

  InitNewNode_(rootNode_);
  CmtLwrBoundTightnng_();
}
/*****************************************************************************/


void LengthCostEnumerator::scheduleInt(int instNum, EnumTreeNode *newNode, bool isPseudoRoot, bool prune)
{
  newNode = NULL;
 
  //PROBEBRANCH
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

  newNode = nodeAlctr_->Alloc(crntNode_, inst, this);
#ifdef IS_CORRECT_LOCALPOOL
if (SolverID_ >= 2) {

   bbt_->GlobalPoolLock_->lock();
   Logger::Info("SolverID %d created node %d", SolverID_, newNode);
   bbt_->GlobalPoolLock_->unlock();
}
#endif
  if (bbt_->isSecondPass())
    newNode->SetLwrBounds(DIR_FRWRD);
  newNode->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);

  assert(newNode);

  ChkCostFsblty_(inst, newNode, false);

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


void LengthCostEnumerator::scheduleNode(EnumTreeNode *node, bool isPseudoRoot, bool prune) {

  EnumTreeNode *newNode = NULL;

 
  //PROBEBRANCH
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

  newNode = nodeAlctr_->Alloc(crntNode_, inst, this, false);
#ifdef IS_CORRECT_LOCALPOOL
if (SolverID_ >= 2) {

   bbt_->GlobalPoolLock_->lock();
   Logger::Info("SolverID %d created node %d", SolverID_, newNode);
   bbt_->GlobalPoolLock_->unlock();
}
#endif
  if (bbt_->isSecondPass())
    newNode->SetLwrBounds(DIR_FRWRD);
  newNode->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);

  assert(newNode);

  ChkCostFsblty_(inst, newNode, false);

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
bool LengthCostEnumerator::scheduleNodeOrPrune(EnumTreeNode *node,
                                               bool isPseudoRoot) {
  // scheduling function for state generation
  InstCount i;
  //bool isEmptyNode;
  SchedInstruction *inst;
  bool isFsbl = true;
  InstCount brnchCnt;
  bool found = false;

  rdyLst_->ResetIterator();
  brnchCnt = rdyLst_->GetInstCnt();

  // iterate until we find the node
  for (i = 0; i < brnchCnt; i++) {
    
    inst = rdyLst_->GetNextPriorityInst();
    if (inst->GetNum() == node->GetInstNum()) {
      found = true;
      // schedule its instruction
      // TODO -- currently we do not prune by history when scheduling the prefix
      // of a stolen node. Enabling history pruning results in overpruning and
      // mismatches
      scheduleInst_(inst, isPseudoRoot, isFsbl, false, false);
      if (!isFsbl) {
        return false;
      }
      break;
    }
  }
  assert(found);

  rdyLst_->ResetIterator();
  return true;
  
}
/*****************************************************************************/
bool LengthCostEnumerator::scheduleIntOrPrune(int instToSchdul,
                                               bool isPseudoRoot) {

#ifdef IS_DEBUG_SEARCH_ORDER
    Logger::Log((Logger::LOG_LEVEL)4, false, "SolverID %d probing global pool inst %d", SolverID_, instToSchdul);
#endif

  InstCount i;
  SchedInstruction *inst;
  bool isFsbl = true;
  InstCount brnchCnt = rdyLst_->GetInstCnt();
  bool flag = false;

  rdyLst_->ResetIterator();


  // iterate until we find the node
  for (i = 0; i < brnchCnt; i++) {
    inst = rdyLst_->GetNextPriorityInst();
    if (inst->GetNum() == instToSchdul) {
      flag = true;
      // schedule its instruction
      scheduleInst_(inst, isPseudoRoot, isFsbl);
      rdyLst_->ResetIterator();
      if (!isFsbl) {
        return false;
      }
      break;
    }
  }

  assert(flag);

#ifdef IS_DEBUG_SEARCH_ORDER
    Logger::Log((Logger::LOG_LEVEL)4, false, "SolverID %d stepping forward global pool inst %d", SolverID_, instToSchdul);
#endif


  return true;
}
/********************************************************************************/


EnumTreeNode *LengthCostEnumerator::scheduleInst_(SchedInstruction *inst, bool isPseudoRoot, bool &isFsbl, bool isRoot, bool prune) {
    // schedule the instruction (e.g. use probeBranch innareds to update state)

  EnumTreeNode *newNode;

  bool isNodeDominated = false, isRlxdFsbl = true, isLngthFsbl = true;
  isFsbl = ProbeBranch_(inst, newNode, isNodeDominated, isRlxdFsbl, isLngthFsbl, prune);

#ifdef DEBUG_GP_HISTORY
  if (isNodeDominated)
    Logger::Info("SolverID %d GlobalPool Node history dominated", SolverID_);
#endif
  if (!isFsbl)
    return nullptr;


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

  crntSched_->AppendInst(instNumToSchdul);

  MovToNxtSlot_(inst);
  assert(crntCycleNum_ <= trgtSchedLngth_);

  if (crntSlotNum_ == 0) {
    InitNewCycle_();
  }

  InitNewNode_(newNode);

#ifdef INSERT_ON_STEPFRWRD
  if (!isSecondPass()) {
    if (IsHistDom()) {
      if (!crntNode_->getRecyclesHistNode()) assert(!crntNode_->IsArchived());
        UDT_HASHVAL key = exmndSubProbs_->HashKey(crntNode_->GetSig());

      if (bbt_->isWorker() && IsFirstPass_) {
        HistEnumTreeNode *crntHstry = crntNode_->GetHistory();
        bbt_->histTableLock(key);
          crntHstry->setFullyExplored(false);
          crntHstry->setCostIsUseable(false);
          if (!crntNode_->getRecyclesHistNode()) {
            assert(!crntHstry->isInserted() || isSecondPass());
            exmndSubProbs_->InsertElement(crntNode_->GetSig(), crntHstry,
                                    hashTblEntryAlctr_, bbt_);
            crntHstry->setInserted(true);
          }
        bbt_->histTableUnlock(key);
      }

      else {
        HistEnumTreeNode *crntHstry = crntNode_->GetHistory();
        if (!crntNode_->getRecyclesHistNode()) {
          assert(!crntHstry->isInserted() || isSecondPass());
          exmndSubProbs_->InsertElement(crntNode_->GetSig(), crntHstry,
                                    hashTblEntryAlctr_, bbt_);
          crntHstry->setInserted(true);
        }
      }
        

    } else {
      assert(crntNode_->IsArchived() == false);
    }
  }
#endif


  if (isPseudoRoot) {
    newNode->setAsRoot(true);
    rootNode_ = newNode;
  }

  CmtLwrBoundTightnng_();
  ClearState_();
  

  return newNode;
}
/*****************************************************************************/
bool LengthCostEnumerator::scheduleArtificialRoot(bool setAsRoot) {
  assert(rdyLst_->GetInstCnt() == 1);
  SchedInstruction *inst = rdyLst_->GetNextPriorityInst();
  bool isFsbl = true;

  scheduleInst_(inst, setAsRoot, isFsbl, true, false);

  return isFsbl;

}
/*****************************************************************************/
void LengthCostEnumerator::scheduleAndSetAsRoot_(SchedInstruction *rootInst,
                                                 LinkedList<SchedInstruction> *frstList,
                                                 LinkedList<SchedInstruction> *scndList) {
  assert(rootInst != NULL);
  scheduleArtificialRoot();
  EnumTreeNode *newNode;


  // set the root node
  rootNode_ = newNode;
  crntNode_ = rootNode_;
}

/*****************************************************************************/
EnumTreeNode *LengthCostEnumerator::checkTreeFsblty(bool &fsbl) {
  assert(rootNode_ != NULL);
  SchedInstruction *inst = rdyLst_->GetNextPriorityInst();
  EnumTreeNode *newNode = scheduleInst_(inst, true, fsbl);
  return newNode;

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

/*****************************************************************************/
void LengthCostEnumerator::schedulePrefixInst_(SchedInstruction *instToSchdul, std::stack<InstCount> &costStack) {
  instToSchdul->Schedule(crntCycleNum_, crntSlotNum_, SolverID_);
  bbt_->schdulInst(instToSchdul, crntCycleNum_, crntSlotNum_, false);
  costStack.push(bbt_->getCrntPeakSpillCost());
  
  ConstrainedScheduler::SchdulInst_(instToSchdul, crntCycleNum_);

  MovToNxtSlot_(instToSchdul);
  if (crntSlotNum_ == 0) {
    InitNewCycle_();
  }

  UpdtRdyLst_(crntCycleNum_, crntSlotNum_);
}
/*****************************************************************************/
void LengthCostEnumerator::unschedulePrefixInst_(SchedInstruction *instToUnschdul, std::stack<InstCount> &costStack) {
  InstCount tempCost = costStack.top();
  costStack.pop();
  bbt_->unschdulInstAndRevert(instToUnschdul, crntCycleNum_, crntSlotNum_, tempCost);
  rdyLst_->RemoveLatestSubList();
  rdyLst_->AddInst(instToUnschdul, bbt_);
  MovToPrevSlot_(crntSlotNum_);
  ConstrainedScheduler::UnSchdulInst_(instToUnschdul);
  instToUnschdul->UnSchedule(SolverID_);

}

/*****************************************************************************/
void LengthCostEnumerator::splitNode(std::shared_ptr<HalfNode> &ExploreNode, InstPool4 *fillPool, int depth) {
  std::queue<int> tempPrefix;
  std::stack<SchedInstruction *> tempStack;
  std::stack<InstCount> costStack;
  SchedInstruction *tempInst = nullptr;
  int tempInstNum;
  ReadyList *originalRdyLst;

  originalRdyLst = new ReadyList(dataDepGraph_, prirts_, SolverID_);
  originalRdyLst->CopyList(rdyLst_);

  int prefixLength = 0;
  
  if (ExploreNode.get() != nullptr) {
    prefixLength = ExploreNode->getPrefixSize();
  }


  for (int i = 0; i < prefixLength; i++) {
    tempInstNum = ExploreNode->getAndRemoveNextPrefixInst();
    tempPrefix.push(tempInstNum);
    getAndRemoveInstFromRdyLst(tempInstNum, tempInst);
    schedulePrefixInst_(tempInst, costStack);
    removeInstFromRdyLst_(tempInstNum);
    tempStack.push(tempInst);
  }

  int rdyListSize = rdyLst_->GetInstCnt();

  for (int i = 0; i < rdyListSize; i++) {
    std::queue<int> tempPrefix2 = tempPrefix;
    unsigned long nextKey;
    SchedInstruction *temp = rdyLst_->GetNextPriorityInst(nextKey);
    unsigned long *heur;
    
    if (bbt_->getGlobalPoolSortMethod() == 1) {
      heur = new unsigned long[depth];
      heur[0] = nextKey;
      unsigned long *prevHeur = nullptr;
      if (ExploreNode.get() != nullptr) prevHeur = ExploreNode->getHeuristic();
      for (int i = 1; i < depth; i++) {
        assert(prevHeur != nullptr);
        heur[i] = prevHeur[i-1];
      }
    }
    else {
      heur = new unsigned long[1];
      heur[0] = nextKey;
    }

    bbt_->updateSpillInfoForSchdul(temp, false);
    tempPrefix2.push(temp->GetNum());

    fillPool->push(std::move(std::make_shared<HalfNode>(tempPrefix2, heur, bbt_->getCrntSpillCost())));
    bbt_->updateSpillInfoForUnSchdul(temp);
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

/*****************************************************************************/
void LengthCostEnumerator::getRdyListAsNodes(std::pair<EnumTreeNode *, unsigned long *> *ExploreNode, InstPool *pool, int depth) {
  std::stack<EnumTreeNode *> prefix;
  std::queue<EnumTreeNode *> subPrefix; 
  int prefixLength = 0;

  EnumTreeNode *node = ExploreNode->first;

  bool flag = false;
  if (node != rootNode_) {
    flag = true;

    prefixLength = node->getPrefixSize();

    if (prefixLength >= 1) {  // then we have insts to schedule
      for (int i = 0; i < prefixLength; i++) {
        EnumTreeNode *temp = node->getAndRemoveNextPrefixInst();
        subPrefix.push(temp);
        scheduleNode(temp, false, false);
        removeInstFromRdyLst_(temp->GetInstNum());
      }
    }
    scheduleNode(node, false, false);
    removeInstFromRdyLst_(node->GetInstNum());
    subPrefix.push(node);
  }

  // TODO -- delete stack
  
  std::pair<SchedInstruction *, unsigned long> nxtInst;
  int rdyListSize = rdyLst_->GetInstCnt();

  std::queue<std::pair<SchedInstruction *, unsigned long>> firstInstPool;

  for (int i = 0; i < rdyListSize; i++) {
    unsigned long nextKey;
    SchedInstruction *temp = rdyLst_->GetNextPriorityInst(nextKey);
    firstInstPool.push(std::make_pair(temp, nextKey));
  }

  rdyLst_->ResetIterator();

  for (int i = 0; i < rdyListSize; i++) {
    nxtInst = firstInstPool.front();
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

    unsigned long *heur;
    if (bbt_->getGlobalPoolSortMethod() == 1) {
      heur = new unsigned long[depth];
      heur[0] = nxtInst.second;
      for (int i = 1; i < depth; i++) {
        heur[i] = ExploreNode->second[i-1];
      }
    }
    else {
      heur = new unsigned long[1];
      heur[0] = nxtInst.second;
    }



    pool->push(std::make_pair(allocAndInitNextNode(nxtInst, node, pushNode, node->GetRdyLst(), subPrefix), heur));
    rdyLst_->AddInst(nxtInst.first, bbt_);
  }

  delete[] ExploreNode->second;

  SchedInstruction *inst = node->GetInst();

  if (flag) {
    bbt_->unschdulInst(inst, crntCycleNum_, crntSlotNum_, crntNode_->GetParent());

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

  if (state_.lwrBoundsTightnd && bbt_->isSecondPass()) {
    UnTightnLwrBounds_(inst);
  }
 
  UndoRsrvSlots_(inst);
  UnSchdulInst_(inst);
  inst->UnSchedule(SolverID_);


 for (int i = 0; i < prefixLength; i++)
    BackTrack_(false);
}

/*****************************************************************************/
ReadyList *LengthCostEnumerator::getGlobalPoolList(EnumTreeNode *newNode) {
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

/*****************************************************************************/
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


  //PROBEBRANCH 
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

  InitNode = nodeAlctr_->Alloc(parent, inst, this, false);
#ifdef IS_CORRECT_LOCALPOOL
if (SolverID_ >= 2) {

   bbt_->GlobalPoolLock_->lock();
   Logger::Info("SolverID %d created node %d", SolverID_, InitNode);
   bbt_->GlobalPoolLock_->unlock();
}
#endif

  InitNode->setPrefix(subPrefix);
  InitNode->setPrevNode(parent);
  if (bbt_->isSecondPass())
    InitNode->SetLwrBounds(DIR_FRWRD);
  InitNode->SetRsrvSlots(rsrvSlotCnt_, rsrvSlots_);

  assert(InitNode);

  ChkCostFsblty_(inst, InitNode, false);

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
    
    if (instToSchdul->GetTplgclOrdr() == minUnschduldTplgclOrdr_) {
      minUnschduldTplgclOrdr_++;
    }
  }

  crntSched_->AppendInst(instNumToSchdul);

  MovToNxtSlot_(instToSchdul);

  if (crntSlotNum_ == 0) {
    InitNewCycle_();
  }

  InitNewGlobalPoolNode_(newNode);

  CmtLwrBoundTightnng_();
  ClearState_();

  InitNode->setPriorityKey(InstNode.second);

  //BACKTRACK

  bbt_->unschdulInst(inst, crntCycleNum_, crntSlotNum_, crntNode_->GetParent());

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
  if (bbt_->isSecondPass())
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

  return InitNode;
}

/*****************************************************************************/
void LengthCostEnumerator::appendToRdyLst(LinkedList<SchedInstruction> *lst)
{
  rdyLst_->AddList(lst, bbt_);
}
/*****************************************************************************/

void LengthCostEnumerator::setRootRdyLst()
{
  rootNode_->SetRdyLst(rdyLst_);
}
/*****************************************************************************/

bool LengthCostEnumerator::EnumStall_() {
  if (!enblStallEnum_)
    return false;
  if (crntNode_->IsNxtSlotStall())
    return true;
  if (crntNode_ == rootNode_)
    return false;
  if (dataDepGraph_->IncludesUnpipelined())
    return true;
  return true;
}
/*****************************************************************************/

void LengthCostEnumerator::InitNewNode_(EnumTreeNode *newNode, bool setCost) {
  Enumerator::InitNewNode_(newNode, setCost);
}
/*****************************************************************************/

void LengthCostEnumerator::InitNewGlobalPoolNode_(EnumTreeNode *newNode) {
  Enumerator::InitNewGlobalPoolNode_(newNode);
}

/*****************************************************************************/
HistEnumTreeNode *LengthCostEnumerator::AllocHistNode_(EnumTreeNode *node, bool setCost) {
  CostHistEnumTreeNode *histNode = histNodeAlctr_->GetObject();

  histNode->Construct(node, false, isGenerateState_, setCost);
  return histNode;
}
/*****************************************************************************/

HistEnumTreeNode *LengthCostEnumerator::AllocTempHistNode_(EnumTreeNode *node) {
  HistEnumTreeNode *histNode = tmpHstryNode_;
  histNode->Construct(node, true, isGenerateState_, false);
  return histNode;
}

/*****************************************************************************/
void LengthCostEnumerator::FreeHistNode_(HistEnumTreeNode *histNode) {
  histNode->Clean();
  histNodeAlctr_->FreeObject((CostHistEnumTreeNode *)histNode);
}

/*****************************************************************************/
void LengthCostEnumerator::setLCEElements(BBThread *bbt, InstCount costLwrBound) {
  bbt_ = bbt;
  costLwrBound_ = costLwrBound;
}
