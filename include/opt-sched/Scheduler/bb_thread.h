#ifndef BB_THREAD_H
#define BB_THREAD_H

#include "opt-sched/Scheduler/OptSchedTarget.h"
#include "opt-sched/Scheduler/defines.h"
#include "opt-sched/Scheduler/gen_sched.h"
#include "opt-sched/Scheduler/sched_region.h"
#include "opt-sched/Scheduler/enumerator.h"
#include "llvm/ADT/SmallVector.h"
#include <map>
#include <set>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <atomic>
#include <stack>

namespace llvm {
namespace opt_sched {

class LengthCostEnumerator;
class EnumTreeNode;
class Register;
class RegisterFile;
class BitVector;



class InstPool4 {
private:
  std::queue<HalfNode *> pool;
  int maxSize_;
  int SortMethod_;
  int Depth_;
public:
  InstPool4();
  ~InstPool4();
  InstPool4(int SortMethod);
  void push(HalfNode *n) {pool.push(n);}
  int size() {return pool.size();}
  HalfNode *front() {return pool.front();}
  void pop() {pool.pop();}
  bool empty() {return pool.empty();}
  void sort();
  inline int getSortMethod() {return SortMethod_;}
  inline void setDepth(int Depth) {Depth_ = Depth;}
};


class InstPool {
private:
  std::queue<std::pair<EnumTreeNode *, unsigned long *>> pool;
  int maxSize_;
  int SortMethod_;
  int Depth_;
public:
  InstPool();
  ~InstPool();
  InstPool(int SortMethod);
  void push(std::pair<EnumTreeNode *, unsigned long *> n) {pool.push(n);}
  int size() {return pool.size();}
  std::pair<EnumTreeNode *, unsigned long *> front() {return pool.front();}
  void pop() {pool.pop();}
  bool empty() {return pool.empty();}
  void sort();
  inline int getSortMethod() {return SortMethod_;}
  inline void setDepth(int Depth) {Depth_ = Depth;}
};







class InstPool2 {
private:
  std::queue<EnumTreeNode *> *pool;
  int maxSize_;
public:
  InstPool2();
  void push(EnumTreeNode * n) {pool->push(n);}
  int size() {return pool->size();}
  EnumTreeNode *front() {return pool->front();}
  void pop() {pool->pop();}
  bool empty() {return pool->empty();}
  void sort();
  inline void setMaxSize(int maxSize) { maxSize_ = maxSize;}
  inline int getMaxSize() {return maxSize_;}
};


class InstPool3 {
private:
  LinkedList<EnumTreeNode> *pool;
  int maxSize_;
public:
  InstPool3();
  InstPool3(int size);
  ~InstPool3();
  void pushToFront(EnumTreeNode * n) {pool->InsrtElmntToFront(n);}
  void pushToBack(EnumTreeNode *n) {pool->InsrtElmnt(n);}
  int size() {return pool->GetElmntCnt();}
  EnumTreeNode *front() {return pool->GetHead();}
  EnumTreeNode *back() {return pool->GetTail();}
  void popFromFront() { pool->RmvElmnt(pool->GetHead(), false);}
  void popFromBack() {pool->RmvLastElmnt(false);}
  bool empty() {return pool->GetElmntCnt() == 0;}
  void removeSpecificElement(SchedInstruction *inst, EnumTreeNode *parent, EnumTreeNode *&removed);
  void sort();
  inline void setMaxSize(int maxSize) { maxSize_ = maxSize;}
  inline int getMaxSize() {return maxSize_;}

  inline LinkedListIterator<EnumTreeNode> end() {
    LinkedListIterator<EnumTreeNode> it (pool, pool->GetBottomEntry());
    return it;
  }
  inline LinkedListIterator<EnumTreeNode> begin() {return pool->begin();}
};



class BBThread {
private:
  // The target machine
  const OptSchedTarget *OST;

  int IssueRate;
  
  int EntryInstCnt_;
  int ExitInstCnt_;
  int NumberOfInsts_;

  

  // A bit vector indexed by register number indicating whether that
  // register is live
  WeightedBitVector *LiveRegs_;

  // A bit vector indexed by physical register number indicating whether
  // that physical register is live
  WeightedBitVector *LivePhysRegs_;

  // Sum of lengths of live ranges. This vector is indexed by register type,
  // and each type will have its sum of live interval lengths computed.
  std::vector<int> SumOfLiveIntervalLengths_;

  int SchduldEntryInstCnt_;
  int SchduldExitInstCnt_;
  int SchduldInstCnt_;

  InstCount *SpillCosts_;
  // Current register pressure for each register type.
  SmallVector<unsigned, 8> RegPressures_;
  InstCount *PeakRegPressures_;
  InstCount CrntStepNum_;
  InstCount PeakSpillCost_;
  InstCount TotSpillCost_;
  InstCount SlilSpillCost_;
  bool TrackLiveRangeLngths_;

  // TODO(max): Document.
  InstCount CrntCycleNum_;
  // TODO(max): Document.
  InstCount CrntSlotNum_;


    // BBWithSpill-specific Functions:
  InstCount CmputCostLwrBound_(InstCount schedLngth);
  void InitForCostCmputtn_();
  InstCount CmputDynmcCost_();

  
  void SetupPhysRegs_();
  void CmputCrntSpillCost_();
  void CmputCnflcts_(InstSchedule *sched);



public:
  BBThread(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
              long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
              SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
              bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
              bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
              SchedulerType HeurSchedType);
  virtual ~BBThread();

  int LocalPoolSizeRet = 0;
  SPILL_COST_FUNCTION SpillCostFuncBBT_;
  // non-virtual

  int CmputCostLwrBound();

  bool ChkCostFsblty(InstCount trgtLngth, EnumTreeNode *treeNode, bool isGlobalPoolNode = false);
  void SchdulInstBBThread(SchedInstruction *inst, InstCount cycleNum, InstCount slotNum,
                  bool trackCnflcts);
  void UnschdulInstBBThread(SchedInstruction *inst, InstCount cycleNum,
                    InstCount slotNum, EnumTreeNode *trgtNode);
  void UnschdulInstBBThread2(SchedInstruction *inst, InstCount cycleNum,
                    InstCount slotNum, InstCount prevPeakSpillCost);  
  void UpdateSpillInfoForUnSchdul_(SchedInstruction *inst);
  void setSttcLwrBounds(EnumTreeNode *node);
  bool ChkInstLgltyBBThread(SchedInstruction *inst);

  void InitForSchdulngBBThread();

  InstSchedule *allocNewSched_();

  void UpdateSpillInfoForSchdul_(SchedInstruction *inst, bool trackCnflcts);
  
  InstCount cmputNormCostBBThread_(InstSchedule *sched, COST_COMP_MODE compMode,
                           InstCount &execCost, bool trackCnflcts);

  inline InstCount getCrntSpillCost() {return CrntSpillCost_;}
  inline InstCount getCrntPeakSpillCost() {return PeakSpillCost_;}
  // virtuals
  virtual InstCount getBestCost() = 0;

  virtual InstCount UpdtOptmlSched(InstSchedule *crntSched,
                           LengthCostEnumerator *enumrtr) = 0;

  virtual bool isSecondPass() = 0;

  virtual bool isWorker() = 0;

  virtual void histTableLock(UDT_HASHVAL key) = 0;
  virtual void histTableUnlock(UDT_HASHVAL key) = 0;

  virtual void allocatorLock() = 0;
  virtual void allocatorUnlock() = 0;

  virtual std::mutex *getAllocatorLock() = 0;

  virtual void incrementImprvmntCnt() = 0;
  
  virtual bool isWorkSteal() = 0;
  virtual bool isWorkStealOn() = 0;
  virtual void setWorkStealOn(bool) = 0;
  // Thread stealing
  //virtual LinkedListIterator<EnumTreeNode> localPoolBegin(int SolverID) = 0;
  //virtual LinkedListIterator<EnumTreeNode> localPoolEnd(int SolverID) = 0 ;    

  virtual void localPoolLock(int SolverID) = 0;
  virtual void localPoolUnlock(int SolverID) = 0;

  virtual void localPoolPushFront(int SolverID, EnumTreeNode *ele) = 0;
  virtual EnumTreeNode *localPoolPopFront(int SolverID) = 0;

  virtual void localPoolPushTail(int SolverID, EnumTreeNode *ele) = 0;
  virtual EnumTreeNode *localPoolPopTail(int SolverID) = 0;

  virtual int getLocalPoolSize(int SolverID) = 0;
  virtual int getLocalPoolMaxSize(int SolverID) = 0;

  virtual EnumTreeNode *viewLocalPoolFront(int SolverID) = 0;

  virtual void localPoolRemoveSpecificElement(int SolverID, SchedInstruction *inst, 
                                              EnumTreeNode *parent, EnumTreeNode *&removed) = 0;


  virtual int getGlobalPoolSortMethod() {return -1;};
  // Needed by aco
  virtual InstCount getHeuristicCost() = 0;

  void SetupForSchdulngBBThread_();
  

protected:
  LengthCostEnumerator *Enumrtr_;
  InstCount CrntSpillCost_;
  InstCount OptmlSpillCost_;

  DataDepGraph *DataDepGraph_;
  MachineModel  *MachMdl_; 

  int SolverID_;

  bool SchedForRPOnly_;

  bool EnblStallEnum_;
  
  bool VrfySched_;

  int SCW_;
  int SchedCostFactor_;

  InstCount MaxLatency_;
  bool SimpleMachineModel_;

  int16_t RegTypeCnt_;
  RegisterFile *RegFiles_;

  InstCount StaticSlilLowerBound_ = 0;
  InstCount DynamicSlilLowerBound_ = 0;
  InstCount StaticLowerBound_ = 0;
  
  uint64_t SubspaceLwrBound_ = INVALID_VALUE;

  // Needed to override SchedRegion virtuals
  bool EnableEnumBBThread_();

  InstCount CmputCost_(InstSchedule *sched, COST_COMP_MODE compMode,
                       InstCount &execCost, bool trackCnflcts);
  

  void FinishOptmlBBThread_();

  bool ChkScheduleBBThread_(InstSchedule *bestSched, InstSchedule *lstSched);

  // Returns the static lower bound
  inline int getCostLwrBound() {return StaticLowerBound_;};

  // Virtual Functions:
  virtual void setBestCost(InstCount BestCost) = 0;


};

/******************************************************************/

class BBInterfacer : public SchedRegion, public BBThread {
private:
    void CmputAbslutUprBound_();

    InstCount CmputCostLwrBound();

protected:
    InstCount *BestCost_;
    InstCount *CostLwrBound_;

    int NumSolvers_;

    void CmputSchedUprBound_();

      // override SchedRegion virtual
    void InitForSchdulng() override {return InitForSchdulngBBThread();}

    void SetupForSchdulng_() override {return SetupForSchdulngBBThread_();}

    bool ChkInstLglty(SchedInstruction *inst) override
    {
      return ChkInstLgltyBBThread(inst);
    }

    bool ChkSchedule_(InstSchedule *bestSched, InstSchedule *lstSched) override
    {
      return ChkScheduleBBThread_(bestSched, lstSched);
    }

    bool EnableEnum_() override
    {
      return EnableEnumBBThread_();
    }

    void FinishOptml_() override
    {
      return FinishOptmlBBThread_();
    }

  // override BBThread virtual
  InstCount getBestCost() override {return *BestCost_;}
 
  void setBestCost(InstCount BestCost) override { *BestCost_ = BestCost; }

  InstCount UpdtOptmlSched(InstSchedule *crntSched,
                             LengthCostEnumerator *enumrtr);


public:
    BBInterfacer(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
              long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
              SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
              bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
              bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
              SchedulerType HeurSchedType);


    inline void SchdulInst(SchedInstruction *inst, InstCount cycleNum, InstCount slotNum,
                  bool trackCnflcts)
    {
      SchdulInstBBThread(inst, cycleNum, slotNum, trackCnflcts);
    }

    inline void UnschdulInst(SchedInstruction *inst, InstCount cycleNum,
                    InstCount slotNum, EnumTreeNode *trgtNode)
    {
      UnschdulInstBBThread(inst, cycleNum, slotNum, trgtNode);
    }

    inline InstCount CmputNormCost_(InstSchedule *sched, COST_COMP_MODE compMode,
                           InstCount &execCost, bool trackCnflcts)
    {
      return cmputNormCostBBThread_(sched, compMode, execCost, trackCnflcts);
    }

    static InstCount ComputeSLILStaticLowerBound(int64_t regTypeCnt_,
                                                 RegisterFile *regFiles_, 
                                                 DataDepGraph *dataDepGraph_);

    bool isSecondPass() override { return isSecondPass_; }

    bool isWorker() override {return false;}

    void histTableLock(UDT_HASHVAL key) override {/*nothing*/; }
    void histTableUnlock(UDT_HASHVAL key) override {/*nothing*/; }

    void incrementImprvmntCnt() override {/*nothing*/;}

    void allocatorLock() override {/*nothing*/;}
    void allocatorUnlock() override {/*nothing*/;}

    std::mutex *getAllocatorLock() override;

    // todo -- return nullptr
    //LinkedListIterator<EnumTreeNode> localPoolBegin(int SolverID) override {/*return nullptr;*/;}
    //LinkedListIterator<EnumTreeNode> localPoolEnd(int SolverID) override {/*return nullptr;*/;}   

    void localPoolLock(int SolverID) override {/*nothing*/;}
    void localPoolUnlock(int SolverID) override {/*nothing*/;}

    void localPoolPushFront(int SolverID, EnumTreeNode *ele) override {/*nothing*/;}
    EnumTreeNode *localPoolPopFront(int SolverID) override {return nullptr;}

    void localPoolPushTail(int SolverID, EnumTreeNode *ele) override {/*nothing*/;}
    EnumTreeNode *localPoolPopTail(int SolverID) override {return nullptr;}

    EnumTreeNode *viewLocalPoolFront(int SolverID) override {return nullptr;}

    int getLocalPoolSize(int SolverID) override {return INVALID_VALUE;}
    int getLocalPoolMaxSize(int SolverID) override {return INVALID_VALUE;}

    virtual void localPoolRemoveSpecificElement(int SolverID, SchedInstruction *inst, 
                                                EnumTreeNode *parent, 
                                                EnumTreeNode *&removed) override {/*nothing*/}




    inline InstCount getHeuristicCost() {return GetHeuristicCost();}

};

/******************************************************************/
class BBWithSpill : public BBInterfacer {
private:

protected:
  int timeoutToMemblock_;
public:
    BBWithSpill(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
                long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
                SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
                bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
                bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
                SchedulerType HeurSchedType, int timeoutToMemblock);

    
    FUNC_RESULT Enumerate_(Milliseconds startTime, Milliseconds rgnTimeout,
                           Milliseconds lngthTimeout, int *OptimalSolverID) override;

    Enumerator *AllocEnumrtr_(Milliseconds timeout);

    uint64_t getExaminedNodeCount() override {return Enumrtr_->GetNodeCnt(); }

    inline bool isWorkSteal() override {return false;}
    inline bool isWorkStealOn() override {
      return false;
    }

    void setWorkStealOn(bool value) override {/*nothing*/};

};


/******************************************************************/
class BBWorker : public BBThread {
private:

    InstCount SchedUprBound_;   // set by master (using schedRegion interface)
    int16_t SigHashSize_;       // set by master (using schedRegion interface)

    int NumSolvers_;


    Pruning PruningStrategy_;
    SchedPriorities EnumPrirts_;
    SPILL_COST_FUNCTION SpillCostFunc_;

    //vector<int> TakenArr;
    //vector<BBWorker> *local_pool = NULL;

    vector<FUNC_RESULT> *RsltAddr_;


    InstSchedule *EnumCrntSched_;
    InstSchedule *EnumBestSched_;




    // local variable holding cost of best schedule for current enumerator
    InstCount BestCost_;
    // cost of the heuristic schedule
    InstCount HeuristicCost_;
    // lower bound of schedule length
    InstCount SchedLwrBound_;

    // shared variable of best schedule
    InstSchedule *MasterSched_;
    // shared variable of the best cost found so far
    InstCount *MasterCost_;       
    // shared variable of the best spill cost found so far
    InstCount *MasterSpill_;
    // share variable of the best sched elgnth found so far
    InstCount *MasterLength_;

    // the best found schedule for the region
    InstSchedule *RegionSched_;

    // TODO replace Count with Cnt
    uint64_t *NodeCount_;
    int *MasterImprvCount_;

    // are we in the second apss
    bool IsSecondPass_;

    // A reference to the shared GlobalPool
    InstPool4 *GlobalPool_;

    vector<InstPool3 *> localPools_;
    std::mutex **localPoolLocks_;

    // References to the locks on shared data
    std::mutex **HistTableLock_;
    std::mutex *GlobalPoolLock_; 
    std::mutex *BestSchedLock_;
    std::mutex *NodeCountLock_;
    std::mutex *ImprvmntCntLock_;
    std::mutex *RegionSchedLock_;
    std::mutex *AllocatorLock_;
    std::mutex *InactiveThreadLock_;

    int *IdleTime_;
    int *InactiveThreads_;
    uint64_t *nodeCounts_;

    bool WorkSteal_;
    bool *WorkStealOn_;
    uint64_t **subspaceLwrBounds_;

    bool IsTimeoutPerInst_;
    int timeoutToMemblock_;

    void handlEnumrtrRslt_(FUNC_RESULT rslt, InstCount trgtLngth);

    // overrides
    inline InstCount getBestCost() {return *MasterCost_;}
    inline void setBestCost(InstCount BestCost) {
      BestCost_ = BestCost;
      }

    // needs to write to master
    InstCount UpdtOptmlSched(InstSchedule *crntSched, LengthCostEnumerator *enumrtr);

    void writeBestSchedToMaster(InstSchedule *BestSchedule, InstCount BestCost, InstCount BestSpill);

public:
    BBWorker(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
              long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
              SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
              bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
              bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
              SchedulerType HeurSchedType, bool IsSecondPass, 
              InstSchedule *MasterSched, InstCount *MasterCost, 
              InstCount *MasterSpill, InstCount *MasterLength, 
              InstPool4 *GlobalPool, 
              uint64_t *NodeCount, int SolverID, std::mutex **HistTableLock, 
              std::mutex *GlobalPoolLock, std::mutex *BestSchedLock, std::mutex *NodeCountLock,
              std::mutex *ImprCountLock, std::mutex *RegionSchedLock, std::mutex *AllocatorLock,
              vector<FUNC_RESULT> *resAddr, int *idleTimes, int NumSolvers, std::vector<InstPool3 *> localPools, 
              std::mutex **localPoolLocks, int *inactiveThreads, std::mutex *inactiveThreadLock, 
              int LocalPoolSize, bool WorkSteal, bool *WorkStealOn, bool IsTimeoutPerInst, uint64_t *nodeCounts,
              int timeoutToMemblock, uint64_t **subspaceLwrBounds);

    ~BBWorker();
    /*
    BBWorker (const BBWorker&) = delete;
    BBWorker& operator= (const BBWorker&) = delete;
    */

    inline SchedInstruction *GetInstByIndex(InstCount index) {return Enumrtr_->GetInstByIndx(index);}

    void setHeurInfo(InstCount SchedUprBound, InstCount HeuristicCost, InstCount SchedLwrBound);

    void allocEnumrtr_(Milliseconds timeout, std::mutex *AllocatorLock);
    void initEnumrtr_(bool scheduleRoot = true);
    void setLCEElements_(InstCount costLwrBound);
    void setLowerBounds_(InstCount costLwrBound);
    inline void setEnumHistTable(BinHashTable<HistEnumTreeNode> *histTable)  {
      Enumrtr_->setHistTable(histTable);
    }

    void allocSched_();

    inline void destroy() {Enumrtr_->destroy();}

    void setBestSched(InstSchedule *sched);
    void setCrntSched(InstSchedule *sched);

    inline bool scheduleArtificialRoot(bool setAsRoot = false) {return Enumrtr_->scheduleArtificialRoot(setAsRoot);}
    
    inline void scheduleAndSetAsRoot(SchedInstruction *inst, 
                                     LinkedList<SchedInstruction> *frstList,
                                     LinkedList<SchedInstruction> *scndList) { 
      Enumrtr_->scheduleAndSetAsRoot_(inst, frstList, scndList);
    }
    
    inline InstCount getRootInstNum() {return Enumrtr_->getRootInstNum();}

    inline int getSolverID() {return SolverID_;}

    inline void appendToRdyLst(LinkedList<SchedInstruction> *lst) {
      Enumrtr_->appendToRdyLst(lst);
    }

    inline void setRootRdyLst() {Enumrtr_->setRootRdyLst();}

    bool generateStateFromNode(EnumTreeNode *GlobalPoolNode, bool isGlobalPoolNode = true);

    bool generateStateFromNode(HalfNode *GlobalPoolNode);

    FUNC_RESULT enumerate_(EnumTreeNode *GlobalPoolNode, Milliseconds StartTime, 
                           Milliseconds RgnTimeout, Milliseconds LngthTimeout, 
                           bool isWorkStealing = false, bool isNodeFsbl = true);

    FUNC_RESULT enumerate_(Milliseconds StartTime, 
                           Milliseconds RgnTimeout, Milliseconds LngthTimeout,
                           bool isWorkStealing = false, bool isNodeFsbl = true);

    FUNC_RESULT generateAndEnumerate(HalfNode *GlobalPoolNode, Milliseconds StartTime, 
                                     Milliseconds RgnTimeout, Milliseconds LngthTimeout);

    //TODO - clean this up
    inline InstCount CmputNormCost_(InstSchedule *sched, COST_COMP_MODE compMode,
                           InstCount &execCost, bool trackCnflcts)
    {
      return cmputNormCostBBThread_(sched, compMode, execCost, trackCnflcts);
    }

    bool isSecondPass() override { return IsSecondPass_;}

    bool isWorker() override {return true;}

    inline InstCount getHeuristicCost() {return HeuristicCost_;}

    inline void setCostLowerBound(InstCount StaticLowerBound) {
      StaticLowerBound_ = StaticLowerBound;
    }

    inline void setMasterSched(InstSchedule *MasterSched) {MasterSched_ = MasterSched;}
    inline void setMasterImprvCount(int *ImprvCount) {MasterImprvCount_ = ImprvCount; }

    inline void setRegionSchedule(InstSchedule *RegionSched) {RegionSched_ = RegionSched;}

    void histTableLock(UDT_HASHVAL key) override;
    void histTableUnlock(UDT_HASHVAL key) override; 

    inline bool isWorkSteal() override {return WorkSteal_;}
    inline bool isWorkStealOn() override {
      return *WorkStealOn_;}
    inline void setWorkStealOn(bool value) override {*WorkStealOn_ = value;}
    

    void allocatorLock() override;
    void allocatorUnlock() override;

    std::mutex *getAllocatorLock() override;

    void incrementImprvmntCnt() override;

    // may need to not inline these
    //LinkedListIterator<EnumTreeNode> localPoolBegin(int SolverID) override;
    //LinkedListIterator<EnumTreeNode> localPoolEnd(int SolverID) override;  

    void localPoolLock(int SolverID) override;
    void localPoolUnlock(int SolverID) override;

    void localPoolPushFront(int SolverID, EnumTreeNode *ele) override;
    EnumTreeNode *localPoolPopFront(int SolverID) override;

    void localPoolPushTail(int SolverID, EnumTreeNode *ele) override;
    EnumTreeNode *localPoolPopTail(int SolverID) override;

    EnumTreeNode *viewLocalPoolFront(int SolverID) override {return localPools_[SolverID]->front();};

    int getLocalPoolSize(int SolverID) override;
    int getLocalPoolMaxSize(int SolverID) override;

    virtual void localPoolRemoveSpecificElement(int SolverID, SchedInstruction *inst, 
                                                EnumTreeNode *parent, 
                                                EnumTreeNode *&removed) override;

    inline void freeAlctrs() {Enumrtr_->FreeAllocators_();}

};

/******************************************************************/

class BBMaster : public BBInterfacer {
private:
    vector<BBWorker *> Workers;
    vector<std::thread> ThreadManager;
    InstPool4 *GlobalPool; 
    int firstLevelSize_;
    int NumThreads_;
    int MinNodesAsMultiple_,MinSplittingDepth_, MaxSplittingDepth_;
    uint64_t MasterNodeCount_;
    vector<FUNC_RESULT> results;

    int InactiveThreads_;
    int NumThreadsToLaunch_;

    std::mutex **HistTableLock;
    std::mutex GlobalPoolLock;
    std::mutex BestSchedLock;
    std::mutex NodeCountLock;
    std::mutex ImprvCountLock;
    std::mutex RegionSchedLock;
    std::mutex AllocatorLock;
    std::mutex InactiveThreadLock;

    int64_t HistTableSize_;

    int *idleTimes;
    uint64_t *nodeCounts;

    std::vector<InstPool3 *> localPools;
    std::mutex **localPoolLocks;

    int LocalPoolSize_;
    float ExploitationPercent_;
    SPILL_COST_FUNCTION GlobalPoolSCF_;
    int GlobalPoolSort_;

    bool WorkSteal_;
    bool WorkStealOn_;
    uint64_t **subspaceLwrBounds_;
    
    int timeoutToMemblock_;

    void initWorkers(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
             long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
             SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
             bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
             bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
             SchedulerType HeurSchedType, InstCount *BestCost, InstCount SchedLwrBound,
             InstSchedule *BestSched, InstCount *BestSpill, 
             InstCount *BestLength, InstPool4 *GlobalPool, 
             uint64_t *NodeCount,  std::mutex **HistTableLock, std::mutex *GlobalPoolLock, std::mutex *BestSchedLock, 
             std::mutex *NodeCountLock, std::mutex *ImprvCountLock, std::mutex *RegionSchedLock, 
             std::mutex *AllocatorLock, vector<FUNC_RESULT> *results, int *idleTimes,
             int NumSolvers, std::vector<InstPool3 *> localPools, std::mutex **localPoolLocks,
             int *InactiveThreads_, std::mutex *InactiveThreadLock, int LocalPoolSize, bool WorkSteal, 
             bool *WorkStealOn, bool IsTimeoutPerInst, uint64_t *nodeCounts, int timeoutToMemblock, uint64_t **subspaceLwrBounds);

  
    bool initGlobalPool();
    bool init();
    void setWorkerHeurInfo();
    Enumerator *allocEnumHierarchy_(Milliseconds timeout, bool *fsbl);

    inline BinHashTable<HistEnumTreeNode> *getEnumHistTable() {
      return Enumrtr_->getHistTable(); 
    }

public:
    BBMaster(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
             long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
             SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
             bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
             bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
             SchedulerType HeurSchedType, int NumThreads, int MinNodesAsMultiple, 
             int MinSplittingDepth,
             int MaxSplittingDepth, int NumSolvers, int LocalPoolSize, float ExploitationPercent,
             SPILL_COST_FUNCTION GlobalPoolSCF, int GlobalPoolSort, bool WorkSteal, bool IsTimeoutPerInst,
             int timeoutToMemblock);

    ~BBMaster();
    
    BBMaster (const BBMaster&) = delete;
    BBMaster& operator= (const BBMaster&) = delete;

    Enumerator *AllocEnumrtr_(Milliseconds timeout);


    FUNC_RESULT Enumerate_(Milliseconds startTime, Milliseconds rgnTimeout,
                           Milliseconds lngthTimeout, int *OptimalSolverID) override;

    
    uint64_t getExaminedNodeCount() override {return MasterNodeCount_; }

    int getGlobalPoolSortMethod() override {return GlobalPool->getSortMethod();}

    inline bool isWorkSteal() override {return WorkSteal_;}
    inline bool isWorkStealOn() override {return WorkStealOn_;}

    void setWorkStealOn(bool value) override {/*nothing*/};


//TODO: destructors, handle resource allocation & deaallocation
//cleanup the virtual funcs in sched_region
    

};


} //optsched namespace
} //llvm namespace

#endif
