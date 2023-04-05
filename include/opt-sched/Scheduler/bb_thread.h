/*******************************************************************************
Description:  This interface allows the enumerator class to generate schedules
              based on spill clost by offering access to not only the register file
              but also schedule costs found in SchedRegion (e.g. from list or ACO).
              
              This interface is also the point of parallelization for the branch
              and bound scheduling algorithm. The bb_thread class is a pure virtual
              class containing the common subset of methods and members. Deriving
              from this are: BBInterfacer, and BBWorker. The BBInterfacer
              class interfaces with SchedRegion in order to obtain
              things like the best schedule and cost found so far, BBWithSpill and
              BBMaster derive from BBInterfacer. The BBWithSpill class implements
              the single-threaded (sequential) algorithm, whereas the BBMaster class
              spawns a number of BBWorker to explore the solution space in parallel.
Author:       Jeffrey Byrnes (JrByrnes1989@gmail.com)
Created:      Jan. 2021
Last Update:  Jan. 2022
*******************************************************************************/


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
#include <iostream>
#include <fstream>

namespace llvm {
namespace opt_sched {

class LengthCostEnumerator;
class EnumTreeNode;
class Register;
class RegisterFile;
class BitVector;


// TODO rename -- diversity pools
class InstPool4 {
private:
  std::queue<std::shared_ptr<HalfNode>> pool;
  int SortMethod_;
  int Depth_;
public:
  InstPool4();
  ~InstPool4();
  InstPool4(int SortMethod);
  void push(std::shared_ptr<HalfNode> n) {pool.push(n);}
  int size() {return pool.size();}
  std::shared_ptr<HalfNode> front() {return pool.front();}
  void pop() {pool.pop();}
  bool empty() {return pool.empty();}
  void sort();
  inline int getSortMethod() {return SortMethod_;}
  inline void setDepth(int Depth) {Depth_ = Depth;}
};


class InstPool {
private:
  std::queue<std::pair<EnumTreeNode *, unsigned long *>> pool;
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


// TODO Document
// BBThread contains the minimum required interface from the Enumerator point of view.
// It is a pure virtual class from which our workers, master, and the single threaded
// classes derive.
class BBThread {
private:
  // The target machine
  const OptSchedTarget *OST;

  int IssueRate_;
  
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
  InstCount cmputCostLwrBound_(InstCount schedLngth);
  void initForCostCmputtn_();
  InstCount cmputDynmcCost_();

  
  void setupPhysRegs_();
  void cmputCrntSpillCost_();
  void cmputCnflcts_(InstSchedule *sched);



public:
  BBThread(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
              long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
              SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
              bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
              bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
              SchedulerType HeurSchedType);
  virtual ~BBThread();
  std::mutex *GlobalPoolLock_;
  std::ofstream ThreadStream_;

  // Stats on the number of nodes examined
  // Number of calls to stepfrwrd
  uint64_t StepFrwrds = 0;
  // NUmber of calls to backtrack
  uint64_t BackTracks = 0;
  // Number of cost infsbl insts
  uint64_t CostInfsbl = 0;
  // Number of hist infsbl insts
  uint64_t HistInfsbl = 0;
  // Number of other infsbl ints
  uint64_t OtherInfsbl = 0;
  // Global Pool Nodes explored
  uint64_t GlobalPoolNodes = 0;

  int *RegCrntUseCnts;
  int *RegNums;
  int16_t *RegTypes;

  struct RegFields {
    int CrntUseCnt;
    int Num;
    int Type;
  };

  DenseMap<llvm::opt_sched::Register *, RegFields> RegToFields;

  void resetRegFields();
  // Allocate register structures needed to track cost
  void setupForSchdulng();
  // Initialize cost and register information (e.g register pressure)
  void initForSchdulng();
  // Not Implemented
  void setSttcLwrBounds(EnumTreeNode *node);
  // Allocate schedule of instructions
  InstSchedule *allocNewSched();
  // Set schedule cycle / slot and update cost info
  void schdulInst(SchedInstruction *inst, InstCount cycleNum, InstCount slotNum,
                  bool trackCnflcts);
  // Update register uses and defs for cost computation
  void updateSpillInfoForSchdul(SchedInstruction *inst, bool trackCnflcts);
  // Unset schedule cycle / slot and update cost info
  void unschdulInst(SchedInstruction *inst, InstCount cycleNum,
                    InstCount slotNum, EnumTreeNode *trgtNode);
  // Unset schedule cycle / slot and revert cost to value passed in
  // This is primarily used when we are not maintaining the active tree 
  // (e.g. there is no trgtNode to grab the cost from)
  void unschdulInstAndRevert(SchedInstruction *inst, InstCount cycleNum,
                    InstCount slotNum, InstCount prevPeakSpillCost);
  // Revert register uses and defs to undo changes to cost
  void updateSpillInfoForUnSchdul(SchedInstruction *inst);
  // Compute cost and "normalize" it (i.e. subtract the lower bound)
  InstCount cmputNormCost(InstSchedule *sched, COST_COMP_MODE compMode,
                          InstCount &execCost, bool trackCnflcts);
  // Check if the partial schedule does not violate cost constraint
  bool chkCostFsblty(InstCount trgtLngth, EnumTreeNode *&treeNode, bool isGlobalPoolNode = false);
  // Not Implemented
  bool chkInstLgltyBBThread(SchedInstruction *inst);
  inline RegFields getRegFields(Register *reg) {return RegToFields[reg];}
  // Returns the spill cost from last partial schedule cost calculation
  inline InstCount getCrntSpillCost() {return CrntSpillCost_;}
  // Returns the peak spill cost from last partial schedule cost calculation
  inline InstCount getCrntPeakSpillCost() {return PeakSpillCost_;}
  // Whether or not we are using two pass version of the algorithm
  inline bool getIsTwoPass() {return TwoPassEnabled_;}
  // VIRTUAL FUNCTIONS
  // Are we currently scheduling for ILP
  virtual bool isSecondPass() = 0;   // TODO(jeff) make this a non-virtual function
  // The cost of heuristic schedule -- needed by ACO
  virtual InstCount getHeuristicCost() = 0; 
  // Returns the best cost found from scheduling
  virtual InstCount getBestCost() = 0;
  // Returns the current sched cost
  virtual InstCount getCrntScheduleCost() = 0;
  // Updates the current schedule with an improved cost schedule
  virtual InstCount UpdtOptmlSched(InstSchedule *crntSched,
                           LengthCostEnumerator *enumrtr) = 0;
  // Synchronized increment the schedule improvmeent count
  virtual void incrementImprvmntCnt() = 0;
  // Is the current instance a worker in master-worker parallel architecture
  virtual bool isWorker() = 0;
    // What method are we using to sort the global pool
  virtual int getGlobalPoolSortMethod() {return -1;}; // TODO(jeff) (use enum for return vals)
  // Mutex lock the hist table
  virtual void histTableLock(UDT_HASHVAL key) = 0;
  // Mutex unlock the hist table
  virtual void histTableUnlock(UDT_HASHVAL key) = 0;
  // Mutex lock the local pool
  virtual void localPoolLock(int SolverID) = 0;
  // Mutex unlock the local pool
  virtual void localPoolUnlock(int SolverID) = 0;
  // Whether or not we are allowing work stealing feature 
  virtual bool isWorkSteal() = 0;
  // Whether or not a thread has run out of work and turned work stealing on
  virtual bool isWorkStealOn() = 0;
  // Used to toggle work steal on/off
  virtual void setWorkStealOn(bool) = 0;
  // Return the node that the current instance stole from another worker
  virtual EnumTreeNode *getStolenNode() = 0;
  

  // Interface / modifiers for the local pool
  virtual void localPoolPushFront(int SolverID, EnumTreeNode *ele) = 0;
  virtual EnumTreeNode *localPoolPopFront(int SolverID) = 0;

  virtual void localPoolPushTail(int SolverID, EnumTreeNode *ele) = 0;
  virtual EnumTreeNode *localPoolPopTail(int SolverID) = 0;

  virtual int getLocalPoolSize(int SolverID) = 0;
  virtual int getLocalPoolMaxSize(int SolverID) = 0;

  virtual EnumTreeNode *viewLocalPoolFront(int SolverID) = 0;

  virtual void localPoolRemoveSpecificElement(int SolverID, SchedInstruction *inst, 
                                              EnumTreeNode *parent, EnumTreeNode *&removed) = 0;


protected:
  LengthCostEnumerator *Enumrtr_;
  InstCount CrntSpillCost_;
  InstCount OptmlSpillCost_;

  DataDepGraph *DataDepGraph_;
  MachineModel  *MachMdl_; 

  // The SolverID_ for the current solver
  // Workers range from 2 : nThread + 2 (0 and 1 are taken by list and master respectively)
  int SolverID_;

  bool TwoPassEnabled_;
  bool SchedForRPOnly_;
  bool EnblStallEnum_;
  bool VrfySched_;

  int SCW_;
  int SchedCostFactor_;
  // The spill cost function used for enumeration
  SPILL_COST_FUNCTION SpillCostFunc_;

  InstCount MaxLatency_;
  bool SimpleMachineModel_;

  int16_t RegTypeCnt_;
  RegisterFile *RegFiles_;

  InstCount StaticSlilLowerBound_ = 0;
  InstCount DynamicSlilLowerBound_ = 0;
  InstCount StaticLowerBound_ = 0;
  
  // SubspaceLwrBound tracks the current cost of a thread as a measure to determine
  // how promising a search space is (for victimizing threads in work stealing)
  int64_t SubspaceLwrBound_ = INVALID_VALUE;

  

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
    void CmputAbslutUprBound_() override;

    InstCount cmputCostLwrBound() override;

protected:
    InstCount *BestCost_;
    InstCount *CostLwrBound_;

    int NumSolvers_;

    void CmputSchedUprBound_() override;

      // override SchedRegion virtual
    void InitForSchdulng() override {return initForSchdulng();}

    void SetupForSchdulng_() override {return setupForSchdulng();}

    bool ChkInstLglty(SchedInstruction *inst) override {
      return chkInstLgltyBBThread(inst);
    }

    bool ChkSchedule_(InstSchedule *bestSched, InstSchedule *lstSched) override {
      return ChkScheduleBBThread_(bestSched, lstSched);
    }

    bool EnableEnum_() override {return EnableEnumBBThread_();}
    void FinishOptml_() override {return FinishOptmlBBThread_();}

  // override BBThread virtual
  InstCount getBestCost() override {return *BestCost_;}
  InstCount getCrntScheduleCost() override {return enumBestSched_->GetCost();}
 
  void setBestCost(InstCount BestCost) override { *BestCost_ = BestCost; }

  InstCount UpdtOptmlSched(InstSchedule *crntSched,
                             LengthCostEnumerator *enumrtr) override;


public:
    BBInterfacer(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
              long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
              SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
              bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
              bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc,
              SchedulerType HeurSchedType, SmallVector<MemAlloc<EnumTreeNode> *, 16> &EnumNodeAllocs,
             SmallVector<MemAlloc<CostHistEnumTreeNode> *, 16> &HistNodeAllocs, 
             SmallVector<MemAlloc<BinHashTblEntry<HistEnumTreeNode>> *, 16> &HashTablAllocs);


    inline void SchdulInst(SchedInstruction *inst, InstCount cycleNum, InstCount slotNum,
                  bool trackCnflcts) override
    {
      schdulInst(inst, cycleNum, slotNum, trackCnflcts);
    }

    inline void UnschdulInst(SchedInstruction *inst, InstCount cycleNum,
                    InstCount slotNum, EnumTreeNode *trgtNode) override
    {
      unschdulInst(inst, cycleNum, slotNum, trgtNode);
    }

    inline InstCount CmputNormCost_(InstSchedule *sched, COST_COMP_MODE compMode,
                           InstCount &execCost, bool trackCnflcts) override
    {
      return cmputNormCost(sched, compMode, execCost, trackCnflcts);
    }

    static InstCount ComputeSLILStaticLowerBound(int64_t regTypeCnt_,
                                                 RegisterFile *regFiles_, 
                                                 DataDepGraph *dataDepGraph_);

    //RegFields getRegFields(Register *reg) override {return RegToFields[reg];}

    bool isSecondPass() override { return isSecondPass_; }

    bool isWorker() override {return false;}

    void histTableLock(UDT_HASHVAL key) override {/*nothing*/; }
    void histTableUnlock(UDT_HASHVAL key) override {/*nothing*/; }

    void incrementImprvmntCnt() override {/*nothing*/;}

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




    inline InstCount getHeuristicCost() override {return GetHeuristicCost();}

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
                SchedulerType HeurSchedType, int timeoutToMemblock, bool isTwoPass,
                bool IsTimeoutPerInst, SmallVector<MemAlloc<EnumTreeNode> *, 16> &EnumNodeAllocs,
                SmallVector<MemAlloc<CostHistEnumTreeNode> *, 16> &HistNodeAllocs, 
                SmallVector<MemAlloc<BinHashTblEntry<HistEnumTreeNode>> *, 16> &HashTablAllocs);

    
    FUNC_RESULT Enumerate_(Milliseconds startTime, Milliseconds rgnTimeout,
                           Milliseconds lngthTimeout, int *OptimalSolverID) override;

    Enumerator *AllocEnumrtr_(Milliseconds timeout, SmallVector<MemAlloc<EnumTreeNode> *, 16> &EnumNodeAllocs,
             SmallVector<MemAlloc<CostHistEnumTreeNode> *, 16> &HistNodeAllocs, 
             SmallVector<MemAlloc<BinHashTblEntry<HistEnumTreeNode>> *, 16> &HashTablAllocs) override;

    uint64_t getExaminedNodeCount() override {return Enumrtr_->GetNodeCnt(); }

    inline bool isWorkSteal() override {return false;}
    inline bool isWorkStealOn() override {
      return false;
    }

    inline EnumTreeNode *getStolenNode() override {return nullptr;}

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
//    std::mutex *GlobalPoolLock_; 
    std::mutex *BestSchedLock_;
    std::mutex *NodeCountLock_;
    std::mutex *ImprvmntCntLock_;
    std::mutex *RegionSchedLock_;
    std::mutex *InactiveThreadLock_;

    int *IdleTime_;
    int *InactiveThreads_;
    uint64_t *nodeCounts_;

    bool WorkSteal_;
    bool *WorkStealOn_;
    int64_t **subspaceLwrBounds_;
    EnumTreeNode *stolenNode_ {nullptr};

    bool IsTimeoutPerInst_;
    int timeoutToMemblock_;

    void handlEnumrtrRslt_(FUNC_RESULT rslt, InstCount trgtLngth);

    // overrides
    inline InstCount getBestCost() override {return *MasterCost_;}
    inline void setBestCost(InstCount BestCost) override {
      BestCost_ = BestCost;
      }

    inline InstCount getCrntScheduleCost() override {return MasterSched_->GetCost();}


    InstCount UpdtOptmlSched(InstSchedule *crntSched, LengthCostEnumerator *enumrtr) override;

    void writeBestSchedToMaster(InstSchedule *BestSchedule, InstCount BestCost, InstCount BestSpill);

    inline void reset_() {
      SubspaceLwrBound_ = INVALID_VALUE;
      stolenNode_ = nullptr;
    };

public:
    BBWorker(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
              long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
              SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
              bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
              bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc, bool twoPassEnabled,
              SchedulerType HeurSchedType, bool IsSecondPass, 
              InstSchedule *MasterSched, InstCount *MasterCost, 
              InstCount *MasterSpill, InstCount *MasterLength, 
              InstPool4 *GlobalPool, 
              uint64_t *NodeCount, int SolverID, std::mutex **HistTableLock, 
              std::mutex *GlobalPoolLock, std::mutex *BestSchedLock, std::mutex *NodeCountLock,
              std::mutex *ImprCountLock, std::mutex *RegionSchedLock,
              vector<FUNC_RESULT> *resAddr, int *idleTimes, int NumSolvers, std::vector<InstPool3 *> localPools, 
              std::mutex **localPoolLocks, int *inactiveThreads, std::mutex *inactiveThreadLock, 
              int LocalPoolSize, bool WorkSteal, bool *WorkStealOn, bool IsTimeoutPerInst, uint64_t *nodeCounts,
              int timeoutToMemblock, int64_t **subspaceLwrBounds);

    ~BBWorker();
  
//    std::mutex *GlobalPoolLock_;
  /*
    BBWorker (const BBWorker&) = delete;
    BBWorker& operator= (const BBWorker&) = delete;
    */

    inline SchedInstruction *GetInstByIndex(InstCount index) {return Enumrtr_->GetInstByIndx(index);}

    void setHeurInfo(InstCount SchedUprBound, InstCount HeuristicCost, InstCount SchedLwrBound);

    void allocEnumrtr_(Milliseconds timeout,  MemAlloc<EnumTreeNode> *EnumNodeAlloc,
    MemAlloc<CostHistEnumTreeNode> *HistNodeAlloc, MemAlloc<BinHashTblEntry<HistEnumTreeNode>> *HashTablAlloc);
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

    bool generateStateFromNode(std::shared_ptr<HalfNode> &GlobalPoolNode);

    FUNC_RESULT enumerate_(EnumTreeNode *GlobalPoolNode, Milliseconds StartTime, 
                           Milliseconds RgnTimeout, Milliseconds LngthTimeout, 
                           bool isWorkStealing = false, bool isNodeFsbl = true);

    FUNC_RESULT enumerate_(Milliseconds StartTime, 
                           Milliseconds RgnTimeout, Milliseconds LngthTimeout,
                           bool isWorkStealing = false, bool isNodeFsbl = true);

    FUNC_RESULT generateAndEnumerate(std::shared_ptr<HalfNode> GlobalPoolNode, Milliseconds StartTime, 
                                     Milliseconds RgnTimeout, Milliseconds LngthTimeout);

    inline InstCount CmputNormCost_(InstSchedule *sched, COST_COMP_MODE compMode,
                           InstCount &execCost, bool trackCnflcts)
    {
      return cmputNormCost(sched, compMode, execCost, trackCnflcts);
    }

    bool isSecondPass() override { return IsSecondPass_;}

    bool isWorker() override {return true;}

    inline InstCount getHeuristicCost() override {return HeuristicCost_;}

    inline void setCostLowerBound(InstCount StaticLowerBound) {
      StaticLowerBound_ = StaticLowerBound;
    }

    inline void setMasterSched(InstSchedule *MasterSched) {MasterSched_ = MasterSched;}
    inline void setMasterImprvCount(int *ImprvCount) {MasterImprvCount_ = ImprvCount; }

    inline void setRegionSchedule(InstSchedule *RegionSched) {RegionSched_ = RegionSched;}

    //RegFields getRegFields(Register *reg) override {return RegToFields[reg];}

    void histTableLock(UDT_HASHVAL key) override;
    void histTableUnlock(UDT_HASHVAL key) override; 

    inline bool isWorkSteal() override {return WorkSteal_;}
    inline bool isWorkStealOn() override {
      return *WorkStealOn_;}
    inline void setWorkStealOn(bool value) override {*WorkStealOn_ = value;}
    

    void incrementImprvmntCnt() override;

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

    inline EnumTreeNode *getStolenNode() override {return stolenNode_;}
    inline void setStolenNode(EnumTreeNode *&stolenNode) {stolenNode_ = stolenNode;}

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

    SmallVector<std::ofstream, 16> ThreadStreams_;
    std::mutex **HistTableLock;
    std::mutex GlobalPoolLock;
    std::mutex BestSchedLock;
    std::mutex NodeCountLock;
    std::mutex ImprvCountLock;
    std::mutex RegionSchedLock;
    std::mutex InactiveThreadLock;

    int64_t HistTableSize_;

    int *idleTimes;
    uint64_t *nodeCounts;

    std::vector<InstPool3 *> localPools;
    std::mutex **localPoolLocks;

    int LocalPoolSize_;
    float ExploitationPercent_;
    SPILL_COST_FUNCTION GlobalPoolSCF_;

    bool WorkSteal_;
    bool WorkStealOn_;
    int64_t **subspaceLwrBounds_;
    
    int timeoutToMemblock_;

    void initWorkers(const OptSchedTarget *OST_, DataDepGraph *dataDepGraph,
             long rgnNum, int16_t sigHashSize, LB_ALG lbAlg,
             SchedPriorities hurstcPrirts, SchedPriorities enumPrirts,
             bool vrfySched, Pruning PruningStrategy, bool SchedForRPOnly,
             bool enblStallEnum, int SCW, SPILL_COST_FUNCTION spillCostFunc, bool twoPassEnabled,
             SchedulerType HeurSchedType, InstCount *BestCost, InstCount SchedLwrBound,
             InstSchedule *BestSched, InstCount *BestSpill, 
             InstCount *BestLength, InstPool4 *GlobalPool, 
             uint64_t *NodeCount,  std::mutex **HistTableLock, std::mutex *GlobalPoolLock, std::mutex *BestSchedLock, 
             std::mutex *NodeCountLock, std::mutex *ImprvCountLock, std::mutex *RegionSchedLock, 
             vector<FUNC_RESULT> *results, int *idleTimes,
             int NumSolvers, std::vector<InstPool3 *> localPools, std::mutex **localPoolLocks,
             int *InactiveThreads_, std::mutex *InactiveThreadLock, int LocalPoolSize, bool WorkSteal, 
             bool *WorkStealOn, bool IsTimeoutPerInst, uint64_t *nodeCounts, int timeoutToMemblock, int64_t **subspaceLwrBounds);

  
    bool initGlobalPool();
    bool init();
    void setWorkerHeurInfo();
    Enumerator *allocEnumHierarchy_(Milliseconds timeout, bool *fsbl,  SmallVector<MemAlloc<EnumTreeNode> *, 16> &EnumNodeAllocs,
             SmallVector<MemAlloc<CostHistEnumTreeNode> *, 16> &HistNodeAllocs, 
             SmallVector<MemAlloc<BinHashTblEntry<HistEnumTreeNode>> *, 16> &HashTablAllocs);

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
             int timeoutToMemblock, bool isTwoPass, SmallVector<MemAlloc<EnumTreeNode> *, 16> &EnumNodeAllocs,
             SmallVector<MemAlloc<CostHistEnumTreeNode> *, 16> &HistNodeAllocs, 
             SmallVector<MemAlloc<BinHashTblEntry<HistEnumTreeNode>> *, 16> &HashTablAllocs);

    ~BBMaster();
    
    BBMaster (const BBMaster&) = delete;
    BBMaster& operator= (const BBMaster&) = delete;

    Enumerator *AllocEnumrtr_(Milliseconds timeout, SmallVector<MemAlloc<EnumTreeNode> *, 16> &EnumNodeAllocs,
                              SmallVector<MemAlloc<CostHistEnumTreeNode> *, 16> &HistNodeAllocs, 
                              SmallVector<MemAlloc<BinHashTblEntry<HistEnumTreeNode>> *, 16> &HashTablAllocs) override;


    FUNC_RESULT Enumerate_(Milliseconds startTime, Milliseconds rgnTimeout,
                           Milliseconds lngthTimeout, int *OptimalSolverID) override;

    
    uint64_t getExaminedNodeCount() override {return MasterNodeCount_; }

    int getGlobalPoolSortMethod() override {return GlobalPool->getSortMethod();}

    inline bool isWorkSteal() override {return WorkSteal_;}
    inline bool isWorkStealOn() override {return WorkStealOn_;}

    inline EnumTreeNode *getStolenNode() override {return nullptr;}

    void setWorkStealOn(bool value) override {/*nothing*/};

    

};


} //optsched namespace
} //llvm namespace

#endif
