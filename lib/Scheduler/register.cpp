#include "opt-sched/Scheduler/register.h"
#include "llvm/ADT/STLExtras.h"

using namespace llvm::opt_sched;

int16_t llvm::opt_sched::Register::GetType() const { return type_; }

int llvm::opt_sched::Register::GetNum(int SolverID) const { return otherCachedVals[SolverID].num_; }

int llvm::opt_sched::Register::GetWght() const { return wght_; }

void llvm::opt_sched::Register::SetType(int16_t type) { type_ = type; }

void llvm::opt_sched::Register::SetNum(int SolverID, int num) { otherCachedVals[SolverID].num_ = num; }

void llvm::opt_sched::Register::setNumSolvers(int NumSolvers) {NumSolvers_ = NumSolvers; }

void llvm::opt_sched::Register::SetWght(int wght) { wght_ = wght; }

bool llvm::opt_sched::Register::IsPhysical() const { return physicalNumber_ != INVALID_VALUE; }

int llvm::opt_sched::Register::GetPhysicalNumber() const { return physicalNumber_; }

void llvm::opt_sched::Register::SetPhysicalNumber(int physicalNumber) {
  physicalNumber_ = physicalNumber;
}

bool llvm::opt_sched::Register::IsLive(int SolverID) const {
  assert(cachedVals[SolverID].crntUseCnt_ <= useCnt_);
  return cachedVals[SolverID].crntUseCnt_ < useCnt_;
}

bool llvm::opt_sched::Register::IsLiveIn() const { return liveIn_; }

bool llvm::opt_sched::Register::IsLiveOut() const { return liveOut_; }

void llvm::opt_sched::Register::SetIsLiveIn(bool liveIn) { liveIn_ = liveIn; }

void llvm::opt_sched::Register::SetIsLiveOut(bool liveOut) { liveOut_ = liveOut; }

void llvm::opt_sched::Register::ResetCrntUseCnt(int SolverID) {cachedVals[SolverID].crntUseCnt_ = 0; }

void llvm::opt_sched::Register::AddUse(const SchedInstruction *inst) {
  uses_.insert(inst);
  useCnt_++;
}

void llvm::opt_sched::Register::AddDef(const SchedInstruction *inst) {
  defs_.insert(inst);
  defCnt_++;
}

int llvm::opt_sched::Register::GetUseCnt() const { return useCnt_; }

const llvm::opt_sched::Register::InstSetType &llvm::opt_sched::Register::GetUseList() const { return uses_; }

size_t llvm::opt_sched::Register::GetSizeOfUseList() const { return uses_.size(); }

int llvm::opt_sched::Register::GetDefCnt() const { return defCnt_; }

const llvm::opt_sched::Register::InstSetType &llvm::opt_sched::Register::GetDefList() const { return defs_; }

size_t llvm::opt_sched::Register::GetSizeOfDefList() const { return defs_.size(); }

int llvm::opt_sched::Register::GetCrntUseCnt(int SolverID) const { return cachedVals[SolverID].crntUseCnt_; }

void llvm::opt_sched::Register::AddCrntUse(int SolverID) { cachedVals[SolverID].crntUseCnt_++; }

void llvm::opt_sched::Register::DelCrntUse(int SolverID) { cachedVals[SolverID].crntUseCnt_--; }

void llvm::opt_sched::Register::ResetCrntLngth() { crntLngth_ = 0; }

int llvm::opt_sched::Register::GetCrntLngth() const { return crntLngth_; }

void llvm::opt_sched::Register::IncrmntCrntLngth() { crntLngth_++; }

void llvm::opt_sched::Register::DcrmntCrntLngth() { crntLngth_--; }

llvm::opt_sched::Register &llvm::opt_sched::Register::operator=(const llvm::opt_sched::Register &rhs) {
  if (this != &rhs) {
    for (int i = 0; i < NumSolvers_; i++)
      otherCachedVals[i].num_ = rhs.otherCachedVals[i].num_;
    type_ = rhs.type_;
  }

  return *this;
}

void llvm::opt_sched::Register::SetupConflicts(int regCnt) { conflicts_.Construct(regCnt); }

void llvm::opt_sched::Register::ResetConflicts() {
  conflicts_.Reset();
  isSpillCnddt_ = false;
}

void llvm::opt_sched::Register::AddConflict(int regNum, bool isSpillCnddt) {
  //assert(regNum != otherCachedVals[0].num_);
  assert(regNum >= 0);
  conflicts_.SetBit(regNum, true);
  isSpillCnddt_ = isSpillCnddt_ || isSpillCnddt;
}

int llvm::opt_sched::Register::GetConflictCnt() const { return conflicts_.GetOneCnt(); }

bool llvm::opt_sched::Register::IsSpillCandidate() const { return isSpillCnddt_; }

bool llvm::opt_sched::Register::AddToInterval(const SchedInstruction *inst) {
  return liveIntervalSet_.insert(inst).second;
}

bool llvm::opt_sched::Register::IsInInterval(const SchedInstruction *inst) const {
  return liveIntervalSet_.count(inst) != 0;
}

const llvm::opt_sched::Register::InstSetType &llvm::opt_sched::Register::GetLiveInterval() const {
  return liveIntervalSet_;
}

bool llvm::opt_sched::Register::AddToPossibleInterval(const SchedInstruction *inst) {
  return possibleLiveIntervalSet_.insert(inst).second;
}

bool llvm::opt_sched::Register::IsInPossibleInterval(const SchedInstruction *inst) const {
  return possibleLiveIntervalSet_.count(inst) != 0;
}

const llvm::opt_sched::Register::InstSetType &llvm::opt_sched::Register::GetPossibleLiveInterval() const {
  return possibleLiveIntervalSet_;
}

llvm::opt_sched::Register::Register(int NumSolvers, int16_t type, int num, int physicalNumber) {
  type_ = type;
  //num_ = num;
  wght_ = 1;
  defCnt_ = 0;
  useCnt_ = 0;
  cachedVals = new paddedVals[NumSolvers];
  otherCachedVals = new paddedVals[NumSolvers];
  
  for (int SolverID = 0; SolverID < NumSolvers; SolverID++) {
    cachedVals[SolverID].crntUseCnt_ = 0;
  }
  
  for (int SolverID = 0; SolverID < NumSolvers; SolverID++) {
    otherCachedVals[SolverID].num_ = num;
  }

  physicalNumber_ = physicalNumber;
  isSpillCnddt_ = false;
  liveIn_ = false;
  liveOut_ = false;
  NumSolvers_ = NumSolvers;
}

llvm::opt_sched::Register::~Register() {
  delete[] cachedVals;
  delete[] otherCachedVals;
}

RegisterFile::RegisterFile() {
  regType_ = 0;
  physRegCnt_ = 0;
}

RegisterFile::~RegisterFile() {}

int RegisterFile::GetRegCnt() const { return getCount(); }

int16_t RegisterFile::GetRegType() const { return regType_; }

void RegisterFile::SetRegType(int16_t regType) { regType_ = regType; }

void RegisterFile::setNumSolvers(int NumSolvers) {NumSolvers_ = NumSolvers; }

void RegisterFile::ResetCrntUseCnts(int SolverID) {
  for (int i = 0; i < getCount(); i++) {
    Regs[i]->ResetCrntUseCnt(SolverID);
  }
}

void RegisterFile::ResetCrntLngths() {
  for (int i = 0; i < getCount(); i++) {
    Regs[i]->ResetCrntLngth();
  }
}

llvm::opt_sched::Register *RegisterFile::getNext() {
  size_t RegNum = Regs.size();
  // TODO
  //auto Reg = llvm::make_unique<Register>(NumSolvers_);
  auto Reg = std::unique_ptr<llvm::opt_sched::Register>(new llvm::opt_sched::Register(NumSolvers_));
  Reg->setNumSolvers(NumSolvers_);
  Reg->SetType(regType_);
  for (int i = 0; i < NumSolvers_; i++)
    Reg->SetNum(i, RegNum);
  Regs.push_back(std::move(Reg));
  return Regs[RegNum].get();
}

void RegisterFile::SetRegCnt(int regCnt) {
  if (regCnt == 0)
    return;

  Regs.resize(regCnt);
  for (int i = 0; i < getCount(); i++) {
    //auto Reg = llvm::make_unique<Register>();
    auto Reg = std::unique_ptr<llvm::opt_sched::Register>(new llvm::opt_sched::Register(NumSolvers_));
    Reg->SetType(regType_);
    for (int j = 0; j < NumSolvers_; j++)
      Reg->SetNum(j, i);
    Regs[i] = std::move(Reg);
  }
}

llvm::opt_sched::Register *RegisterFile::GetReg(int num) const {
  if (num >= 0 && num < getCount()) {
    return Regs[num].get();
  } else {
    return NULL;
  }
}

llvm::opt_sched::Register *RegisterFile::FindLiveReg(int physNum, int SolverID) const {
  for (int i = 0; i < getCount(); i++) {
    if (Regs[i]->GetPhysicalNumber() == physNum && Regs[i]->IsLive(SolverID) == true)
      return Regs[i].get();
  }
  return NULL;
}

int RegisterFile::FindPhysRegCnt() {
  int maxPhysNum = -1;
  for (int i = 0; i < getCount(); i++) {
    if (Regs[i]->GetPhysicalNumber() != INVALID_VALUE &&
        Regs[i]->GetPhysicalNumber() > maxPhysNum)
      maxPhysNum = Regs[i]->GetPhysicalNumber();
  }

  // Assume that physical registers are given sequential numbers
  // starting from 0.
  physRegCnt_ = maxPhysNum + 1;
  return physRegCnt_;
}

int RegisterFile::GetPhysRegCnt() const { return physRegCnt_; }

void RegisterFile::SetupConflicts() {
  for (int i = 0; i < getCount(); i++)
    Regs[i]->SetupConflicts(getCount());
}

void RegisterFile::ResetConflicts() {
  for (int i = 0; i < getCount(); i++)
    Regs[i]->ResetConflicts();
}

int RegisterFile::GetConflictCnt() {
  int cnflctCnt = 0;
  for (int i = 0; i < getCount(); i++) {
    cnflctCnt += Regs[i]->GetConflictCnt();
  }
  return cnflctCnt;
}

void RegisterFile::AddConflictsWithLiveRegs(int regNum, int liveRegCnt, int SolverID) {
  Logger::Info("in addconflict with live regs");
  bool isSpillCnddt = (liveRegCnt + 1) > physRegCnt_;
  int conflictCnt = 0;
  for (int i = 0; i < getCount(); i++) {
    if (i != regNum && Regs[i]->IsLive(SolverID) == true) {
      Regs[i]->AddConflict(regNum, isSpillCnddt);
      Regs[regNum]->AddConflict(i, isSpillCnddt);
      conflictCnt++;
    }
    if (conflictCnt == liveRegCnt)
      break;
  }
}
