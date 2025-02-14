//===- OptSchedDDGWrapperGCN.cpp - GCN DDG Wrapper ------------------------===//
//
// Conversion from LLVM ScheduleDAG to OptSched DDG for amdgcn target.
//
//===----------------------------------------------------------------------===//

#include "OptSchedDDGWrapperGCN.h"
#include "GCNRegPressure.h"
#include "opt-sched/Scheduler/register.h"
#include "SIRegisterInfo.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/MachineInstrBundle.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include <cstdio>

#define DEBUG_TYPE "optsched-ddg-wrapper"

using namespace llvm;
using namespace llvm::opt_sched;

OptSchedDDGWrapperGCN::OptSchedDDGWrapperGCN(MachineSchedContext *Context,
                                             ScheduleDAGOptSched *DAG,
                                             OptSchedMachineModel *MM,
                                             LATENCY_PRECISION LatencyPrecision,
                                             const std::string &RegionID, int NumSolvers)
    : OptSchedDDGWrapperBasic(Context, DAG, MM, LatencyPrecision, RegionID, NumSolvers),
      SUnits(DAG->SUnits), LIS(DAG->getLIS()), MRI(DAG->MRI) {}

namespace {

std::unique_ptr<SubRegSet>
createSubRegSet(unsigned Reg, const MachineRegisterInfo &MRI, int16_t Type) {
  unsigned numSubRegs =
      SIRegisterInfo::getNumCoveredRegs(MRI.getMaxLaneMaskForVReg(Reg));
  return std::make_unique<SubRegSet>(numSubRegs, Type);
}

// Copied from Target/AMDGPU/GCNRegPressure.cpp
LaneBitmask getDefRegMask(const MachineOperand &MO,
                          const MachineRegisterInfo &MRI) {
  assert(MO.isDef() && MO.isReg() && MO.getReg().isVirtual());

  // We don't rely on read-undef flag because in case of tentative schedule
  // tracking it isn't set correctly yet. This works correctly however since
  // use mask has been tracked before using LIS.
  return MO.getSubReg() == 0
             ? MRI.getMaxLaneMaskForVReg(MO.getReg())
             : MRI.getTargetRegisterInfo()->getSubRegIndexLaneMask(
                   MO.getSubReg());
}

// Copied from Target/AMDGPU/GCNRegPressure.cpp
LaneBitmask getUsedRegMask(const MachineOperand &MO,
                           const MachineRegisterInfo &MRI,
                           const LiveIntervals &LIS) {
  assert(MO.isUse() && MO.isReg() && MO.getReg().isVirtual());

  if (auto SubReg = MO.getSubReg())
    return MRI.getTargetRegisterInfo()->getSubRegIndexLaneMask(SubReg);

  auto MaxMask = MRI.getMaxLaneMaskForVReg(MO.getReg());
  if (MaxMask == LaneBitmask::getLane(0)) // cannot have subregs
    return MaxMask;

  // For a tentative schedule LIS isn't updated yet but livemask should remain
  // the same on any schedule. Subreg defs can be reordered but they all must
  // dominate uses anyway.
  auto SI = LIS.getInstructionIndex(*MO.getParent()).getBaseIndex();
  return getLiveLaneMask(MO.getReg(), SI, LIS, MRI);
}

// Copied from Target/AMDGPU/GCNRegPressure.cpp
SmallVector<RegisterMaskPair, 8>
collectVirtualRegUses(const MachineInstr &MI, const LiveIntervals &LIS,
                      const MachineRegisterInfo &MRI) {
  SmallVector<RegisterMaskPair, 8> Res;
  for (ConstMIBundleOperands MIO(MI); MIO.isValid(); ++MIO) {
    const MachineOperand MO = *MIO;
#ifdef DEBUG_REG
    Logger::Info("processing Op");
    MO.print(errs());
    errs() << "\n";

    if (!MO.isReg()) {
      Logger::Info("Is Not Reg");
      continue;
    }
    if (!MO.getReg().isVirtual())
      Logger::Info("Is Not VirtReg");
    if (!MO.isUse())
      Logger::Info("Is Not Use");
    if (!MO.readsReg())
      Logger::Info("Is Not Reads Reg");
#endif

    if (!MO.isReg() || !MO.getReg().isVirtual())
      continue;
    if (!MO.isUse() || !MO.readsReg())
      continue;

    const auto UsedMask = getUsedRegMask(MO, MRI, LIS);

    auto Reg = MO.getReg();

#ifdef DEBUG_REG
    Logger::Info("found use");
    Logger::Info("has Reg %u", Reg.id());
    auto maskPrint = PrintLaneMask(UsedMask);
    errs() << maskPrint;
    errs() << "\n";
#endif
    auto I =
        std::find_if(Res.begin(), Res.end(), [Reg](const RegisterMaskPair &RM) {
          return RM.RegUnit == Reg;
        });
    if (I != Res.end())
      I->LaneMask |= UsedMask;
    else
      Res.push_back(RegisterMaskPair(Reg, UsedMask));
  }
  return Res;
}



SmallVector<RegisterMaskPair, 8>
collectVirtualRegDefs(const MachineInstr &MI, const LiveIntervals &LIS,
                      const MachineRegisterInfo &MRI,
                      const ScheduleDAGOptSched *DAG) {
  SmallVector<RegisterMaskPair, 8> Res;

  for (ConstMIBundleOperands MIO(MI); MIO.isValid(); ++MIO) {
    const MachineOperand MO = *MIO;
#ifdef DEBUG_REG
    Logger::Info("Processing Op");
    MO.print(errs());
    errs() << "\n";

    if (!MO.isReg()) {
      Logger::Info("Is Not Reg");
      continue;
    }
    if (!MO.getReg().isVirtual())
      Logger::Info("Is Not VirtReg");
    if (!MO.isDef())
      Logger::Info("Is Not Def");
    if (MO.isDead())
      Logger::Info("Is Dead");
#endif

    if (!MO.isReg() || !MO.getReg().isVirtual() || MO.isDead() || !MO.isDef()) {
      continue;
    }

    const auto DefMask = getDefRegMask(MO, MRI);

    auto Reg = MO.getReg();

#ifdef DEBUG_REG
    Logger::Info("found def");
    Logger::Info("has Reg %u", Reg.id());
    auto maskPrint = PrintLaneMask(DefMask);
    errs() << maskPrint;
    errs() << "\n";
#endif

    auto I =
        std::find_if(Res.begin(), Res.end(), [Reg](const RegisterMaskPair &RM) {
          return RM.RegUnit == Reg;
        });
    if (I != Res.end())
      I->LaneMask |= DefMask;
    else
      Res.push_back(RegisterMaskPair(Reg, DefMask));
  }
  return Res;
}

SmallVector<RegisterMaskPair, 8>
collectLiveSubRegsAtInstr(const MachineInstr *MI, const LiveIntervals *LIS,
                          const MachineRegisterInfo &MRI, bool After) {
  SlotIndex SI = After ? LIS->getInstructionIndex(*MI).getDeadSlot()
                       : LIS->getInstructionIndex(*MI).getBaseIndex();

  SmallVector<RegisterMaskPair, 8> Res;
  for (unsigned I = 0, E = MRI.getNumVirtRegs(); I != E; ++I) {
    auto Reg = llvm::Register::index2VirtReg(I);
    if (!LIS->hasInterval(Reg))
      continue;

    auto LiveMask = getLiveLaneMask(Reg, SI, *LIS, MRI);
    if (LiveMask.any()) {
      Res.emplace_back(Reg, LiveMask);
    }
  }
  return Res;
}

} // end anonymous namespace

unsigned OptSchedDDGWrapperGCN::getRegKind(unsigned Reg) const {
  assert(llvm::Register::isVirtualRegister(Reg));
  const auto RC = MRI.getRegClass(Reg);
  auto STI = static_cast<const SIRegisterInfo *>(MRI.getTargetRegisterInfo());
  if (STI->isAGPRClass(RC))
    Logger::Info("FOUND AGPR!");
  return STI->isSGPRClass(RC) ? SGPR32 : VGPR32;
}

void OptSchedDDGWrapperGCN::convertRegFiles() {
  for (int i = 0; i < MM->GetRegTypeCnt(); i++)
    RegFiles[i].SetRegType(i);

  // Add live-in subregs
  for (const auto &MaskPair :
       collectLiveSubRegsAtInstr(SUnits[0].getInstr(), LIS, MRI, false))
    addSubRegDefs(GetRootInst(), MaskPair.RegUnit, MaskPair.LaneMask, true);

  for (const auto &SU : SUnits) {
    const MachineInstr *MI = SU.getInstr();
#ifdef DEBUG_REG
    Logger::Info("Parsing Inst");
    MI->print(errs());
#endif

    for (const auto &MaskPair : collectVirtualRegUses(*MI, *LIS, MRI))
      addSubRegUses(GetInstByIndx(SU.NodeNum), MaskPair.RegUnit,
                    MaskPair.LaneMask);

    for (const auto &MaskPair : collectVirtualRegDefs(*MI, *LIS, MRI, DAG))
      addSubRegDefs(GetInstByIndx(SU.NodeNum), MaskPair.RegUnit,
                    MaskPair.LaneMask);
  }

  // Add live-out subregs
  for (const auto &MaskPair : collectLiveSubRegsAtInstr(
           SUnits[SUnits.size() - 1].getInstr(), LIS, MRI, true))
    addSubRegUses(GetLeafInst(), MaskPair.RegUnit, MaskPair.LaneMask,
                  /*LiveOut=*/true);

  // TODO: Count defined-and-not-used registers as live-out uses to avoid assert
  // errors in OptSched.
  for (int16_t i = 0; i < MM->GetRegTypeCnt(); i++)
    for (int j = 0; j < RegFiles[i].GetRegCnt(); j++) {
      Register *Reg = RegFiles[i].GetReg(j);
      if (Reg->GetUseCnt() == 0)
        addDefAndNotUsed(Reg);
    }

  LLVM_DEBUG(DAG->dumpLLVMRegisters());
  LLVM_DEBUG(dumpOptSchedRegisters());
}


void OptSchedDDGWrapperGCN::addSubRegDefs(SchedInstruction *Instr, unsigned Reg,
                                          const LaneBitmask &LiveMask,
                                          bool LiveIn) {
  if (RegionRegs[Reg] == nullptr) {
    RegionRegs[Reg] = createSubRegSet(Reg, MRI, getRegKind(Reg));
  }

  SubRegSet &SubRegs = *RegionRegs[Reg].get();
  RegisterFile &RF = RegFiles[SubRegs.Type];
  unsigned Lane = 0;
#ifdef DEBUG_REG
  Logger::Info("Processing LLVM Reg %u", Reg);
  auto Temp = Reg;
#endif
  for (auto &ResNo : SubRegs) {
    if ((LiveMask.getLane(Lane) & LiveMask).any() ||
        (LiveMask.getLane(Lane + 1) & LiveMask).any()) {

      Register *Reg = RF.getNext();
      ResNo = Reg->GetNum(0);
#ifdef DEBUG_REG
      Logger::Info("maps to OptSched Reg %d", Reg->GetNum());
      Logger::Info(
          "Adding def for subreg of reg %u (optsched vreg %d, type = %d)", Temp,
          ResNo, Reg->GetType());
#endif
      Instr->AddDef(Reg);
      // Weight should always be one since we are only tracking VGPR32 and
      // SGPR32
      Reg->SetWght(1);
      Reg->AddDef(Instr);
      Reg->SetIsLiveIn(LiveIn);
    }
    if ((LiveMask.getLane(Lane) & LiveMask).any() !=
        (LiveMask.getLane(Lane + 1) & LiveMask).any()) {
      Logger::Info("found lane mismatch");
    }
    Lane += 2;
  }
}

void OptSchedDDGWrapperGCN::addSubRegUses(SchedInstruction *Instr, unsigned Reg,
                                          const LaneBitmask &LiveMask,
                                          bool LiveOut) {
  auto temp = RegionRegs[Reg].get();
  if (temp == nullptr)
    DAG->MF.print(errs());
  SubRegSet &SubRegs = *temp;
  RegisterFile &RF = RegFiles[SubRegs.Type];
  unsigned Lane = 0;
#ifdef DEBUG_REG
  Logger::Info("Processing LLVM Reg %u", Reg);
  auto Temp = Reg;
#endif
  for (auto &ResNo : SubRegs) {
    if ((LiveMask.getLane(Lane) & LiveMask).any() ||
        (LiveMask.getLane(Lane + 1) & LiveMask).any()) {
      Register *Reg = RF.GetReg(ResNo);
#ifdef DEBUG_REG
      Logger::Info("maps to OptSched Reg %d", Reg->GetNum());
      Logger::Info(
          "Adding use for subreg of reg %u (optsched vreg %d, type = %d)", Temp,
          ResNo, Reg->GetType());
#endif
      Instr->AddUse(Reg);
      Reg->AddUse(Instr);
      Reg->SetIsLiveOut(LiveOut);
    }
    if ((LiveMask.getLane(Lane) & LiveMask).any() !=
        (LiveMask.getLane(Lane + 1) & LiveMask).any()) {
      Logger::Info("found lane mismatch");
    }

    Lane += 2;
  }
}

int OptSchedDDGWrapperGCN::getSize() {
  return DAG->SUnits.size() + 2;
}

