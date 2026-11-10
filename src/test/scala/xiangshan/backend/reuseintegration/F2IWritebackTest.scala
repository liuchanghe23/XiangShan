package xiangshan.backend.reuseintegration

import chisel3._
import chisel3.util._
import chisel3.reflect.DataMirror
import chisel3.simulator.scalatest.ChiselSim
import org.chipsalliance.cde.config.Parameters
import org.scalatest.flatspec.AnyFlatSpec
import top.MinimalIntRegReuseConfig
import xiangshan._
import xiangshan.backend.BackendParams
import xiangshan.backend.Bundles.F2IWriteBackBundle
import xiangshan.backend.datapath.{DataPath, DataSource, WbDataPath}
import xiangshan.backend.fu.{FuConfig, FuType}
import xiangshan.backend.issue.SchdBlockParams
import xiangshan.backend.regfile.{IntPregParams, RfWritePortBundle}
import xiangshan.backend.rename.IntReuseAllocation
import xiangshan.backend.rob.RobPtr

// Exercise the real EXU, WB arbiter, RF write pipeline, owner check and operand
// read path. An F2I-only result must write even while local toRob.valid is false.
private class F2IWritebackHarness(implicit p: Parameters) extends XSModule {
  private implicit val params: BackendParams = p(XSCoreParamsKey).backendParams
  private implicit val scheduler: SchdBlockParams = params.intSchdParams.get
  params.allExuParams.zipWithIndex.foreach { case (unit, index) =>
    unit.bindBackendParam(params)
    unit.updateIQWakeUpConfigs(params.iqWakeUpParams)
    unit.updateExuIdx(index)
  }
  params.allSchdParams.foreach(_.bindBackendParam(params))
  params.allIssueParams.foreach { block =>
    block.bindBackendParam(params)
    block.exuBlockParams.foreach(_.bindIssueBlockParam(block))
  }
  private val (bridgeIQ, bridgeEXU, bridgeParams) = scheduler.issueBlockParams.zipWithIndex.flatMap {
    case (block, i) => block.exuBlockParams.zipWithIndex.collect {
      case (unit, j) if unit.needDataFromF2I => (i, j, unit)
    }
  }.head
  private val (readIQ, readEXU) = scheduler.issueBlockParams.zipWithIndex.flatMap {
    case (block, i) => block.exuBlockParams.zipWithIndex.collect {
      case (unit, j) if unit.fuConfigs.contains(FuConfig.AluCfg) => (i, j)
    }
  }.head
  val io = IO(new Bundle {
    val f2i = Input(new F2IWriteBackBundle)
    val local = Input(new F2IWriteBackBundle) // local ALU uses data + 0
    val allocate = Input(Valid(new IntReuseAllocation))
    val flush = Input(Valid(new Redirect))
    val read = Input(Valid(UInt(IntPhyRegIdxWidth.W)))
    val readReady = Output(Bool())
    val readValid = Output(Bool())
    val readData = Output(UInt(XLEN.W))
    val wb = Output(new RfWritePortBundle(params.intPregParams))
    val robValid = Output(Bool())
    val robOwner = Output(new RobPtr)
  })
  val exu = Module(bridgeParams.genExuModule)
  val wb = Module(new WbDataPath(params, scheduler))
  val dataPath = Module(new DataPath)
  def defaultInputs(data: Data): Unit = data match {
    case a: Aggregate => a.getElements.foreach(defaultInputs)
    case leaf if DataMirror.directionOf(leaf) == ActualDirection.Input => leaf := 0.U.asTypeOf(leaf)
    case _ =>
  }
  defaultInputs(exu.io)
  defaultInputs(wb.io)
  defaultInputs(dataPath.io)
  exu.io.flush := io.flush
  exu.io.F2IDataIn.get := io.f2i
  exu.io.in.valid := io.local.valid
  exu.io.in.bits.ctrl.fuType := FuType.alu.U
  exu.io.in.bits.ctrl.fuOpType := ALUOpType.add
  exu.io.in.bits.ctrl.rfWen.foreach(_ := true.B)
  exu.io.in.bits.toRF.pdest := io.local.pdest
  exu.io.in.bits.robIdx := io.local.owner
  exu.io.in.bits.data.src(0) := io.local.data
  wb.io.flush := io.flush
  wb.io.fromIntExu(bridgeIQ)(bridgeEXU) <> exu.io.out
  io.wb := wb.io.toIntPreg(bridgeParams.getIntWBPort.get.port)
  io.robValid := exu.io.out.bits.toRob.valid
  io.robOwner := exu.io.out.bits.toRob.bits.robIdx
  dataPath.io.fromIntWb.get := wb.io.toIntPreg
  dataPath.io.flush := io.flush
  dataPath.io.intReuseRedirect := io.flush
  dataPath.io.intReuseAlloc(0) := io.allocate
  (dataPath.io.toIntExu ++ dataPath.io.toFpExu ++ dataPath.io.toVecExu).flatten.foreach(_.ready := true.B)
  val read = dataPath.io.fromIntIQ(readIQ)(readEXU)
  read.valid := io.read.valid
  read.bits.fuType := FuType.alu.U
  read.bits.robIdx.isFormer := true.B
  read.bits.dataSources.foreach(_.value := DataSource.zero)
  read.bits.dataSources(0).value := DataSource.reg
  read.bits.rfBankRen.get(0).zipWithIndex.foreach { case (enable, bank) =>
    enable := io.read.valid && io.read.bits(log2Ceil(coreParams.intPreg.numBank) - 1, 0) === bank.U
  }
  dataPath.io.fromIntIQDeqOg1Payload(readIQ)(readEXU).psrc(0) := io.read.bits
  io.readReady := read.ready
  io.readValid := dataPath.io.toIntExu(readIQ)(readEXU).valid
  io.readData := dataPath.io.toIntExu(readIQ)(readEXU).bits.src(0)
}

class F2IWritebackTest extends AnyFlatSpec with ChiselSim {
  private val base = new MinimalIntRegReuseConfig
  private val core = base(XSTileKey).head
  private implicit val p: Parameters = base.alterPartial {
    case XSCoreParamsKey => core
    case DebugOptionsKey => DebugOptions(FPGAPlatform = true, AlwaysBasicDiff = false,
      EnablePerfDebug = false, AlwaysBasicDB = false)
    case utility.LogUtilsOptionsKey => utility.LogUtilsOptions(false, false, true)
    case utility.PerfCounterOptionsKey => utility.PerfCounterOptions(false, false, utility.XSPerfLevel.VERBOSE, 0)
  }
  private def owner(ptr: RobPtr, value: Int, flag: Boolean = false, former: Boolean = true): Unit = {
    ptr.value.poke(value.U); ptr.flag.poke(flag.B); ptr.isFormer.poke(former.B)
  }
  private def clear(data: Data): Unit = data match {
    case a: Aggregate => a.getElements.foreach(clear)
    case b: Bool => b.poke(false.B)
    case u: UInt => u.poke(0.U)
    case other => throw new IllegalArgumentException(s"unexpected redirect field $other")
  }
  private def initialize(dut: F2IWritebackHarness): Unit = {
    for (port <- Seq(dut.io.f2i, dut.io.local)) {
      port.valid.poke(false.B); port.pdest.poke(0.U); port.data.poke(0.U); owner(port.owner, 0)
    }
    dut.io.allocate.valid.poke(false.B)
    dut.io.allocate.bits.tag.poke(0.U); owner(dut.io.allocate.bits.owner, 0)
    dut.io.flush.valid.poke(false.B)
    clear(dut.io.flush.bits)
    dut.io.read.valid.poke(false.B); dut.io.read.bits.poke(0.U)
    dut.clock.step(3)
  }
  private def allocate(dut: F2IWritebackHarness, tag: Int, rob: Int,
                       flag: Boolean = false, former: Boolean = true): Unit = {
    dut.io.allocate.valid.poke(true.B)
    dut.io.allocate.bits.tag.poke(tag.U); owner(dut.io.allocate.bits.owner, rob, flag, former)
    dut.clock.step(); dut.io.allocate.valid.poke(false.B)
  }
  private def read(dut: F2IWritebackHarness, tag: Int, expected: BigInt): Unit = {
    dut.io.read.valid.poke(true.B); dut.io.read.bits.poke(tag.U)
    dut.io.readReady.expect(true.B)
    dut.clock.step(); dut.io.read.valid.poke(false.B)
    dut.io.readValid.expect(true.B); dut.io.readData.expect(expected.U)
    dut.clock.step()
  }
  private def f2i(dut: F2IWritebackHarness, tag: Int, rob: Int, data: BigInt,
                  flag: Boolean = false, former: Boolean = true): Unit = {
    dut.io.f2i.valid.poke(true.B); dut.io.f2i.pdest.poke(tag.U); dut.io.f2i.data.poke(data.U)
    owner(dut.io.f2i.owner, rob, flag, former)
    dut.io.wb.wen.expect(true.B); dut.io.wb.pdest.expect(tag.U); dut.io.wb.data.expect(data.U)
    dut.io.wb.owner.value.expect(rob.U); dut.io.wb.owner.flag.expect(flag.B)
    dut.io.wb.owner.isFormer.expect(former.B)
    dut.io.robValid.expect(false.B) // floating completion must not fabricate a local completion
    dut.clock.step()
  }
  behavior of "production F2I writeback with integer reuse"

  it should "write consecutive floating results with distinct full owners and retain local ALU ownership" in {
    simulate(new F2IWritebackHarness) { dut =>
      initialize(dut)
      allocate(dut, 1, 10, flag = true)
      allocate(dut, 2, 10, flag = true, former = false)
      f2i(dut, 1, 10, 1, flag = true)
      f2i(dut, 2, 10, BigInt("4337fffffffffff4", 16), flag = true, former = false)
      dut.io.f2i.valid.poke(false.B); dut.clock.step()
      read(dut, 1, 1); read(dut, 2, BigInt("4337fffffffffff4", 16))
      allocate(dut, 3, 11, flag = true)
      dut.io.local.valid.poke(true.B); dut.io.local.pdest.poke(3.U); dut.io.local.data.poke(37.U)
      owner(dut.io.local.owner, 11, flag = true)
      // Invalid F2I metadata must not replace a real integer producer.
      owner(dut.io.f2i.owner, 20, flag = false, former = false)
      dut.io.wb.wen.expect(true.B); dut.io.wb.owner.value.expect(11.U)
      dut.io.wb.owner.flag.expect(true.B); dut.io.wb.owner.isFormer.expect(true.B)
      dut.io.robValid.expect(true.B); dut.io.robOwner.value.expect(11.U)
      dut.clock.step(); dut.io.local.valid.poke(false.B); dut.clock.step()
      read(dut, 3, 37)
    }
  }

  it should "reject stale owners after base reallocation and write a subsequent reused version" in {
    simulate(new F2IWritebackHarness) { dut =>
      initialize(dut)
      allocate(dut, 1, 10, flag = true, former = false)
      // Same index, wrong wrap bit or compressed slot must fail the RF check.
      f2i(dut, 1, 10, 99, flag = false, former = false)
      f2i(dut, 1, 10, 98, flag = true, former = true)
      dut.io.f2i.valid.poke(false.B); dut.clock.step()
      dut.io.read.valid.poke(true.B); dut.io.read.bits.poke(1.U)
      dut.io.readReady.expect(false.B); dut.io.read.valid.poke(false.B)
      f2i(dut, 1, 10, 100, flag = true, former = false)
      dut.io.f2i.valid.poke(false.B); dut.clock.step()
      read(dut, 1, 100)
      allocate(dut, 65, 11, flag = true)
      dut.io.local.valid.poke(true.B); dut.io.local.pdest.poke(65.U); dut.io.local.data.poke(101.U)
      owner(dut.io.local.owner, 11, flag = true)
      dut.clock.step(); dut.io.local.valid.poke(false.B); dut.clock.step()
      read(dut, 65, 101)
      // A fresh lifetime of the same base must reject the previous producer.
      allocate(dut, 1, 12, flag = true)
      f2i(dut, 1, 10, 999, flag = true, former = false)
      dut.io.f2i.valid.poke(false.B); dut.clock.step()
      dut.io.read.valid.poke(true.B); dut.io.read.bits.poke(1.U)
      dut.io.readReady.expect(false.B); dut.io.read.valid.poke(false.B)
      f2i(dut, 1, 12, 7, flag = true)
      dut.io.f2i.valid.poke(false.B); dut.clock.step()
      read(dut, 1, 7)
    }
  }

  it should "keep a surviving floating result while flushing the latter ROB slot in the RF pipeline" in {
    simulate(new F2IWritebackHarness) { dut =>
      initialize(dut)
      allocate(dut, 1, 10)
      allocate(dut, 2, 10, former = false)
      f2i(dut, 1, 10, 1)
      // The former write is in DataPath's register when the redirect arrives.
      dut.io.flush.valid.poke(true.B)
      owner(dut.io.flush.bits.robIdx, 10)
      dut.io.flush.bits.level.poke(false.B) // flushAfter
      f2i(dut, 2, 10, 2, former = false)
      dut.io.f2i.valid.poke(false.B); dut.io.flush.valid.poke(false.B)
      dut.clock.step(3)
      read(dut, 1, 1)
      dut.io.read.valid.poke(true.B); dut.io.read.bits.poke(2.U)
      dut.io.readReady.expect(false.B)
    }
  }

  it should "preserve the integer version bit above the floating destination width" in {
    // CDE alterPartial evaluates its RHS on every lookup. All modules must
    // share the same mutable backend bindings (EXU indices and wakeup pairs).
    val fullCore = core.copy(intPreg = IntPregParams(224, 4, None, None))
    val full = p.alterPartial {
      case XSCoreParamsKey => fullCore
    }
    simulate(new F2IWritebackHarness()(full)) { dut =>
      initialize(dut)
      allocate(dut, 1, 10)
      f2i(dut, 1, 10, 100)
      dut.io.f2i.valid.poke(false.B); dut.clock.step()
      allocate(dut, 257, 11)
      // F2I is not normally reused by rename; inject a v1 tag here to ensure
      // the shared bridge never truncates an integer destination to FP width.
      f2i(dut, 257, 11, 101)
      dut.io.f2i.valid.poke(false.B); dut.clock.step()
      read(dut, 257, 101)
    }
  }
}
