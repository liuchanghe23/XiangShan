package xiangshan.backend.reuseintegration

import chisel3._
import chisel3.reflect.DataMirror
import chisel3.simulator.scalatest.ChiselSim
import org.chipsalliance.cde.config.Parameters
import org.scalatest.flatspec.AnyFlatSpec
import top.MinimalIntRegReuseConfig
import xiangshan._
import xiangshan.backend.BackendParams
import xiangshan.backend.datapath.{DataPath, DataSource}
import xiangshan.backend.fu.{FuConfig, FuType}
import xiangshan.backend.issue.SchdBlockParams

private class DataPathRecoveryHarness(implicit p: Parameters) extends XSModule {
  private implicit val params: BackendParams = p(XSCoreParamsKey).backendParams
  private implicit val scheduler: SchdBlockParams = params.intSchdParams.get
  // DataPath normally receives parameters bound by BackendInlined.
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
  val io = IO(new Bundle {
    val offer = Input(Bool())
    val redirect = Input(Bool())
    val younger = Input(Bool())
    val ready = Output(Bool())
    val recoveryBusy = Output(Bool())
    val execute = Output(Bool())
    val failed = Output(Bool())
    val finalSuccess = Output(Bool())
  })
  val dut = Module(new DataPath)
  def defaultInputs(data: Data): Unit = data match {
    case a: Aggregate => a.getElements.foreach(defaultInputs)
    case leaf if DataMirror.directionOf(leaf) == ActualDirection.Input => leaf := 0.U.asTypeOf(leaf)
    case _ =>
  }
  defaultInputs(dut.io)
  (dut.io.toIntExu ++ dut.io.toFpExu ++ dut.io.toVecExu).flatten.foreach(_.ready := true.B)
  val (iq, exu) = scheduler.issueBlockParams.zipWithIndex.flatMap { case (block, i) =>
    block.exuBlockParams.zipWithIndex.collect {
      case (unit, j) if unit.fuConfigs.contains(FuConfig.AluCfg) => (i, j)
    }
  }.last
  val in = dut.io.fromIntIQ(iq)(exu)
  in.valid := io.offer
  in.bits.fuType := FuType.alu.U
  in.bits.rfWen.foreach(_ := true.B)
  in.bits.pdest := 1.U
  in.bits.robIdx.value := Mux(io.younger, 12.U, 8.U)
  in.bits.robIdx.isFormer := true.B
  in.bits.dataSources.foreach(_.value := DataSource.zero)
  dut.io.intReuseRedirect.valid := io.redirect
  dut.io.intReuseRedirect.bits.robIdx.value := 10.U
  dut.io.intReuseRedirect.bits.robIdx.isFormer := true.B
  // Region registers the execution redirect one cycle after the local RF sees it.
  dut.io.flush := RegNext(dut.io.intReuseRedirect, 0.U.asTypeOf(dut.io.intReuseRedirect))
  io.ready := in.ready
  io.recoveryBusy := dut.io.intReuseRecoveryBusy
  io.execute := dut.io.toIntExu(iq)(exu).fire
  io.failed := dut.io.toIntIQ(iq)(exu).og1resp.failed
  io.finalSuccess := dut.io.toIntIQ(iq)(exu).og1resp.finalSuccess
}

class DataPathRecoveryTest extends AnyFlatSpec with ChiselSim {
  private val base = new MinimalIntRegReuseConfig
  private val core = base(XSTileKey).head
  private implicit val p: Parameters = base.alterPartial {
    case XSCoreParamsKey => core
    case DebugOptionsKey => DebugOptions(FPGAPlatform = true, AlwaysBasicDiff = false,
      EnablePerfDebug = false, AlwaysBasicDB = false)
    case utility.LogUtilsOptionsKey => utility.LogUtilsOptions(false, false, true)
    case utility.PerfCounterOptionsKey => utility.PerfCounterOptions(false, false, utility.XSPerfLevel.VERBOSE, 0)
  }
  behavior of "production DataPath at the local recovery boundary"

  it should "drain an accepted ALU at OG1 while blocking new OG0 issue" in {
    simulate(new DataPathRecoveryHarness) { dut =>
      dut.io.offer.poke(false.B); dut.io.redirect.poke(false.B); dut.io.younger.poke(false.B)
      dut.clock.step(3)
      dut.io.offer.poke(true.B); dut.io.ready.expect(true.B)
      dut.clock.step()
      dut.io.redirect.poke(true.B)
      dut.io.recoveryBusy.expect(true.B)
      dut.io.ready.expect(false.B)
      // Its speculative zero-latency wakeup has escaped by OG1. Rejecting it
      // here leaves consumers ready and pointing to an unwritten RC entry.
      dut.io.execute.expect(true.B)
      dut.io.failed.expect(false.B)
      dut.io.finalSuccess.expect(true.B)
      dut.clock.step()
      dut.io.offer.poke(false.B); dut.io.redirect.poke(false.B)
      dut.io.execute.expect(false.B)
      dut.clock.step(2)
    }
  }

  it should "still flush a younger ALU through the execution redirect" in {
    simulate(new DataPathRecoveryHarness) { dut =>
      dut.io.offer.poke(false.B); dut.io.redirect.poke(false.B); dut.io.younger.poke(true.B)
      dut.clock.step(3)
      dut.io.redirect.poke(true.B)
      dut.io.ready.expect(false.B)
      dut.clock.step()
      dut.io.redirect.poke(false.B); dut.io.offer.poke(true.B)
      dut.clock.step()
      dut.io.offer.poke(false.B)
      dut.io.execute.expect(false.B)
      dut.clock.step(2)
    }
  }
}
