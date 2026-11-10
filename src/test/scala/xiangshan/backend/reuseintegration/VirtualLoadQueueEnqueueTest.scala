package xiangshan.backend.reuseintegration

import chisel3._
import chisel3.util.RegEnable
import chisel3.reflect.DataMirror
import chisel3.simulator.scalatest.ChiselSim
import org.chipsalliance.cde.config.Parameters
import org.scalatest.flatspec.AnyFlatSpec
import top.MinimalIntRegReuseConfig
import xiangshan._
import xiangshan.mem.VirtualLoadQueue

private class VirtualLoadQueueEnqueueHarness(implicit p: Parameters) extends XSModule {
  val io = IO(new Bundle {
    val enqueue = Input(Bool())
    val sqCanAccept = Input(Bool())
    val complete = Input(Bool())
    val canAccept = Output(Bool())
    val empty = Output(Bool())
  })
  private val dut = Module(new VirtualLoadQueue)
  private def defaultInputs(data: Data): Unit = data match {
    case aggregate: Aggregate => aggregate.getElements.foreach(defaultInputs)
    case leaf if DataMirror.directionOf(leaf) == ActualDirection.Input =>
      leaf := 0.U.asTypeOf(leaf)
    case _ =>
  }
  defaultInputs(dut.io)
  dut.io.enq.sqCanAccept := io.sqCanAccept
  // Match the failing gcc_s04 enqueue lane. A load reserves only an LQ entry.
  private val req = dut.io.enq.req(1)
  req.valid := io.enqueue
  req.bits.numLsElem := 1.U
  req.bits.robIdx.value := 1.U
  req.bits.robIdx.isFormer := true.B
  req.bits.lqIdx := dut.io.enq.resp(1)
  dut.io.enq.needAlloc(1) := io.enqueue
  private val lastIndex = RegEnable(dut.io.enq.resp(1), io.enqueue)
  private val wb = dut.io.ldin.head
  wb.valid := io.complete
  wb.bits.uop.lqIdx := lastIndex
  wb.bits.uop.robIdx.value := 1.U
  wb.bits.uop.robIdx.isFormer := true.B
  wb.bits.updateAddrValid := true.B
  io.canAccept := dut.io.enq.canAccept
  io.empty := dut.io.lqEmpty
}

class VirtualLoadQueueEnqueueTest extends AnyFlatSpec with ChiselSim {
  private val base = new MinimalIntRegReuseConfig
  private val core = base(XSTileKey).head
  private implicit val p: Parameters = base.alterPartial {
    case XSCoreParamsKey => core
    case DebugOptionsKey => DebugOptions(AlwaysBasicDiff = false,
      EnablePerfDebug = false, AlwaysBasicDB = false)
    // Keep XSError assertions enabled: FPGA mode would hide the regression.
    case utility.LogUtilsOptionsKey => utility.LogUtilsOptions(false, false, false)
    case utility.PerfCounterOptionsKey =>
      utility.PerfCounterOptions(false, false, utility.XSPerfLevel.VERBOSE, 0)
  }
  behavior of "production VirtualLoadQueue independent admission"

  it should "accept loads while SQ is unavailable and retain its own capacity limit" in {
    simulate(new VirtualLoadQueueEnqueueHarness) { dut =>
      dut.io.enqueue.poke(false.B)
      dut.io.complete.poke(false.B)
      dut.io.sqCanAccept.poke(false.B)
      dut.clock.step(3)
      for (sqReady <- Seq(false, true)) {
        dut.io.sqCanAccept.poke(sqReady.B)
        dut.io.canAccept.expect(true.B)
        dut.io.enqueue.poke(true.B)
        dut.clock.step()
        dut.io.enqueue.poke(false.B)
        dut.clock.step()
        dut.io.empty.expect(false.B)
        dut.io.complete.poke(true.B)
        dut.clock.step()
        dut.io.complete.poke(false.B)
        dut.clock.step(6)
        dut.io.empty.expect(true.B)
      }
      dut.io.sqCanAccept.poke(false.B)
      for (_ <- 0 until core.VirtualLoadQueueSize) {
        dut.io.canAccept.expect(true.B)
        dut.io.enqueue.poke(true.B)
        dut.clock.step()
      }
      dut.io.enqueue.poke(false.B)
      dut.io.canAccept.expect(false.B)
      dut.io.empty.expect(false.B)
      // Releasing SQ pressure cannot make a full LQ accept another load.
      dut.io.sqCanAccept.poke(true.B)
      dut.io.canAccept.expect(false.B)
      dut.clock.step(2)
    }
  }
}
