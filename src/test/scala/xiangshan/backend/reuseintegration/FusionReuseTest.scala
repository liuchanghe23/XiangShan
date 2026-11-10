package xiangshan.backend.reuseintegration

import chisel3._
import chisel3.util._
import chisel3.simulator.scalatest.ChiselSim
import org.chipsalliance.cde.config.Parameters
import org.scalatest.flatspec.AnyFlatSpec
import top.IntRegReuseConfig
import xiangshan._
import xiangshan.backend.decode.FusionDecoder
import xiangshan.backend.fu.FuType

private class FusionReuseHarness(implicit p: Parameters) extends XSModule {
  val io = IO(new Bundle {
    val valid = Input(Bool())
    val first = Input(UInt(32.W))
    val second = Input(UInt(32.W))
    val fused = Output(Bool())
    val clearSecond = Output(Bool())
    val op = Output(FuOpType())
    val src2 = Output(UInt(LogicRegsWidth.W))
  })
  val decoder = Module(new FusionDecoder)
  decoder.io.disableFusion := false.B
  decoder.io.in := 0.U.asTypeOf(decoder.io.in)
  decoder.io.dec := 0.U.asTypeOf(decoder.io.dec)
  decoder.io.inReady.foreach(_ := true.B)
  decoder.io.in(0).valid := io.valid
  decoder.io.in(1).valid := io.valid
  decoder.io.in(0).bits := io.first
  decoder.io.in(1).bits := io.second
  decoder.io.dec(0).fuType := FuType.alu.U
  io.fused := decoder.io.out(0).valid
  io.clearSecond := decoder.io.clear(1)
  io.op := decoder.io.out(0).bits.fuOpType.bits
  io.src2 := decoder.io.out(0).bits.lsrc2.bits
}

class FusionReuseTest extends AnyFlatSpec with ChiselSim {
  private val base = new IntRegReuseConfig
  private implicit val p: Parameters = base.alterPartial {
    case XSCoreParamsKey => base(XSTileKey).head
    case DebugOptionsKey => DebugOptions(FPGAPlatform = true, AlwaysBasicDiff = false,
      EnablePerfDebug = false, AlwaysBasicDB = false)
    case utility.LogUtilsOptionsKey => utility.LogUtilsOptions(false, false, true)
    case utility.PerfCounterOptionsKey => utility.PerfCounterOptions(false, false, utility.XSPerfLevel.VERBOSE, 0)
  }
  behavior of "production fusion in the reuse configuration"
  it should "keep self-redefining shift-add fusion enabled with the actual second source" in {
    simulate(new FusionReuseHarness) { dut =>
      dut.io.valid.poke(true.B)
      // slli x5,x5,1; add x5,x5,x2 -> sh1add x5,x5,x2
      dut.io.first.poke("h00129293".U)
      dut.io.second.poke("h002282b3".U)
      dut.clock.step()
      dut.io.fused.expect(true.B)
      dut.io.clearSecond.expect(true.B)
      dut.io.op.expect(ALUOpType.sh1add)
      dut.io.src2.expect(2.U)
    }
  }
}
