package xiangshan.backend.reuseintegration

import chisel3._
import chisel3.util._
import chisel3.simulator.scalatest.ChiselSim
import org.chipsalliance.cde.config.Parameters
import org.scalatest.flatspec.AnyFlatSpec
import top.IntRegReuseConfig
import xiangshan._
import xiangshan.backend.regfile.IntPregParams
import xiangshan.backend.rename.{RenameTable, Reg_I}

private class RatRecoveryHarness(implicit p: Parameters) extends XSModule {
  val io = IO(new Bundle {
    val spec = Input(Valid(new Bundle { val logical = UInt(5.W); val tag = UInt(PhyRegIdxWidth.W) }))
    val arch = Input(Valid(new Bundle { val logical = UInt(5.W); val tag = UInt(PhyRegIdxWidth.W) }))
    val read = Input(UInt(5.W))
    val tag = Output(UInt(PhyRegIdxWidth.W))
    val free = Output(Bool())
    val old = Output(UInt(PhyRegIdxWidth.W))
    val snapshot = Input(Bool())
    val redirect = Input(Bool())
    val useSnapshot = Input(Bool())
  })
  val rat = Module(new RenameTable(Reg_I, 0))
  rat.io.redirect := io.redirect
  rat.io.snpt := 0.U.asTypeOf(rat.io.snpt)
  rat.io.snpt.snptEnq := io.snapshot
  rat.io.snpt.useSnpt := io.useSnapshot
  for (r <- rat.io.readPorts) { r.hold := false.B; r.addr := io.read }
  rat.io.specWritePorts := 0.U.asTypeOf(rat.io.specWritePorts)
  rat.io.archWritePorts := 0.U.asTypeOf(rat.io.archWritePorts)
  rat.io.specWritePorts(0).wen := io.spec.valid
  rat.io.specWritePorts(0).addr := io.spec.bits.logical
  rat.io.specWritePorts(0).data := io.spec.bits.tag
  rat.io.archWritePorts(0).wen := io.arch.valid
  rat.io.archWritePorts(0).addr := io.arch.bits.logical
  rat.io.archWritePorts(0).data := io.arch.bits.tag
  io.tag := rat.io.readPorts(0).data
  io.free := rat.io.need_free(0)
  io.old := rat.io.old_pdest(0)
}

class RatRecoveryTest extends AnyFlatSpec with ChiselSim {
  private val base = new IntRegReuseConfig
  private implicit val p: Parameters = base.alterPartial {
    case utility.PerfCounterOptionsKey => utility.PerfCounterOptions(enablePerfPrint = false, enablePerfDB = false, perfLevel = utility.XSPerfLevel.VERBOSE, perfDBHartID = 0)
    case utility.LogUtilsOptionsKey => utility.LogUtilsOptions(enableDebug = false, enablePerf = false, fpgaPlatform = true)
    case XSCoreParamsKey => base(XSTileKey).head.copy(intPreg = IntPregParams(16, 4, None, None))
    case DebugOptionsKey => DebugOptions(FPGAPlatform = true, AlwaysBasicDiff = false,
      EnablePerfDebug = false, AlwaysBasicDB = false)
  }
  behavior of "versioned production RAT"
  it should "restore snapshot versions and release a base only after all aliases leave" in {
    simulate(new RatRecoveryHarness) { dut =>
      dut.io.spec.valid.poke(false.B); dut.io.spec.bits.logical.poke(0.U); dut.io.spec.bits.tag.poke(0.U)
      dut.io.arch.valid.poke(false.B); dut.io.arch.bits.logical.poke(0.U); dut.io.arch.bits.tag.poke(0.U)
      dut.io.read.poke(5.U); dut.io.snapshot.poke(false.B)
      dut.io.redirect.poke(false.B); dut.io.useSnapshot.poke(false.B)
      def spec(tag: Int): Unit = {
        dut.io.spec.valid.poke(true.B); dut.io.spec.bits.logical.poke(5.U); dut.io.spec.bits.tag.poke(tag.U)
        dut.clock.step(); dut.io.spec.valid.poke(false.B); dut.clock.step(3)
      }
      def arch(logical: Int, tag: Int, expectFree: Boolean): Unit = {
        dut.io.arch.valid.poke(true.B); dut.io.arch.bits.logical.poke(logical.U); dut.io.arch.bits.tag.poke(tag.U)
        dut.clock.step(); dut.io.arch.valid.poke(false.B); dut.clock.step()
        dut.io.free.expect(expectFree.B)
        dut.clock.step()
      }
      arch(5, 3, false)
      arch(6, 3, false) // move alias
      arch(5, 19, false) // v0 and v1 share base3
      arch(5, 4, false) // x6 still owns base3
      arch(6, 5, true) // last alias leaves: exactly one base release
      spec(19)
      dut.io.snapshot.poke(true.B); dut.clock.step(); dut.io.snapshot.poke(false.B); dut.clock.step(3)
      spec(7); dut.io.tag.expect(7.U)
      dut.io.useSnapshot.poke(true.B); dut.io.redirect.poke(true.B); dut.clock.step()
      dut.io.redirect.poke(false.B); dut.clock.step(4); dut.io.tag.expect(19.U)
      dut.io.useSnapshot.poke(false.B); dut.io.redirect.poke(true.B); dut.clock.step()
      dut.io.redirect.poke(false.B); dut.clock.step(4); dut.io.tag.expect(4.U)
    }
  }
}
