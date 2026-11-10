package xiangshan.backend.reuseintegration

import chisel3._
import chisel3.util._
import chisel3.reflect.DataMirror
import chisel3.simulator.scalatest.ChiselSim
import org.chipsalliance.cde.config.Parameters
import org.scalatest.flatspec.AnyFlatSpec
import top.IntRegReuseConfig
import xiangshan._
import xiangshan.backend.fu.FuType
import xiangshan.backend.regfile.IntPregParams
import xiangshan.backend.rename.Rename

private class RenameReuseHarness(implicit p: Parameters) extends XSModule {
  val io = IO(new Bundle {
    val offer = Input(Bool())
    val stall = Input(Bool())
    val valid = Input(Vec(RenameWidth, Bool()))
    val lsrc = Input(Vec(RenameWidth, UInt(5.W)))
    val ldest = Input(Vec(RenameWidth, UInt(5.W)))
    val move = Input(Vec(RenameWidth, Bool()))
    val ready = Output(Bool())
    val pdest = Output(Vec(RenameWidth, UInt(PhyRegIdxWidth.W)))
    val psrc = Output(Vec(RenameWidth, UInt(PhyRegIdxWidth.W)))
    val rob = Output(Vec(RenameWidth, UInt(log2Ceil(RobSize).W)))
  })
  val dut = Module(new Rename)
  def defaultInputs(data: Data): Unit = data match {
    case a: Aggregate => a.getElements.foreach(defaultInputs)
    case leaf if DataMirror.directionOf(leaf) == ActualDirection.Input => leaf := 0.U.asTypeOf(leaf)
    case _ =>
  }
  defaultInputs(dut.io)
  dut.io.toLsqEnqCtrl.canAccept := true.B
  for (i <- 0 until RenameWidth) {
    dut.io.out(i).ready := !io.stall
    dut.io.in(i).valid := io.offer && io.valid(i)
    dut.io.validVec(i) := io.offer && io.valid(i)
    dut.io.in(i).bits.rfWen := true.B
    dut.io.in(i).bits.fuType := FuType.alu.U
    dut.io.in(i).bits.fuOpType := ALUOpType.add
    dut.io.in(i).bits.srcType(0) := SrcType.xp
    dut.io.in(i).bits.srcType(1) := SrcType.imm
    dut.io.in(i).bits.srcType(2) := SrcType.no
    dut.io.in(i).bits.lsrc(0) := io.lsrc(i)
    dut.io.in(i).bits.ldest := io.ldest(i)
    dut.io.in(i).bits.firstUop := true.B
    dut.io.in(i).bits.lastUop := true.B
    dut.io.in(i).bits.numWB := 1.U
    dut.io.in(i).bits.simple := true.B
    dut.io.in(i).bits.isMove := io.move(i)
    dut.io.intReadPorts(i)(0).addr := io.lsrc(i)
    io.pdest(i) := dut.io.out(i).bits.pdest
    io.psrc(i) := dut.io.out(i).bits.psrc(0)
    io.rob(i) := dut.io.out(i).bits.robIdx.value
  }
  io.ready := dut.io.in.head.ready
}

class RenameReuseTest extends AnyFlatSpec with ChiselSim {
  private val base = new IntRegReuseConfig
  private implicit val p: Parameters = base.alterPartial {
    case utility.PerfCounterOptionsKey => utility.PerfCounterOptions(enablePerfPrint = false, enablePerfDB = false, perfLevel = utility.XSPerfLevel.VERBOSE, perfDBHartID = 0)
    case utility.LogUtilsOptionsKey => utility.LogUtilsOptions(enableDebug = false, enablePerf = false, fpgaPlatform = true)
    case XSCoreParamsKey => base(XSTileKey).head.copy(intPreg = IntPregParams(16, 4, None, None))
    case DebugOptionsKey => DebugOptions(FPGAPlatform = true, AlwaysBasicDiff = false,
      EnablePerfDebug = false, AlwaysBasicDB = false)
  }
  behavior of "production Rename with integer reuse and ROB compression"
  it should "forward eight dependent lanes through alternating fresh and shadow destinations" in {
    simulate(new RenameReuseHarness) { dut =>
      dut.io.offer.poke(false.B); dut.io.stall.poke(false.B)
      for (i <- 0 until 8) {
        dut.io.valid(i).poke(true.B); dut.io.move(i).poke(false.B)
        dut.io.lsrc(i).poke((if (i == 0) 1 else 5).U); dut.io.ldest(i).poke(5.U)
      }
      dut.clock.step(3)
      dut.io.offer.poke(true.B)
      dut.io.ready.expect(true.B)
      val tags = Seq(1, 17, 2, 18, 3, 19, 4, 20)
      for (i <- 0 until 8) {
        dut.io.pdest(i).expect(tags(i).U)
        dut.io.psrc(i).expect((if (i == 0) 0 else tags(i - 1)).U)
        dut.io.rob(i).expect(0.U) // all simple members share one compressed entry
      }
      dut.io.stall.poke(true.B); dut.clock.step(2)
      for (i <- 0 until 8) dut.io.pdest(i).expect(tags(i).U)
      dut.io.stall.poke(false.B); dut.clock.step()
      dut.io.offer.poke(false.B)
      // A move sees the full v1 identity. A following redefinition uses fresh.
      for (i <- 0 until 8) {
        dut.io.valid(i).poke((i < 2).B)
        dut.io.lsrc(i).poke(5.U)
        dut.io.ldest(i).poke((if (i == 0) 6 else 5).U)
        dut.io.move(i).poke((i == 0).B)
      }
      dut.clock.step(3); dut.io.offer.poke(true.B)
      dut.io.pdest(0).expect(20.U); dut.io.pdest(1).expect(5.U)
      dut.io.psrc(1).expect(20.U)
      dut.clock.step()
    }
  }

  it should "admit a full reuse bundle with zero free base registers" in {
    simulate(new RenameReuseHarness) { dut =>
      dut.io.offer.poke(false.B); dut.io.stall.poke(false.B)
      for (i <- 0 until 8) {
        dut.io.valid(i).poke(true.B); dut.io.move(i).poke(false.B)
        dut.io.lsrc(i).poke(0.U); dut.io.ldest(i).poke((i + 1).U)
      }
      dut.clock.step(3); dut.io.offer.poke(true.B); dut.io.ready.expect(true.B)
      dut.clock.step(); dut.io.offer.poke(false.B)
      for (i <- 0 until 8) {
        dut.io.valid(i).poke((i < 7).B)
        dut.io.ldest(i).poke((i + 9).U)
      }
      dut.clock.step(3); dut.io.offer.poke(true.B); dut.io.ready.expect(true.B)
      dut.clock.step(); dut.io.offer.poke(false.B)
      for (i <- 0 until 8) {
        dut.io.valid(i).poke(true.B)
        dut.io.lsrc(i).poke((i + 1).U); dut.io.ldest(i).poke((i + 1).U)
      }
      dut.clock.step(3); dut.io.offer.poke(true.B)
      dut.io.ready.expect(true.B)
      for (i <- 0 until 8) dut.io.pdest(i).expect((i + 17).U)
      dut.clock.step(); dut.io.offer.poke(false.B)
    }
  }


  it should "carry version bit eight in the default 224-base configuration" in {
    val full = p.alterPartial { case XSCoreParamsKey => base(XSTileKey).head }
    simulate(new RenameReuseHarness()(full)) { dut =>
      dut.io.offer.poke(false.B); dut.io.stall.poke(false.B)
      for (i <- 0 until 8) {
        dut.io.valid(i).poke(true.B); dut.io.move(i).poke(false.B)
        dut.io.lsrc(i).poke((if (i == 0) 1 else 5).U); dut.io.ldest(i).poke(5.U)
      }
      dut.clock.step(3); dut.io.offer.poke(true.B); dut.io.ready.expect(true.B)
      val tags = Seq(1, 257, 2, 258, 3, 259, 4, 260)
      for (i <- 0 until 8) {
        dut.io.pdest(i).expect(tags(i).U)
        dut.io.psrc(i).expect((if (i == 0) 0 else tags(i - 1)).U)
      }
      dut.clock.step()
    }
  }

}
