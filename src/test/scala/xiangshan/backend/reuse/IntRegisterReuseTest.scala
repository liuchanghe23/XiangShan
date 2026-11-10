package xiangshan.backend.reuse

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.flatspec.AnyFlatSpec
import xiangshan.backend.rename.IntRegisterReuse

class IntRegisterReuseTest extends AnyFlatSpec with ChiselSim {
  behavior of "ordered integer reuse qualification"

  it should "accept dependent lanes across fresh allocation boundaries without readiness" in {
    simulate(new IntRegisterReuse(16, 8)) { dut =>
      dut.io.accept.poke(false.B)
      for (i <- 0 until 8) {
        dut.io.valid(i).poke(true.B)
        dut.io.eligibleOp(i).poke((i % 2 == 1).B)
        dut.io.selfSource(i).poke((i / 2 + 1).U)
        dut.io.fresh(i).poke((i % 2 == 0).B)
        dut.io.dest(i).poke((i / 2 + 1 + (if (i % 2 == 1) 16 else 0)).U)
        dut.io.src(i)(0).valid.poke((i > 0).B)
        dut.io.src(i)(0).bits.poke((if (i % 2 == 1) i / 2 + 1 else i / 2 + 16).U)
        dut.io.src(i)(1).valid.poke(false.B)
        dut.io.src(i)(1).bits.poke(0.U)
      }
      for (i <- 0 until 8) dut.io.reuse(i).expect((i % 2 == 1).B)
      // A stalled bundle must not reserve versions or register consumers.
      dut.clock.step(3)
      for (i <- 0 until 8) dut.io.reuse(i).expect((i % 2 == 1).B)
      dut.io.accept.poke(true.B)
      dut.clock.step()
    }
  }

  it should "allow eight independent self sources connected by other-source RAW edges" in {
    simulate(new IntRegisterReuse(16, 8)) { dut =>
      dut.io.accept.poke(true.B)
      for (i <- 0 until 8) {
        dut.io.valid(i).poke(true.B)
        dut.io.eligibleOp(i).poke(true.B)
        dut.io.selfSource(i).poke((i + 1).U)
        dut.io.fresh(i).poke(false.B)
        dut.io.dest(i).poke((i + 17).U)
        dut.io.src(i)(0).valid.poke(true.B)
        dut.io.src(i)(0).bits.poke((i + 1).U)
        dut.io.src(i)(1).valid.poke((i > 0).B)
        dut.io.src(i)(1).bits.poke((i + 16).U)
        dut.io.reuse(i).expect(true.B)
      }
      dut.clock.step()
      // Restoring an old mapping does not reissue the already reserved v1 tag.
      for (i <- 0 until 8) dut.io.reuse(i).expect(false.B)
    }
  }

  it should "reject older readers and move aliases and deduplicate repeated sources" in {
    simulate(new IntRegisterReuse(16, 8)) { dut =>
      dut.io.accept.poke(false.B)
      for (i <- 0 until 8) {
        dut.io.valid(i).poke((i < 2).B)
        dut.io.eligibleOp(i).poke((i == 1).B)
        dut.io.selfSource(i).poke(3.U)
        dut.io.fresh(i).poke(false.B)
        dut.io.dest(i).poke((if (i == 0) 3 else 19).U)
        for (j <- 0 until 2) {
          dut.io.src(i)(j).valid.poke(true.B)
          dut.io.src(i)(j).bits.poke(3.U)
        }
      }
      dut.io.reuse(1).expect(false.B) // lane0 is a move/other reader
      dut.io.valid(0).poke(false.B)
      dut.io.reuse(1).expect(true.B) // two identical operands count as one instruction
      dut.io.selfSource(1).poke(0.U)
      dut.io.reuse(1).expect(false.B)
      dut.io.selfSource(1).poke(19.U)
      dut.io.reuse(1).expect(false.B) // one shadow cannot hold a third version
    }
  }
}
