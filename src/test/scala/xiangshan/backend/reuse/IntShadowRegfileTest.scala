package xiangshan.backend.reuse

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.flatspec.AnyFlatSpec
import xiangshan.backend.regfile.IntShadowRegfile

class IntShadowRegfileTest extends AnyFlatSpec with ChiselSim {
  behavior of "row-local integer checkpoint cells"

  private def initialize(dut: IntShadowRegfile): Unit = {
    dut.io.redirect.poke(false.B)
    dut.io.kill.foreach(_.foreach(_.poke(false.B)))
    dut.io.retired.foreach(_.foreach(_.poke(false.B)))
    for (a <- dut.io.allocate) { a.valid.poke(false.B); a.bits.tag.poke(0.U); a.bits.owner.poke(0.U) }
    for (c <- dut.io.commit) { c.valid.poke(false.B); c.bits.tag.poke(0.U); c.bits.owner.poke(0.U) }
    for (w <- dut.io.write) { w.valid.poke(false.B); w.bits.tag.poke(0.U); w.bits.owner.poke(0.U); w.bits.data.poke(0.U) }
    for (port <- dut.io.readAddr; addr <- port) addr.poke(0.U)
  }

  it should "reserve before v0 stores, read only main, and restore locally after squash" in {
    simulate(new IntShadowRegfile(8, 2, 1, 2, 2, 64, 8)) { dut =>
      initialize(dut)
      for (v <- 0 until 2) {
        dut.io.allocate(v).valid.poke(true.B)
        dut.io.allocate(v).bits.tag.poke((3 + 8 * v).U)
        dut.io.allocate(v).bits.owner.poke((10 + v).U)
      }
      dut.clock.step(); dut.io.allocate.foreach(_.valid.poke(false.B))
      dut.io.canIssueReuse(3).expect(false.B)
      val w = dut.io.write(0)
      w.valid.poke(true.B); w.bits.tag.poke(3.U); w.bits.owner.poke(10.U); w.bits.data.poke(100.U)
      dut.clock.step(); w.valid.poke(false.B)
      dut.io.canIssueReuse(3).expect(true.B)
      w.valid.poke(true.B); w.bits.tag.poke(11.U); w.bits.owner.poke(11.U); w.bits.data.poke(101.U)
      dut.io.checkpointed(3).expect(true.B)
      dut.clock.step(); w.valid.poke(false.B)
      dut.io.debugData(3).expect(100.U) // observation only
      dut.io.debugData(11).expect(101.U)
      dut.io.residentReady(3).expect(false.B)
      dut.io.residentReady(11).expect(true.B)
      // Even a v0 address returns main, NEVER shadow. Caller must check residency.
      dut.io.readAddr(0)(1).poke(1.U)
      dut.clock.step(); dut.io.readData(0)(1).expect(101.U)
      dut.io.readAddr(0)(1).poke(5.U)
      dut.clock.step(); dut.io.readData(0)(1).expect(101.U)
      dut.io.redirect.poke(true.B); dut.io.kill(3)(1).poke(true.B)
      dut.io.recoveryBusy.expect(true.B)
      dut.clock.step(); dut.io.redirect.poke(false.B); dut.io.kill(3)(1).poke(false.B)
      dut.io.recoveryBusy.expect(true.B); dut.io.recovered.valid.expect(true.B)
      dut.io.recovered.bits.expect(3.U)
      dut.clock.step()
      dut.io.recoveryBusy.expect(false.B); dut.io.residentReady(3).expect(true.B)
      dut.io.residentReady(11).expect(false.B)
      dut.io.readData(0)(1).expect(100.U)
      // Canceled v1 cannot write after restoration, even with its original owner.
      w.valid.poke(true.B); w.bits.tag.poke(11.U); w.bits.owner.poke(11.U); w.bits.data.poke(999.U)
      dut.clock.step(); w.valid.poke(false.B)
      dut.io.readData(0)(1).expect(100.U)
    }
  }

  it should "restore one pending row per cycle, retain committed versions, and discard killed pairs" in {
    simulate(new IntShadowRegfile(8, 2, 1, 4, 4, 64, 8)) { dut =>
      initialize(dut)
      for (i <- 0 until 4) {
        dut.io.allocate(i).valid.poke(true.B)
        dut.io.allocate(i).bits.tag.poke((1 + i / 2 + (i % 2) * 8).U)
        dut.io.allocate(i).bits.owner.poke((20 + i).U)
      }
      dut.clock.step(); dut.io.allocate.foreach(_.valid.poke(false.B))
      for (i <- 0 until 2) {
        dut.io.write(i).valid.poke(true.B); dut.io.write(i).bits.tag.poke((1 + i).U)
        dut.io.write(i).bits.owner.poke((20 + 2 * i).U); dut.io.write(i).bits.data.poke((10 + i).U)
      }
      dut.clock.step()
      for (i <- 0 until 2) {
        dut.io.write(i).bits.tag.poke((9 + i).U)
        dut.io.write(i).bits.owner.poke((21 + 2 * i).U); dut.io.write(i).bits.data.poke((30 + i).U)
      }
      dut.clock.step(); dut.io.write.foreach(_.valid.poke(false.B))
      dut.io.redirect.poke(true.B)
      dut.io.kill(1)(1).poke(true.B); dut.io.kill(2)(1).poke(true.B)
      dut.clock.step(); dut.io.redirect.poke(false.B); dut.io.kill.foreach(_.foreach(_.poke(false.B)))
      dut.io.recovered.bits.expect(1.U); dut.clock.step()
      dut.io.recovered.bits.expect(2.U); dut.io.recoveryBusy.expect(true.B); dut.clock.step()
      dut.io.recoveryBusy.expect(false.B)
      dut.io.residentReady(1).expect(true.B); dut.io.residentReady(2).expect(true.B)
      // Fresh allocation begins a new lifetime; commit protects against stale age.
      dut.io.allocate(0).valid.poke(true.B); dut.io.allocate(0).bits.tag.poke(1.U); dut.io.allocate(0).bits.owner.poke(40.U)
      dut.clock.step(); dut.io.allocate(0).valid.poke(false.B)
      dut.io.retired(1)(0).poke(true.B) // ROB retirement can precede RAB commit
      dut.clock.step(); dut.io.retired(1)(0).poke(false.B)
      dut.io.kill(1)(0).poke(true.B); dut.clock.step(); dut.io.kill(1)(0).poke(false.B)
      dut.io.versionLive(1)(0).expect(true.B)
      // Both old and new owners canceled: no old state needs restoration.
      dut.io.kill(2)(0).poke(true.B); dut.io.kill(2)(1).poke(true.B)
      dut.clock.step(); dut.io.kill.foreach(_.foreach(_.poke(false.B)))
      dut.io.recoveryBusy.expect(false.B); dut.io.versionLive(2)(0).expect(false.B)
    }
  }

  it should "match randomized local saves and restores without exposing a shadow read port" in {
    simulate(new IntShadowRegfile(12, 4, 1, 2, 2, 64, 12, debugEnabled = false)) { dut =>
      initialize(dut)
      val rng = new scala.util.Random(20261108L)
      for (iteration <- 0 until 100) {
        val base = 1 + rng.nextInt(11)
        val old = BigInt(64, rng); val young = BigInt(64, rng)
        for (v <- 0 until 2) {
          dut.io.allocate(v).valid.poke(true.B)
          dut.io.allocate(v).bits.tag.poke((base + 16 * v).U)
          dut.io.allocate(v).bits.owner.poke((iteration * 2 + v + 1).U)
        }
        dut.clock.step(); dut.io.allocate.foreach(_.valid.poke(false.B))
        dut.io.canIssueReuse(base).expect(false.B)
        val w = dut.io.write(0)
        w.valid.poke(true.B); w.bits.tag.poke(base.U); w.bits.owner.poke((iteration * 2 + 1).U); w.bits.data.poke(old.U)
        dut.clock.step(); w.valid.poke(false.B)
        dut.io.canIssueReuse(base).expect(true.B)
        w.valid.poke(true.B); w.bits.tag.poke((base + 16).U); w.bits.owner.poke((iteration * 2 + 2).U); w.bits.data.poke(young.U)
        dut.clock.step(); w.valid.poke(false.B)
        dut.io.readAddr(0)(base % 4).poke((base / 4).U)
        dut.clock.step(); dut.io.readData(0)(base % 4).expect(young.U)
        dut.io.debugData(base).expect(0.U)
        dut.io.redirect.poke(true.B); dut.io.kill(base)(1).poke(true.B)
        dut.clock.step(); dut.io.redirect.poke(false.B); dut.io.kill(base)(1).poke(false.B)
        dut.clock.step()
        dut.io.readData(0)(base % 4).expect(old.U)
        dut.io.residentReady(base).expect(true.B)
      }
    }
  }
}
