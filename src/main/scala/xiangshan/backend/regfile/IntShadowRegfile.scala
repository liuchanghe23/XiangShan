package xiangshan.backend.regfile

import chisel3._
import chisel3.util._

/** Main-only operand access with one row-local checkpoint cell.
  * shadow.D is connected exclusively to main.Q. shadow.Q feeds only its own
  * main's restore mux and optional simulation observation, never operand reads.
  * A reuse may be reserved before v0 exists; its execution must wait for
  * canIssueReuse. This avoids stalling a non-backpressurable fixed-latency WB.
  */
class IntShadowRegfile(
  numBases: Int, banks: Int, reads: Int, writes: Int, allocations: Int,
  dataBits: Int, ownerBits: Int, debugEnabled: Boolean = true
) extends Module {
  require(numBases > 1 && isPow2(banks) && numBases % banks == 0)
  private val baseBits = log2Ceil(numBases)
  private val tagBits = baseBits + 1
  private val bankBits = log2Ceil(banks)
  private val tagStride = 1 << baseBits
  class Identity extends Bundle {
    val tag = UInt(tagBits.W)
    val owner = UInt(ownerBits.W)
  }
  val io = IO(new Bundle {
    val readAddr = Input(Vec(reads, Vec(banks, UInt((tagBits - bankBits).W))))
    val readData = Output(Vec(reads, Vec(banks, UInt(dataBits.W))))
    val allocate = Input(Vec(allocations, Valid(new Identity)))
    val commit = Input(Vec(allocations, Valid(new Identity)))
    val write = Input(Vec(writes, Valid(new Bundle {
      val tag = UInt(tagBits.W)
      val owner = UInt(ownerBits.W)
      val data = UInt(dataBits.W)
    })))
    // The integration compares full circular ROB identities, including slots.
    val redirect = Input(Bool())
    val kill = Input(Vec(numBases, Vec(2, Bool())))
    val retired = Input(Vec(numBases, Vec(2, Bool())))
    val versionOwner = Output(Vec(numBases, Vec(2, UInt(ownerBits.W))))
    val versionLive = Output(Vec(numBases, Vec(2, Bool())))
    val versionCommitted = Output(Vec(numBases, Vec(2, Bool())))
    val residentReady = Output(Vec(2 * tagStride, Bool()))
    val canIssueReuse = Output(Vec(numBases, Bool()))
    val recoveryBusy = Output(Bool())
    val recovered = Output(Valid(UInt(baseBits.W)))
    val checkpointed = Output(Vec(numBases, Bool()))
    // Simulation observation is kept separate from the functional read path.
    val debugData = Output(Vec(2 * tagStride, UInt(dataBits.W)))
    val debugValid = Output(Vec(2 * tagStride, Bool()))
  })

  private val main = Reg(Vec(numBases, UInt(dataBits.W)))
  private val shadow = Reg(Vec(numBases, UInt(dataBits.W)))
  private val mainIsV1 = RegInit(VecInit.fill(numBases)(false.B))
  private val shadowValid = RegInit(VecInit.fill(numBases)(false.B))
  private val allocated = RegInit(VecInit.fill(numBases)(VecInit.fill(2)(false.B)))
  private val written = RegInit(VecInit.fill(numBases)(VecInit.fill(2)(false.B)))
  private val committed = RegInit(VecInit.fill(numBases)(VecInit.fill(2)(false.B)))
  private val owner = Reg(Vec(numBases, Vec(2, UInt(ownerBits.W))))
  private val pendingRestore = RegInit(VecInit.fill(numBases)(false.B))
  private val restoreOH = PriorityEncoderOH(pendingRestore.asUInt)

  io.versionOwner := owner
  io.versionLive := allocated
  io.versionCommitted := committed
  io.recoveryBusy := io.redirect || pendingRestore.asUInt.orR
  io.residentReady := 0.U.asTypeOf(io.residentReady)
  io.canIssueReuse := 0.U.asTypeOf(io.canIssueReuse)
  io.recovered.valid := false.B
  io.recovered.bits := 0.U
  io.checkpointed := 0.U.asTypeOf(io.checkpointed)
  io.debugData := 0.U.asTypeOf(io.debugData)
  io.debugValid := 0.U.asTypeOf(io.debugValid)

  for (base <- 1 until numBases) {
    val alloc = (0 until 2).map(v => io.allocate.map(a => a.valid && a.bits.tag === (base + v * tagStride).U))
    val fresh = alloc(0).reduce(_ || _)
    val killed = (0 until 2).map(v => io.kill(base)(v) && !committed(base)(v))
    for (v <- 0 until 2) {
      val retire = io.commit.map(c => c.valid && c.bits.tag === (base + v * tagStride).U &&
        c.bits.owner === owner(base)(v) && allocated(base)(v)).reduce(_ || _)
      assert(PopCount(alloc(v)) <= 1.U, "duplicate integer version allocation")
      when(retire || io.retired(base)(v)) { committed(base)(v) := true.B }
      when(killed(v)) { allocated(base)(v) := false.B; written(base)(v) := false.B }
      when(fresh) {
        allocated(base)(v) := false.B
        written(base)(v) := false.B
        committed(base)(v) := false.B
      }
      when(alloc(v).reduce(_ || _)) {
        allocated(base)(v) := true.B
        written(base)(v) := false.B
        committed(base)(v) := false.B
        owner(base)(v) := Mux1H(alloc(v), io.allocate.map(_.bits.owner))
      }
    }
    val hit = (0 until 2).map(v => io.write.map(w =>
      w.valid && w.bits.tag === (base + v * tagStride).U && allocated(base)(v) &&
        w.bits.owner === owner(base)(v) && !fresh && !killed(v)))
    val wen = hit.map(_.reduce(_ || _))
    val data = hit.map(h => Mux1H(h, io.write.map(_.bits.data)))
    for (v <- 0 until 2) {
      assert(PopCount(hit(v)) <= 1.U, "two writes to the same integer version")
      when(wen(v)) { written(base)(v) := true.B }
    }
    // Restore one pending row per cycle. Other-row surviving writebacks proceed.
    val restore = restoreOH(base) && !io.redirect && !fresh &&
      allocated(base)(0) && !killed(0) && !wen.reduce(_ || _)
    when(killed(1) && mainIsV1(base) && allocated(base)(0) && !killed(0)) {
      pendingRestore(base) := true.B
    }
    when(killed(0) || fresh) { pendingRestore(base) := false.B }
    when(fresh) {
      mainIsV1(base) := false.B
      shadowValid(base) := false.B
    }.elsewhen(restore) {
      assert(shadowValid(base), "restoring v0 without a checkpoint")
      main(base) := shadow(base)
      mainIsV1(base) := false.B
      shadowValid(base) := false.B
      pendingRestore(base) := false.B
      io.recovered.valid := true.B
      io.recovered.bits := base.U
    }.elsewhen(wen(1)) {
      // v1 is issued only after the previous producer really stored v0.
      assert(!wen(0), "v0/v1 write collision must be prevented before issue")
      assert(written(base)(0) && allocated(base)(0), "v1 before v0 is stored")
      assert(!mainIsV1(base) && !shadowValid(base), "repeated destructive v1 write")
      shadow(base) := main(base) // the ONLY functional input to the shadow cell
      shadowValid(base) := true.B
      main(base) := data(1)
      mainIsV1(base) := true.B
      io.checkpointed(base) := true.B
    }.elsewhen(wen(0)) {
      assert(!mainIsV1(base), "late v0 write after destructive reuse")
      main(base) := data(0)
    }
    val available = !io.recoveryBusy && !killed(0) && !killed(1)
    io.residentReady(base) := available && allocated(base)(0) && written(base)(0) && !mainIsV1(base)
    io.residentReady(base + tagStride) := available && allocated(base)(1) && written(base)(1) && mainIsV1(base)
    io.canIssueReuse(base) := io.residentReady(base) && allocated(base)(1) && !written(base)(1)
    if (debugEnabled) {
      io.debugData(base) := Mux(mainIsV1(base), shadow(base), main(base))
      io.debugData(base + tagStride) := main(base)
      io.debugValid(base) := allocated(base)(0) && written(base)(0)
      io.debugValid(base + tagStride) := allocated(base)(1) && written(base)(1)
    }
  }
  io.residentReady(0) := true.B
  io.debugValid(0) := true.B
  // Ordinary banked operand reads depend ONLY on main. The caller validates
  // version residency before accepting a read; debugData must never feed them.
  val mainForRead = Wire(Vec(numBases, UInt(dataBits.W)))
  mainForRead := main
  mainForRead(0) := 0.U
  for (port <- 0 until reads; bank <- 0 until banks) {
    val row = RegNext(io.readAddr(port)(bank)(baseBits - bankBits - 1, 0))
    val bankData = VecInit((bank until numBases by banks).map(mainForRead(_)))
    io.readData(port)(bank) := bankData(row)
  }
  for (a <- io.allocate) {
    when(a.valid) {
      assert(!io.recoveryBusy, "allocation during local recovery")
      assert(a.bits.tag(baseBits - 1, 0) =/= 0.U && a.bits.tag(baseBits - 1, 0) < numBases.U)
    }
  }
}
