package xiangshan.backend.rename

import chisel3._
import chisel3.util._
/** Ordered rename metadata, independent of producer readiness and commit.
  * A base admits v0 -> v1 once per fresh lifetime. Wrong-path consumption and
  * reservations remain sticky until a fresh allocation, so no tag is reissued
  * merely because a snapshot restored an older mapping.
  */
class IntRegisterReuse(numBases: Int, lanes: Int, sources: Int = 2) extends Module {
  require(numBases > 1 && lanes > 0 && sources > 0)
  private val baseBits = log2Ceil(numBases)
  private val tagBits = baseBits + 1
  val io = IO(new Bundle {
    val accept = Input(Bool())
    val valid = Input(Vec(lanes, Bool()))
    val eligibleOp = Input(Vec(lanes, Bool()))
    val selfSource = Input(Vec(lanes, UInt(tagBits.W)))
    val src = Input(Vec(lanes, Vec(sources, Valid(UInt(tagBits.W)))))
    val fresh = Input(Vec(lanes, Bool()))
    val dest = Input(Vec(lanes, UInt(tagBits.W)))
    val reuse = Output(Vec(lanes, Bool()))
  })

  private val used = RegInit(VecInit.fill(numBases)(false.B))
  private val read = RegInit(VecInit.fill(numBases)(false.B))
  private val usedStage = Wire(Vec(lanes + 1, Vec(numBases, Bool())))
  private val readStage = Wire(Vec(lanes + 1, Vec(numBases, Bool())))
  usedStage(0) := used
  readStage(0) := read
  for (lane <- 0 until lanes) {
    val source = io.selfSource(lane)
    val sourceBase = source(baseBits - 1, 0)
    io.reuse(lane) := io.valid(lane) && io.eligibleOp(lane) &&
      !source(baseBits) && sourceBase =/= 0.U && sourceBase < numBases.U &&
      !usedStage(lane)(sourceBase) && !readStage(lane)(sourceBase)
    for (base <- 0 until numBases) {
      val fresh = io.valid(lane) && io.fresh(lane) && io.dest(lane)(baseBits - 1, 0) === base.U
      val reused = io.reuse(lane) && sourceBase === base.U
      val consumed = io.valid(lane) && io.src(lane).map(s =>
        s.valid && !s.bits(baseBits) && s.bits(baseBits - 1, 0) === base.U
      ).reduce(_ || _)
      usedStage(lane + 1)(base) := Mux(fresh, false.B, usedStage(lane)(base)) || reused
      readStage(lane + 1)(base) := Mux(fresh, false.B, readStage(lane)(base)) || consumed
    }
  }
  when(io.accept) {
    used := usedStage(lanes)
    read := readStage(lanes)
  }
}
