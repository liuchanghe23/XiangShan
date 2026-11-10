package xiangshan.backend.rename

import chisel3._
import org.chipsalliance.cde.config.Parameters
import xiangshan._
import xiangshan.backend.rob.RobPtr

/** One accepted destination identity. Move aliases do not allocate an identity. */
class IntReuseAllocation(implicit p: Parameters) extends XSBundle {
  val tag = UInt(IntPhyRegIdxWidth.W)
  val owner = new RobPtr
}

/** Whole physical ROB entry retirement; NORMAL entries contain only former. */
class IntReuseRetirement(implicit p: Parameters) extends XSBundle {
  val owner = new RobPtr
  val hasLatter = Bool()
}
