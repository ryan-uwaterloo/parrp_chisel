/*
 * Copyright 2019 SiFive, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You should have received a copy of LICENSE.Apache2 along with
 * this software. If not, you may obtain a copy at
 *
 *    https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package parrp_chisel.blocks.inclusivecache

import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink._
import parrp_chisel.util._

class SourceARequest(params: InclusiveCacheParameters) extends InclusiveCacheBundle(params)
{
  val tag    = UInt(params.tagBits.W)
  val set    = UInt(params.setBits.W)
  val param  = UInt(3.W)
  val source = UInt(params.outer.bundle.sourceBits.W)
  val block  = Bool()
}

class SourceA(params: InclusiveCacheParameters) extends Module
{
  val io = IO(new Bundle {
    val req = Flipped(Decoupled(new SourceARequest(params)))
    val a = Decoupled(new TLBundleA(params.outer.bundle))
  })

  // ready must be a register, because we derive valid from ready
  require (!params.micro.outerBuf.a.pipe && params.micro.outerBuf.a.isDefined)

  val a = Wire(chiselTypeOf(io.a))
  io.a <> params.micro.outerBuf.a(a)

  io.req.ready := a.ready
  a.valid := io.req.valid
  params.ccover(a.valid && !a.ready, "SOURCEA_STALL", "Backpressured when issuing an Acquire")

  a.bits.opcode  := Mux(io.req.bits.block, TLMessages.AcquireBlock, TLMessages.AcquirePerm)
  a.bits.param   := io.req.bits.param
  a.bits.size    := params.offsetBits.U
  a.bits.source  := io.req.bits.source
  a.bits.address := params.expandAddress(io.req.bits.tag, io.req.bits.set, 0.U)
  a.bits.mask    := ~0.U(params.outer.manager.beatBytes.W)
  a.bits.data    := 0.U
  a.bits.corrupt := false.B

  // clock cycle counter
    val clk_cycle = RegInit(0.U(32.W))
    clk_cycle := clk_cycle + 1.U

  when (a.valid && a.ready){
    
    //printf_helper.ReqPrinter.printSrcA(a.bits)
    //printf(cf"New Source A Request! opcode: ${a.bits.opcode}, addr: 0x${a.bits.address}%x\n")
    when (a.bits.opcode === 0.U){
      printf(cf"@ clk_cycle ${clk_cycle}: New Source A Request! opcode: PutFullData, addr: 0x${a.bits.address}%x\n")
    } .elsewhen (a.bits.opcode === 1.U){
      printf(cf"@ clk_cycle ${clk_cycle}: New Source A Request! opcode: PutPartialData, addr: 0x${a.bits.address}%x\n")
    } .elsewhen (a.bits.opcode === 2.U){
      printf(cf"@ clk_cycle ${clk_cycle}: New Source A Request! opcode: ArithmeticData, addr: 0x${a.bits.address}%x\n")
    } .elsewhen (a.bits.opcode === 3.U) {
      printf(cf"@ clk_cycle ${clk_cycle}: New Source A Request! opcode: LogicalData, addr: 0x${a.bits.address}%x\n")
    } .elsewhen (a.bits.opcode === 4.U) {
      printf(cf"@ clk_cycle ${clk_cycle}: New Source A Request! opcode: Get, addr: 0x${a.bits.address}%x\n")
    } .elsewhen (a.bits.opcode === 5.U) {
      printf(cf"@ clk_cycle ${clk_cycle}: New Source A Request! opcode: Hint, addr: 0x${a.bits.address}%x\n")
    } .elsewhen (a.bits.opcode === 6.U) {
      printf(cf"@ clk_cycle ${clk_cycle}: New Source A Request! opcode: AcquireBlock, addr: 0x${a.bits.address}%x\n")
    } .elsewhen(a.bits.opcode === 7.U) {
      printf(cf"@ clk_cycle ${clk_cycle}: New Source A Request! opcode: AcquirePerm, addr: 0x${a.bits.address}%x\n")
    }}

  //   def a = Seq( ("PutFullData",TLPermissions.PermMsgReserved),
  //              ("PutPartialData",TLPermissions.PermMsgReserved),
  //              ("ArithmeticData",TLAtomics.ArithMsg),
  //              ("LogicalData",TLAtomics.LogicMsg),
  //              ("Get",TLPermissions.PermMsgReserved),
  //              ("Hint",TLHints.HintsMsg),
  //              ("AcquireBlock",TLPermissions.PermMsgGrow),
  //              ("AcquirePerm",TLPermissions.PermMsgGrow))
  // }
}