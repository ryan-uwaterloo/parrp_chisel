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
import org.chipsalliance.cde.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.MuxT
import MetaData._
import chisel3.experimental.dataview.BundleUpcastable
import freechips.rocketchip.util.DescribedSRAM
import scala.math.{log, ceil}

class DirectoryEntry(params: InclusiveCacheParameters) extends InclusiveCacheBundle(params)
{
  val dirty   = Bool() // true => TRUNK or TIP
  val state   = UInt(params.stateBits.W)
  val clients = UInt(params.clientBits.W)
  val tag     = UInt(params.tagBits.W)
  val in_core = UInt(params.coreIndexMap.size.W) //teehee I might make a coreBits later
}

class DirectoryWrite(params: InclusiveCacheParameters) extends InclusiveCacheBundle(params)
{
  val set  = UInt(params.setBits.W)
  val way  = UInt(params.wayBits.W)
  val data = new DirectoryEntry(params)
  val core = UInt(log2Ceil(params.coreIndexMap.size + 1).W)//I think this is feasible to store in MSHR
  val parrp_data_vec = Vec(params.partitionSize, new ParrpEntry(params))
}

class DirectoryRead(params: InclusiveCacheParameters) extends InclusiveCacheBundle(params)
{
  val set = UInt(params.setBits.W)
  val tag = UInt(params.tagBits.W)
  val sourceId = UInt(params.inner.bundle.sourceBits.W)//add sourceId to read
}

class DirectoryResult(params: InclusiveCacheParameters) extends DirectoryEntry(params)
{
  val hit = Bool()
  val way = UInt(params.wayBits.W)
  val core = UInt(log2Ceil(params.coreIndexMap.size + 1).W) //return this and store in MSHR to return later, I think this is fine to do, saves hw for write
  val parrp_data_vec = Vec(params.partitionSize, new ParrpEntry(params))//of the same type as one core of the read-out val. (read out one partition to update)
  val parrp_way = UInt(log2Ceil(params.partitionSize).W)
  val parrp_hit = Bool()
}

class ParrpEntry(params: InclusiveCacheParameters) extends Bundle{//have one parrp entry per core per way per set
  val state = UInt(2.W)//assign this to a ParrpState
  val lru = UInt((log2Ceil(params.partitionSize)).W)//lru bits is how many bits to store max age (num ways in partition)
  val way_index = UInt(params.wayBits.W)//way index is based on phy way num
}

object ParrpStates{ // states, yippee
  def invalid = 0.U(2.W)
  def deallocated = 1.U(2.W)
  def allocated = 2.U(2.W)
  def reserved = 3.U(2.W)
}

class Directory(params: InclusiveCacheParameters) extends Module
{
  val io = IO(new Bundle {
    val write  = Flipped(Decoupled(new DirectoryWrite(params)))
    val read   = Flipped(Valid(new DirectoryRead(params))) // sees same-cycle write
    val result = Valid(new DirectoryResult(params))
    val ready  = Bool() // reset complete; can enable access
  })

  val codeBits = new DirectoryEntry(params).getWidth

  val cc_dir =  DescribedSRAM(
    name = "cc_dir",
    desc = "Directory RAM",
    size = params.cache.sets,
    data = Vec(params.cache.ways, new DirectoryEntry(params)) //edit to type for my sanity lmao
  )

  val write = Queue(io.write, 1) // must inspect contents => max size 1
  // a flow Q creates a WaR hazard... this MIGHT not cause a problem
  // a pipe Q causes combinational loop through the scheduler

  // Wiping the Directory with 0s on reset has ultimate priority
  val wipeCount = RegInit(0.U((params.setBits + 1).W))
  val wipeOff = RegNext(false.B, true.B) // don't wipe tags during reset
  val wipeDone = wipeCount(params.setBits)
  val wipeSet = wipeCount(params.setBits - 1,0)

  io.ready := wipeDone
  when (!wipeDone && !wipeOff) { wipeCount := wipeCount + 1.U }
  assert (wipeDone || !io.read.valid)

  // Be explicit for dumb 1-port inference
  val ren = io.read.valid
  val wen = (!wipeDone && !wipeOff) || write.valid
  assert (!io.read.valid || wipeDone)

  require (codeBits <= 256)

  write.ready := !io.read.valid
  when (!ren && wen) {
    cc_dir.write(
      Mux(wipeDone, write.bits.set, wipeSet),
      VecInit.fill(params.cache.ways) { Mux(wipeDone, write.bits.data, 0.U.asTypeOf(new DirectoryEntry(params))) },
      UIntToOH(write.bits.way, params.cache.ways).asBools.map(_ || !wipeDone))
  }

  val ren1 = RegInit(false.B)
  val ren2 = if (params.micro.dirReg) RegInit(false.B) else ren1
  ren2 := ren1
  ren1 := ren

  val bypass_valid = params.dirReg(write.valid)
  val bypass = params.dirReg(write.bits, ren1 && write.valid)
  val regout = params.dirReg(cc_dir.read(io.read.bits.set, ren), ren1)
  val tag = params.dirReg(RegEnable(io.read.bits.tag, ren), ren1)
  val set = params.dirReg(RegEnable(io.read.bits.set, ren), ren1)

  // Compute the victim way in case of an evicition
  val victimLFSR = random.LFSR(width = params.cache.ways, params.dirReg(ren))(log2Ceil(params.cache.ways) - 1, 0)//this will give us an lsfr value that will shift no more than the amount of ways we have
  val victimSums = Seq.tabulate(params.cache.ways) { i => ((1 << InclusiveCacheParameters.lfsrBits)*i / params.cache.ways).U }
  val victimLTE  = Cat(victimSums.map { _ <= victimLFSR }.reverse)
  val victimSimp = Cat(0.U(1.W), victimLTE(params.cache.ways-1, 1), 1.U(1.W))
  val victimWayOH = victimSimp(params.cache.ways-1,0) & ~(victimSimp >> 1)
  val victimWay = OHToUInt(victimWayOH)
  assert (!ren2 || victimLTE(0) === 1.U)
  assert (!ren2 || ((victimSimp >> 1) & ~victimSimp) === 0.U) // monotone
  assert (!ren2 || PopCount(victimWayOH) === 1.U)

  //OKAY SO
  //name based grouping??
  //transition to LRU
  
  //==== MOVE CORE GROUPING TO PARAMS OBJECT FOR MY SANITY AND LETTING US OFFLOAD SOME WORK TO OTHER CYCLES ====//
  // // println(s"Params.inner!! = ${params.inner.client}\n")
  // // val isCore = params.inner.client.clients.map(client => if(client.name.contains("Core")) 1 else 0)
  // // println(s"list comprehension?? = ${isCore}\n")//yes this works
  // //next, count the total number of cores??? what to do about non-core entities like monitors, etc, that are present on the TL interface?
  // //I guess this comes down to how we want accelerators to behave..?
  // //num partitions = num cores + 1 for everything else??
  // // convert this to the client bits function...
  // val groupedByCore = params.inner.client.clients.groupBy { client => //group clients by core (and everything else, group by first 2 words of the name LOL)
  //   client.name.split(" ").take(2).mkString(" ") //extract "Core X" from the name
  // }

  // groupedByCore.foreach { case (core, ranges) =>
  //   val rangesList = ranges.map(_.sourceId)//make a list of the ranges per group
  //   println(s"$core -> ${rangesList.mkString(", ")}") //yes this works!
  // }
  // //val req_clientBit = params.clientBit(request.source)//this means we need the source...or even better the client itself!
  //==============================//

  
  val cc_parrp =  DescribedSRAM(//this is our actual sram bank for the partitions.
    name = "cc_parrp",
    desc = "ParRP RAM",
    size = params.cache.sets,//per set
    data = Vec(params.coreIndexMap.size, Vec(params.partitionSize, new ParrpEntry(params)))//per core, per way in partition, store a ParrpEntry which contains way, state, and LRU data
  )

  //we SHOULD be able to piggyback off of the existing wiping logic because it's per set as well.
  val wipeVec = VecInit((0 until params.partitionSize).map {  i => //the is the internal vec of the parrp state 
    var way = Wire(new ParrpEntry(params))//blank bundle
    way.way_index := 0.U 
    way.state := ParrpStates.invalid
    way.lru := i.U
    way //this should init our wipevector to produce a valid LRU state
  })

  dontTouch(wipeVec)

  // read register
  val parrp_table_reg_vec = params.dirReg(cc_parrp.read(io.read.bits.set, ren), ren1) //need reg to store read vals, this should still infer right, it's how the directory does it.
  // I'm pretty sure this is inferring a mask which is lit

  //post-read logic

  // def findCoreIndex(sourceId: UInt): UInt = { //this is a HARDWARE function // => move to params
  //   val core_index = UInt(log2Ceil(params.sourceIdRanges.size).W)
  //   sourceIdRanges.foreach{}
  //   sourceIdRanges.collectFirst { //the ID should only map to one range, but maybe this makes better hardware?? I have no idea.
  //     case (range, coreIndex) if ((range.start.U < sourceId).asBool && (range.end.U > sourceId).asBool) => coreIndex
  //   }
  // }

  val sourceId = RegEnable(io.read.bits.sourceId, ren)
  val not_a_core = params.coreIndexMap.size.U //0 index means that size is out of range
  val core = MuxCase( not_a_core, //default is an out of range core so we can ignore non-core requests.
    params.sourceIdRanges.map { case (range, core) => //get our core index from 
      (sourceId >= range.start.U && sourceId <= range.end.U, core) //mapping to tuples should work fine
    }
  )

  val read_core = params.dirReg(core, ren1)// get the core in the dir in parallel to the read, update cycle after read. This has 1 cycle to convert so should be fine.

  val used_core = Mux(read_core < not_a_core, read_core, 0.U)//if not a core, avoid overflowing range

  val parrp_table_core_vec = Mux((used_core === bypass.core && bypass_valid && bypass.set === set), bypass.parrp_data_vec, parrp_table_reg_vec(used_core)) // make this a mux to allow for bypassing of parrp_table into raw read before processing

  val updated_lru_vec = VecInit(parrp_table_core_vec.map { parrp_entry => //this is the table of just the core, a vec of parrp_entries
    // this table will need a type definition or else I'm worried about bit width sizing.
    val new_entry = Wire(new ParrpEntry(params))
    new_entry.lru := Mux(parrp_entry.state === ParrpStates.invalid, (params.partitionSize - 1).U, //if invalid, set val to max_val
      Mux(parrp_entry.state === ParrpStates.deallocated, parrp_entry.lru, 0.U)) //if not in core, store lru state. if in core, do not evict.
        
    new_entry.way_index := parrp_entry.way_index //don't change way indices
    new_entry.state := parrp_entry.state
      
    new_entry //return new entry
  })//wrap the whole thing in a vec

  val parrp_speculated_victim_index: UInt = updated_lru_vec.zipWithIndex.map{case (data, idx) => 
    (data, idx.U)}.reduce { (a, b) =>
    MuxT(a._1.lru > b._1.lru, a, b) //collapse vector down to way index of maximum adjusted lru value.  
  }._2

  // val phy_vacant_way = MuxCase(params.cache.ways.U,
  //   regout.zipWithIndex.map { case (way, idx) => //use regout to get right width, idk if we actually need it but ~optimization~ fixes it lol
  //     (parrp_table_reg_vec.zipWithIndex.map { case (core_table, core_idx) => //per core
  //         Mux((core_idx.U === bypass.core && bypass_valid && bypass.set === set), bypass.parrp_data_vec, //bypass write_core if same set
  //         core_table).map{ case (entry) => //per entry in core
  //         entry.way_index === idx.U && entry.state =/= ParrpStates.invalid //if the way is in the state table and it's not invalid
  //       }.reduce(_ || _) //if any match in this core
  //     }.reduce(_ || _) === 0.U, idx.U) //if no match for this way, it is free.
  //   } // this is such a nasty mux lmfao
  // )

  // val true_victim = Mux(phy_vacant_way =/= params.cache.ways.U, phy_vacant_way, parrp_table_core_vec(parrp_speculated_victim_index).way_index)// get lowest index unowned way (cheapest), could consider random too.

  //the draft:
  def rotateLeft[T <: Data](vec: Vec[T], shift: UInt): Vec[Bool] = {
    val n = vec.length
    val asUInt = vec.asUInt
    val rotated = (asUInt << shift) | (asUInt >> (n.U - shift))
    VecInit(rotated(n - 1, 0).asBools) //yeah yeah the type juggling is cringe
  }

  def rotateRight[T <: Data](vec: Vec[T], shift: UInt): Vec[Bool] = {
    val n = vec.length
    val asUInt = vec.asUInt
    val rotated = (asUInt >> shift) | (asUInt << (n.U - shift))
    VecInit(rotated(n - 1, 0).asBools)
  }

  val free_ways : Vec[Bool] = VecInit(
    regout.zipWithIndex.map { case (way, idx) => //use regout to get right width, idk if we actually need it but ~optimization~ fixes it lol, operate per way
      (parrp_table_reg_vec.zipWithIndex.map { case (core_table, core_idx) => //per core
          Mux(((core_idx.U === bypass.core) && bypass_valid && (bypass.set === set)), bypass.parrp_data_vec, //bypass write_core if same set
            core_table
          ).map{ case (entry) => //per entry in core
            entry.way_index === idx.U && entry.state =/= ParrpStates.invalid //if the way is in the state table and it's not invalid
        }.reduce(_ || _) //if we match in this core
      }.reduce(_ || _) === 0.U) //if no match in any core, it is free.
    })

  dontTouch(free_ways)
  val phy_vacant_way_random = PriorityEncoderOH(
    rotateLeft(free_ways, victimLFSR) //circular shift victim state for random vacant replacement
  )  
  val phy_vacant_way = Cat(rotateRight(VecInit(phy_vacant_way_random), victimLFSR).reverse)//undo shift to give us our final victim
  dontTouch(phy_vacant_way)

  val true_victim = Mux(phy_vacant_way =/= 0.U, OHToUInt(phy_vacant_way), parrp_table_core_vec(parrp_speculated_victim_index).way_index)// get lowest index unowned way (cheapest), could consider random too.

  //if no unowned, take parrp victim as true victim. should be max parallelism.
  //need to examing more what "unowned" means... this doesn't mean it has no clients, because L2 owns data not in L1 (no clients)

  // val phy_victim = PriorityMux(
  //   regout.zipWithIndex.map{ case (entry, index) => 
  //     (entry.clients === 0.U, index.U)
  //   }
  // )

  //assign to IOs
  io.result.bits.core := read_core
  // io.result.bits.parrp_evicted_way := parrp_speculated_victim //I think we return this as well as true victim to save LRU hw cost.
  io.result.bits.parrp_data_vec := parrp_table_core_vec //ignore non-core reads

  // when (!ren && wen) {//this should then let firrtl infer single ported without issue. //remove this impl after verif.
  //   when(wipeDone){
  //     cc_parrp.write(
  //       wipeSet, 
  //       Vec(params.sourceIdRanges.size, wipeVec)//this should probably also work?
  //       // I don't think I need to specify way for this cause it should default to all of them....RIGHT??
  //     )
  //   }.otherwise{//I need to draw out some state charts LOL
  //     cc_parrp.write(
  //       write.bits.set,//we are writing to the same set that's being written to, yes.
  //       write.bits.parrp_data//we need to update all ways for the core. => throw all of the work off to the MSHR to update the values in some other cycles!
  //       UIntToOH(write.bits.core, params.sourceIdRanges.size).asBools.map(_ || !wipeDone))//do bitmasking for target way only (but we will need to change this)
  //   }
  // }.otherwise{}//default? do we need this to get it to work? who knows!
  // //

  //write to memory when written
  when (!ren && wen && ((write.bits.core < not_a_core) || !wipeDone)) { //we will run freely over non-core entities, but need to wipe regardless of value in write queue.
    cc_parrp.write(
      Mux(wipeDone, write.bits.set, wipeSet), //muxing to do our wiping
      VecInit.fill(params.coreIndexMap.size) { Mux(wipeDone, write.bits.parrp_data_vec, wipeVec) },
      // VecInit.fill(params.coreIndexMap.size) { Mux(wipeDone, write.bits.parrp_data_vec, VecInit.fill(params.partitionSize){0.U.asTypeOf(new ParrpEntry(params))})}, //maybe this will give me a real wipe vector??
      UIntToOH(write.bits.core, params.coreIndexMap.size).asBools.map(_ || !wipeDone)) //bitmask out for core, or write to all cores during wipe
  }


  val setQuash = bypass_valid && bypass.set === set
  val tagMatch = bypass.data.tag === tag
  val wayMatch = bypass.way === true_victim

  val ways = regout//.map(d => d.asTypeOf(new DirectoryEntry(params)))
  val hits = Cat(ways.zipWithIndex.map { case (w, i) =>
    w.tag === tag && w.state =/= INVALID && (!setQuash || i.U =/= bypass.way)
  }.reverse)
  val hit = hits.orR

  io.result.valid := ren2//there is NO reg between read and output, would probably need one if LRU logic depends on hit... maybe always compute replacement?
  io.result.bits.viewAsSupertype(chiselTypeOf(bypass.data)) := Mux(hit, Mux1H(hits, ways), Mux(setQuash && (tagMatch || wayMatch), bypass.data, Mux1H(UIntToOH(true_victim), ways)))
  io.result.bits.hit := hit || (setQuash && tagMatch && bypass.data.state =/= INVALID)
  val way_final = Mux(hit, OHToUInt(hits), Mux(setQuash && tagMatch, bypass.way, true_victim))
  dontTouch(way_final)

  //consider moving this, so we can get its value too, consider 1st cycle. consult profs for IO/logic tradeoff...
  val parrp_hit: Bool = (parrp_table_core_vec.map { case entry => 
    (entry.way_index === way_final) && (entry.state =/= ParrpStates.invalid) && (!setQuash || entry.way_index =/= bypass.way)
  }.reduce(_ || _)) //is our target way in our parrp partition? indep of phy cache.
  //this is just wrong
  io.result.bits.parrp_hit := parrp_hit


  val parrp_index = Mux(parrp_hit, OHToUInt(Cat(parrp_table_core_vec.map{ entry =>
    (entry.way_index === way_final) && (entry.state =/= INVALID) && (!setQuash || entry.way_index =/= bypass.way)
  }.reverse)), parrp_speculated_victim_index) //cursed tagmatch to get the way to pass to the mshr
  io.result.bits.way := way_final
  io.result.bits.parrp_way := parrp_index //if hit, index of hit. if miss, index of evicted parrp line


  params.ccover(ren2 && setQuash && tagMatch, "DIRECTORY_HIT_BYPASS", "Bypassing write to a directory hit")
  params.ccover(ren2 && setQuash && !tagMatch && wayMatch, "DIRECTORY_EVICT_BYPASS", "Bypassing a write to a directory eviction")

  def json: String = s"""{"clients":${params.clientBits},"mem":"${cc_dir.pathName}","clean":"${wipeDone.pathName}"}"""

  // clock cycle counter
    val clk_cycle = RegInit(0.U(32.W))
    clk_cycle := clk_cycle + 1.U
  when (ren || wen) {
    printf(cf"@ clk_cycle ${clk_cycle}: Directory read: ${ren} set: 0x${io.read.bits.set}%x tag: 0x${io.read.bits.tag}%x /write: ${wen} set: 0x${write.bits.set}%x tag: 0x${write.bits.data.tag}%x state: 0x${write.bits.data.state}%x clients: 0x${write.bits.data.clients}\n")
  }
  when (io.write.valid) {
    printf(cf"@ clk_cycle ${clk_cycle}: Queueing write! set: 0x${io.write.bits.set}%x tag: 0x${io.write.bits.data.tag}%x state: 0x${io.write.bits.data.state}%x clients: 0x${write.bits.data.clients}\n")
  }
  when (ren1){
    printf(cf"@ clk_cycle ${clk_cycle}: ParRP read_core: ${read_core}, speculated victim: ${parrp_table_core_vec(parrp_speculated_victim_index)}, true victim: ${true_victim}, table: ${parrp_table_core_vec}\n")
  }
  when (wen){
    printf(cf"@ clk_cycle ${clk_cycle}: ParRP write_core: ${write.bits.core}, write_data: ${write.bits.parrp_data_vec}\n")
  }
  when (ren2){
    printf(cf"@ clk_cycle ${clk_cycle}: Directory read data: ${io.result.bits}\n")
  }
  when (ren){
    printf(cf"@ clk_cycle ${clk_cycle}: sourceID: ${io.read.bits.sourceId}\n")
  }
  println(s"partitionSize_dir: ${params.partitionSize}\n")

}
