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
import chisel3.internal.sourceinfo.SourceInfo
import freechips.rocketchip.tilelink._
import TLPermissions._
import TLMessages._
import MetaData._
import chisel3.PrintableHelper
import chisel3.experimental.dataview.BundleUpcastable

class ScheduleRequest(params: InclusiveCacheParameters) extends InclusiveCacheBundle(params)
{
  val a = Valid(new SourceARequest(params))
  val b = Valid(new SourceBRequest(params))
  val c = Valid(new SourceCRequest(params))
  val d = Valid(new SourceDRequest(params))
  val e = Valid(new SourceERequest(params))
  val x = Valid(new SourceXRequest(params))
  val dir = Valid(new DirectoryWrite(params))
  val reload = Bool() // get next request via allocate (if any)
}

class MSHRStatus(params: InclusiveCacheParameters) extends InclusiveCacheBundle(params)
{
  val set = UInt(params.setBits.W)
  val tag = UInt(params.tagBits.W)
  val way = UInt(params.wayBits.W)
  val blockB = Bool()
  val nestB  = Bool()
  val blockC = Bool()
  val nestC  = Bool()
}

class NestedWriteback(params: InclusiveCacheParameters) extends InclusiveCacheBundle(params)
{
  val set = UInt(params.setBits.W)
  val tag = UInt(params.tagBits.W)
  val b_toN       = Bool() // nested Probes may unhit us
  val b_toB       = Bool() // nested Probes may demote us
  val b_clr_dirty = Bool() // nested Probes clear dirty
  val c_set_dirty = Bool() // nested Releases MAY set dirty
}

sealed trait CacheState
{
  val code = CacheState.index.U
  CacheState.index = CacheState.index + 1
}

object CacheState
{
  var index = 0
}

case object S_INVALID  extends CacheState
case object S_BRANCH   extends CacheState
case object S_BRANCH_C extends CacheState
case object S_TIP      extends CacheState
case object S_TIP_C    extends CacheState
case object S_TIP_CD   extends CacheState
case object S_TIP_D    extends CacheState
case object S_TRUNK_C  extends CacheState
case object S_TRUNK_CD extends CacheState

class MSHR(params: InclusiveCacheParameters) extends Module
{
  val io = IO(new Bundle {
    val allocate  = Flipped(Valid(new AllocateRequest(params))) // refills MSHR for next cycle
    val directory = Flipped(Valid(new DirectoryResult(params))) // triggers schedule setup
    val status    = Valid(new MSHRStatus(params))
    val schedule  = Decoupled(new ScheduleRequest(params))
    val sinkc     = Flipped(Valid(new SinkCResponse(params)))
    val sinkd     = Flipped(Valid(new SinkDResponse(params)))
    val sinke     = Flipped(Valid(new SinkEResponse(params)))
    val nestedwb  = Flipped(new NestedWriteback(params))
    val storeack  = Flipped(Bool())
    val age   = UInt(params.ageBits.W)
  })

  val request_valid = RegInit(false.B)
  val request = Reg(new FullRequest(params))
  val meta_valid = RegInit(false.B)
  val meta = Reg(new DirectoryResult(params))

  // Define which states are valid
  when (meta_valid) {
    when (meta.state === INVALID) {
      assert (!meta.clients.orR)
      assert (!meta.dirty)
    }
    when (meta.state === BRANCH) {
      assert (!meta.dirty)
    }
    when (meta.state === TRUNK) {
      assert (meta.clients.orR)
      assert ((meta.clients & (meta.clients - 1.U)) === 0.U) // at most one
    }
    when (meta.state === TIP) {
      // noop
    }
  }

  // Completed transitions (s_ = scheduled), (w_ = waiting)
  val s_rprobe         = RegInit(true.B) // B
  val w_rprobeackfirst = RegInit(true.B)
  val w_rprobeacklast  = RegInit(true.B)
  val s_release        = RegInit(true.B) // CW w_rprobeackfirst
  val w_releaseack     = RegInit(true.B)
  val s_pprobe         = RegInit(true.B) // B
  val s_acquire        = RegInit(true.B) // A  s_release, s_pprobe [1]
  val s_flush          = RegInit(true.B) // X  w_releaseack
  val w_grantfirst     = RegInit(true.B)
  val w_grantlast      = RegInit(true.B)
  val w_grant          = RegInit(true.B) // first | last depending on wormhole
  val w_pprobeackfirst = RegInit(true.B)
  val w_pprobeacklast  = RegInit(true.B)
  val w_pprobeack      = RegInit(true.B) // first | last depending on wormhole
  val s_probeack       = RegInit(true.B) // C  w_pprobeackfirst (mutually exclusive with next two s_*)
  val s_grantack       = RegInit(true.B) // E  w_grantfirst ... CAN require both outE&inD to service outD
  val s_execute        = RegInit(true.B) // D  w_pprobeack, w_grant
  val w_grantack       = RegInit(true.B)
  val s_writeback      = RegInit(true.B) // W  w_*
  val s_speculativerel = RegInit(true.B) // when we get a dirty release that's not going right back to another L1, wb to main mem for better bound on other reqs
  val w_store          = RegInit(true.B) //wait for BS to complete

  // [1]: We cannot issue outer Acquire while holding blockB (=> outA can stall)
  // However, inB and outC are higher priority than outB, so s_release and s_pprobe
  // may be safely issued while blockB. Thus we must NOT try to schedule the
  // potentially stuck s_acquire with either of them (scheduler is all or none).

  // Meta-data that we discover underway
  val sink = Reg(UInt(params.outer.bundle.sinkBits.W))
  val gotT = Reg(Bool())
  val bad_grant = Reg(Bool())
  val probes_done = Reg(UInt(params.clientBits.W))
  val probes_toN = Reg(UInt(params.clientBits.W))
  val probes_noT = Reg(Bool())

  // When a nested transaction completes, update our meta data
  when (meta_valid && meta.state =/= INVALID &&
        io.nestedwb.set === request.set && io.nestedwb.tag === meta.tag) {
    when (io.nestedwb.b_clr_dirty) { meta.dirty := false.B }
    when (io.nestedwb.c_set_dirty) { meta.dirty := true.B }
    when (io.nestedwb.b_toB) { meta.state := BRANCH }
    when (io.nestedwb.b_toN) { meta.hit := false.B }
  }

  // Scheduler status
  io.status.valid := request_valid
  io.status.bits.set    := request.set
  io.status.bits.tag    := request.tag
  io.status.bits.way    := meta.way
  io.status.bits.blockB := !meta_valid || ((!w_releaseack || !w_rprobeacklast || !w_pprobeacklast) && !w_grantfirst)
  io.status.bits.nestB  := meta_valid && w_releaseack && w_rprobeacklast && w_pprobeacklast && !w_grantfirst
  // The above rules ensure we will block and not nest an outer probe while still doing our
  // own inner probes. Thus every probe wakes exactly one MSHR.
  io.status.bits.blockC := !meta_valid
  // io.status.bits.nestC  := meta_valid && (!w_rprobeackfirst || !w_pprobeackfirst || !w_grantfirst)
  io.status.bits.nestC  := meta_valid && (!w_rprobeacklast || !w_pprobeacklast) //if there are ANY outstanding probes, we must assume that the release needs to be handled out of band.
  // The w_grantfirst in nestC is necessary to deal with:
  //   acquire waiting for grant, inner release gets queued, outer probe -> inner probe -> deadlock
  // ... this is possible because the release+probe can be for same set, but different tag
  // ...... this all depends on cache not being last level - which it is in this design. So, disregard.
  io.age := request.age //relay req age to outside for arbitration

  // We can only demand: block, nest, or queue
  assert (!io.status.bits.nestB || !io.status.bits.blockB)
  assert (!io.status.bits.nestC || !io.status.bits.blockC)

  // Scheduler requests
  val no_wait = w_rprobeacklast && w_releaseack && w_grantlast && w_pprobeacklast && (w_grantack) && s_flush && s_execute //allow bypassing of waiting on grantack to make requests 1 cycle shorter, nvm this crashes the peekpoke tester LMAO
  io.schedule.bits.a.valid := !s_acquire && s_release && s_pprobe
  io.schedule.bits.b.valid := !s_rprobe || !s_pprobe
  io.schedule.bits.c.valid := (!s_release && w_rprobeackfirst) || (!s_probeack && w_pprobeackfirst) || (!s_speculativerel && w_store)
  io.schedule.bits.d.valid := !s_execute && w_pprobeack && w_grant
  io.schedule.bits.e.valid := !s_grantack && w_grantfirst && s_execute //depend on srcd issue to serialize :)
  io.schedule.bits.x.valid := !s_flush && w_releaseack
  // io.schedule.bits.dir.valid := (!s_release && w_rprobeackfirst) || (!s_writeback && no_wait) // we must complete lru modifications before these conditions are met...
  io.schedule.bits.dir.valid := (!s_writeback && no_wait) // change to serialize directory access for flushes.
  io.schedule.bits.reload := no_wait
  io.schedule.valid := io.schedule.bits.a.valid || io.schedule.bits.b.valid || io.schedule.bits.c.valid ||
                       io.schedule.bits.d.valid || io.schedule.bits.e.valid || io.schedule.bits.x.valid ||
                       io.schedule.bits.dir.valid

  // Schedule completions
  when (io.schedule.ready) {
                                    s_rprobe     := true.B
    when (w_rprobeackfirst)       { s_release    := true.B }
                                    s_pprobe     := true.B
    when (s_release && s_pprobe)  { s_acquire    := true.B }
    when (w_releaseack)           { s_flush      := true.B }
    when (w_pprobeackfirst)       { s_probeack   := true.B }
    when (w_grantfirst && s_execute){ s_grantack   := true.B }
    when (w_pprobeack && w_grant) { s_execute    := true.B } //sorry to your nice formatting :(
    when (w_store)                { s_speculativerel := true.B}
    when (no_wait)                { s_writeback  := true.B }
    // Await the next operation
    when (no_wait) {
      request_valid := false.B
      meta_valid := false.B
    }
  }

  // Resulting meta-data
  val final_meta_writeback = WireInit(meta)

  val req_clientBit = params.clientBit(request.source)
  val req_needT = needT(request.opcode, request.param)
  val req_acquire = request.opcode === AcquireBlock || request.opcode === AcquirePerm
  val meta_no_clients = !meta.clients.orR
  val req_promoteT = req_acquire && Mux(meta.hit, meta_no_clients && meta.state === TIP, gotT)

  when (request.prio(2) && (!params.firstLevel).B) { // always a hit
    final_meta_writeback.dirty   := (meta.dirty && (request.opcode =/= 7.U) && !(request.opcode === 6.U && ((meta.modified_cores ^ req_clientBit) === 0.U))) || (request.opcode(0) && !request.opcode(1)) //we will be way cleaner now right? only ProbeAckData (0b101) sets us dirty, not ReleaseData (0b111)
    final_meta_writeback.state   := Mux(((meta.clients & ~Mux(isToN(request.param), req_clientBit, 0.U)) === 0.U) && meta.state === TRUNK, TIP, meta.state) //only when final client transitions to N can we reclaim tip safely.
    final_meta_writeback.clients := meta.clients & ~Mux(isToN(request.param), req_clientBit, 0.U)
    final_meta_writeback.hit     := true.B // chained requests are hits
    final_meta_writeback.modified_cores := meta.modified_cores & ~req_clientBit //remove set bit from modified cores
    // printf(cf"using prio2 client update! fmw.clients = ${final_meta_writeback.clients}, meta.clients = ${meta.clients}, isToN = ${isToN(request.param)}, param = ${request.param}, req_clientBit = ${req_clientBit}\n")
  } .elsewhen (request.control && params.control.B) { // request.prio(0)
    when (meta.hit) {
      final_meta_writeback.dirty   := false.B
      final_meta_writeback.state   := INVALID
      final_meta_writeback.clients := meta.clients & ~probes_toN
    }
    final_meta_writeback.hit := false.B
    // printf("using prio0 client update! \n")
  } .otherwise {
    final_meta_writeback.dirty := (meta.hit && meta.dirty) || !request.opcode(2)
    final_meta_writeback.state := Mux(req_needT,
                                    Mux(req_acquire, TRUNK, TIP),
                                    Mux(!meta.hit, Mux(gotT, Mux(req_acquire, TRUNK, TIP), BRANCH),
                                      MuxLookup(meta.state, 0.U(2.W), Seq(
                                        INVALID -> BRANCH,
                                        BRANCH  -> BRANCH,
                                        TRUNK   -> TIP,
                                        TIP     -> Mux(meta_no_clients && req_acquire, TRUNK, TIP)))))
    final_meta_writeback.clients := Mux(meta.hit, meta.clients & ~probes_toN, 0.U) |
                                    Mux(req_acquire, req_clientBit, 0.U)
    final_meta_writeback.tag := request.tag
    final_meta_writeback.hit := true.B
  }

  when (bad_grant) {
    when (meta.hit) {
      // upgrade failed (B -> T)
      assert (!meta_valid || meta.state === BRANCH)
      final_meta_writeback.hit     := true.B
      final_meta_writeback.dirty   := false.B
      final_meta_writeback.state   := BRANCH
      final_meta_writeback.clients := meta.clients & ~probes_toN
    } .otherwise {
      // failed N -> (T or B)
      final_meta_writeback.hit     := false.B
      final_meta_writeback.dirty   := false.B
      final_meta_writeback.state   := INVALID
      final_meta_writeback.clients := 0.U
    }
  }


   //=============== PARRP DATA LRU CALCULATION ====================//
  //notes:
    // how do we pipeline this, where is the critical path?
    // isolate lru state of hit way in clk1 
    // update lrus in clk2?
  //cases and stuff:
    // if we demote something due to an explicit release -> change state to not in core, | if it is a releaseData or was previously dirty, update LRU. Else, leave info unchanged.
  val new_parrp_data = Wire(Vec(params.partitionSize, new ParrpEntry(params)))
  val we_released_something = Reg(Bool()) //surely a flag is disgusting enough to work LOL
  val noncoherent_request = request.prio(0) && (request.opcode === 0.U || request.opcode === 1.U || request.opcode === 4.U || request.opcode === 5.U)//a put or a get (or an intent) on A channel are non-coherent requests
  we_released_something := false.B
  new_parrp_data := meta.parrp_data_vec
  when(meta_valid && request.prio(2) && (request.opcode === 6.U || request.opcode === 7.U)) {// release
    //val new_parrp_data = meta.parrp_data_vec
    new_parrp_data(meta.parrp_way).state := ParrpStates.deallocated //deallocate state, leave the rest of the vec alone!
    when((request.param < 3.U) || (request.param === 5.U)){final_meta_writeback.in_core := meta.in_core & ~UIntToOH(meta.core)} //remove core from phy owners vec if it is pruning (has no reason to prune to B (param = 0))
    when((!s_speculativerel && w_store) || (!s_release && w_rprobeackfirst)){ //extra when clause to print less >_>
      printf(cf"Releasing! in_core = ${final_meta_writeback.in_core}, updated_entry = ${new_parrp_data(meta.parrp_way)}\n") 
    }

    when((request.opcode === 7.U && meta.state =/= TIP) || (request.opcode === 6.U && meta.state === INVALID && (meta.modified_cores & req_clientBit).orR)) { //this is the conditions under which we update LRU (as they represent a store)
      meta.parrp_data_vec.zipWithIndex.map { case(d, i) =>
        new_parrp_data(i).lru := Mux(d.lru < meta.parrp_data_vec(meta.parrp_way).lru, d.lru + 1.U, d.lru) //parrp way contains stored way
      }

      new_parrp_data(meta.parrp_way).lru := 0.U //set stored way to MRU
      // printf(cf"Updated LRU due to store! New LRU = ${new_parrp_data}\n")
    }
    // printf(cf"for my sanity: fmw.state = ${final_meta_writeback.state}, m.state = ${meta.state} \n")
  }.elsewhen(meta_valid && !s_release){ //if our request prompts a cache release, we must mark parrp_entry as invalid so it is properly populated with state info.
    new_parrp_data(meta.parrp_way).state := ParrpStates.invalid //set state invalid (may be unneeded idk, depends what else is going on)
    we_released_something := true.B   
  }.elsewhen(meta_valid && !(meta.parrp_hit && request.prio(0) && (meta.parrp_data_vec(meta.parrp_way).state === ParrpStates.allocated) && (request.opcode(2) && request.opcode(1)))){ //for all other valid reqs, only change L2 state for acquires to lines that are not "in core" as these are supposed to be L1 hits instead!
    // if we hit something in not in core section of partition -> set state to in core, update LRU for all ways.
    meta.parrp_data_vec.zipWithIndex.map { case(d, i) => //collapse lru update map function to save hw later...
      new_parrp_data(i).lru := Mux(d.lru < meta.parrp_data_vec(meta.parrp_way).lru, d.lru + 1.U, d.lru) //parrp way contains either hit or evicted, depending on miss status - but what we do doesn't change!
    }
      // if we evict something from vec to take in a new value -> set state to in core, update LRU for all ways, update phy way index for replaced.
    // val new_parrp_data_vec_replace = meta.parrp_data_vec.map { _ => //replace subcase
    //   val new_parrp_entry = Wire(new ParrpEntry(params)) //blank new vec
    //   parrp_entry := _ //some of this should get overwritten and be inferred, right?
    //   parrp_entry.lru := Mux(_.lru < meta.parrp_data_vec(meta.way).lru, _.lru + 1.U, _.lru) // increment LRU by 1 iff it is lower in LRU order than replaced way
    //   parrp_entry //return updated parrp entry
    // }
    new_parrp_data(meta.parrp_way).lru := 0.U //set updated way lru to 0 as it is MRU
    //replace updated way with new data beyond lru if a miss or overwriting a replaced way
    when(!meta.parrp_hit || (we_released_something && request.prio(0)) || (meta.parrp_hit && request.prio(0))){ //when we miss in partition, update partition. Don't care about phy eviction/hit status. Don't update on flush commands (prio(1)). Ensure that state gets re-set in the event of a hit in L2 so it's allocated in core.
      we_released_something := we_released_something //latch until meta is no longer valid
      new_parrp_data(meta.parrp_way).state := Mux(noncoherent_request, ParrpStates.deallocated, ParrpStates.allocated) //set to deallocated for noncoherent, allocated state for coherent in partition
      new_parrp_data(meta.parrp_way).way_index := meta.way //set index to the phy way we are using (when replaced and not invalid, this way is released from L2 first, but no wb)
    }
    final_meta_writeback.in_core := Mux(noncoherent_request, meta.in_core, meta.in_core | UIntToOH(meta.core)) //add core to phy owners vec if coherent request
  
  }
  final_meta_writeback.parrp_data_vec := new_parrp_data //so this will properly replay in mshr
  //printf(cf"are you crazy? fmw_in_core: ${final_meta_writeback.in_core} meta_core: ${meta.core} \n")

  //=============== END PARRP DATA LRU CALCULATION ================//

  //=============== PARRP DATA ASSIGNMMENT ========================//
  io.schedule.bits.dir.bits.parrp_data_vec := new_parrp_data //this will change on an aborted transition due to reasons xyz, or nested requests
  io.schedule.bits.dir.bits.core := meta.core //this might change but I don't think it does
  //=============== END PARRP DATA ASSIGNMMENT ====================//


  val invalid = Wire(new DirectoryEntry(params))
  invalid.dirty   := false.B
  invalid.state   := INVALID
  invalid.clients := 0.U
  invalid.tag     := 0.U
  invalid.in_core := 0.U
  invalid.modified_cores := 0.U

  // Just because a client says BtoT, by the time we process the request he may be N.
  // Therefore, we must consult our own meta-data state to confirm he owns the line still.
  val honour_BtoT = meta.hit && (meta.clients & req_clientBit).orR

  // The client asking us to act is proof they don't have permissions.
  val excluded_client = Mux(meta.hit && request.prio(0) && skipProbeN(request.opcode, params.cache.hintsSkipProbe), req_clientBit, 0.U)
  io.schedule.bits.a.bits.tag     := request.tag
  io.schedule.bits.a.bits.set     := request.set
  io.schedule.bits.a.bits.param   := Mux(req_needT, Mux(meta.hit, BtoT, NtoT), NtoB)
  io.schedule.bits.a.bits.block   := request.size =/= log2Ceil(params.cache.blockBytes).U ||
                                     !(request.opcode === PutFullData || request.opcode === AcquirePerm)
  io.schedule.bits.a.bits.source  := 0.U
  io.schedule.bits.b.bits.param   := Mux(!s_rprobe, toN, Mux(request.prio(1), request.param, Mux(req_needT, toN, toB)))
  io.schedule.bits.b.bits.tag     := Mux(!s_rprobe, meta.tag, request.tag)
  io.schedule.bits.b.bits.set     := request.set
  io.schedule.bits.b.bits.clients := meta.clients & ~excluded_client
  io.schedule.bits.c.bits.opcode  := Mux(!s_speculativerel, ReleaseData, Mux(meta.dirty, ReleaseData, Release)) //speculative release is always ReleaseData as it is always dirty -> tbh we should be eliminating the 2nd ReleaseData entirely but hey, being confident is hard
  io.schedule.bits.c.bits.param   := Mux(!s_speculativerel, TtoT ,Mux(meta.state === BRANCH, BtoN, TtoN)) //speculative release is a report
  io.schedule.bits.c.bits.source  := 0.U
  io.schedule.bits.c.bits.tag     := meta.tag
  io.schedule.bits.c.bits.set     := request.set
  io.schedule.bits.c.bits.way     := meta.way
  io.schedule.bits.c.bits.dirty   := Mux(!s_speculativerel, true.B, meta.dirty) //speculative release is always dirty
  io.schedule.bits.d.bits.viewAsSupertype(chiselTypeOf(request)) := request
  io.schedule.bits.d.bits.param   := Mux(!req_acquire, request.param,
                                       MuxLookup(request.param, request.param, Seq(
                                         NtoB -> Mux(req_promoteT, NtoT, NtoB),
                                         BtoT -> Mux(honour_BtoT,  BtoT, NtoT),
                                         NtoT -> NtoT)))
  io.schedule.bits.d.bits.sink    := 0.U
  io.schedule.bits.d.bits.way     := meta.way
  io.schedule.bits.d.bits.bad     := bad_grant
  io.schedule.bits.e.bits.sink    := sink
  io.schedule.bits.x.bits.fail    := false.B
  io.schedule.bits.dir.bits.set   := request.set
  io.schedule.bits.dir.bits.way   := meta.way
  io.schedule.bits.dir.bits.data  := Mux(!s_release, invalid, WireInit(new DirectoryEntry(params), init = final_meta_writeback))

  // Coverage of state transitions
  def cacheState(entry: DirectoryEntry, hit: Bool) = {
    val out = WireDefault(0.U)
    val c = entry.clients.orR
    val d = entry.dirty
    switch (entry.state) {
      is (BRANCH)  { out := Mux(c, S_BRANCH_C.code, S_BRANCH.code) }
      is (TRUNK)   { out := Mux(d, S_TRUNK_CD.code, S_TRUNK_C.code) }
      is (TIP)     { out := Mux(c, Mux(d, S_TIP_CD.code, S_TIP_C.code), Mux(d, S_TIP_D.code, S_TIP.code)) }
      is (INVALID) { out := S_INVALID.code }
    }
    when (!hit) { out := S_INVALID.code }
    out
  }

  val p = !params.lastLevel  // can be probed
  val c = !params.firstLevel // can be acquired
  val m = params.inner.client.clients.exists(!_.supports.probe)   // can be written (or read)
  val r = params.outer.manager.managers.exists(!_.alwaysGrantsT) // read-only devices exist
  val f = params.control     // flush control register exists
  val cfg = (p, c, m, r, f)
  val b = r || p // can reach branch state (via probe downgrade or read-only device)

  // The cache must be used for something or we would not be here
  require(c || m)

  val evict = cacheState(meta, !meta.hit)
  val before = cacheState(meta, meta.hit)
  val after  = cacheState(final_meta_writeback, true.B)

  def eviction(from: CacheState, cover: Boolean)(implicit sourceInfo: SourceInfo): Unit = {
    if (cover) {
      params.ccover(evict === from.code, s"MSHR_${from}_EVICT", s"State transition from ${from} to evicted ${cfg}")
    } else {
      assert(!(evict === from.code), cf"State transition from ${from} to evicted should be impossible ${cfg}")
    }
    if (cover && f) {
      params.ccover(before === from.code, s"MSHR_${from}_FLUSH", s"State transition from ${from} to flushed ${cfg}")
    } else {
      assert(!(before === from.code), cf"State transition from ${from} to flushed should be impossible ${cfg}")
    }
  }

  def transition(from: CacheState, to: CacheState, cover: Boolean)(implicit sourceInfo: SourceInfo): Unit = {
    if (cover) {
      params.ccover(before === from.code && after === to.code, s"MSHR_${from}_${to}", s"State transition from ${from} to ${to} ${cfg}")
    } else {
      assert(!(before === from.code && after === to.code), cf"State transition from ${from} to ${to} should be impossible ${cfg}")
    }
  }

  //speculative releases don't change permission states so they don't need an assertion array.

  when ((!s_release && w_rprobeackfirst) && io.schedule.ready) { //assert back-inv freedom.
    eviction(S_BRANCH,    b)      // MMIO read to read-only device
    eviction(S_BRANCH_C,  b && c) // you need children to become C
    eviction(S_TIP,       true)   // MMIO read || clean release can lead to this state ||// ONLY EVICT THINGS YOU OWN, PARRP
    eviction(S_TIP_C,     false)      // needs two clients || client + mmio || downgrading client
    eviction(S_TIP_CD,    false)      // needs two clients || client + mmio || downgrading client
    eviction(S_TIP_D,     false)   // MMIO write || dirty release lead here
    eviction(S_TRUNK_C,   false)      // acquire for write
    eviction(S_TRUNK_CD,  false)      // dirty release then reacquire
  }

  when ((!s_writeback && no_wait) && io.schedule.ready) {
    transition(S_INVALID,  S_BRANCH,   b && m) // only MMIO can bring us to BRANCH state
    transition(S_INVALID,  S_BRANCH_C, b && c) // C state is only possible if there are inner caches
    transition(S_INVALID,  S_TIP,      m)      // MMIO read
    transition(S_INVALID,  S_TIP_C,    false)  // we would go S_TRUNK_C instead
    transition(S_INVALID,  S_TIP_CD,   false)  // acquire does not cause dirty immediately
    transition(S_INVALID,  S_TIP_D,    m)      // MMIO write
    transition(S_INVALID,  S_TRUNK_C,  c)      // acquire
    transition(S_INVALID,  S_TRUNK_CD, false)  // acquire does not cause dirty immediately

    transition(S_BRANCH,   S_INVALID,  b && p) // probe can do this (flushes run as evictions)
    transition(S_BRANCH,   S_BRANCH_C, b && c) // acquire
    transition(S_BRANCH,   S_TIP,      b && m) // prefetch write
    transition(S_BRANCH,   S_TIP_C,    false)  // we would go S_TRUNK_C instead
    transition(S_BRANCH,   S_TIP_CD,   false)  // acquire does not cause dirty immediately
    transition(S_BRANCH,   S_TIP_D,    b && m) // MMIO write
    transition(S_BRANCH,   S_TRUNK_C,  b && c) // acquire
    transition(S_BRANCH,   S_TRUNK_CD, false)  // acquire does not cause dirty immediately

    transition(S_BRANCH_C, S_INVALID,  b && c && p)
    transition(S_BRANCH_C, S_BRANCH,   b && c)      // clean release (optional)
    transition(S_BRANCH_C, S_TIP,      b && c && m) // prefetch write
    transition(S_BRANCH_C, S_TIP_C,    false)       // we would go S_TRUNK_C instead
    transition(S_BRANCH_C, S_TIP_D,    b && c && m) // MMIO write
    transition(S_BRANCH_C, S_TIP_CD,   false)       // going dirty means we must shoot down clients
    transition(S_BRANCH_C, S_TRUNK_C,  b && c)      // acquire
    transition(S_BRANCH_C, S_TRUNK_CD, false)       // acquire does not cause dirty immediately

    transition(S_TIP,      S_INVALID,  p)
    transition(S_TIP,      S_BRANCH,   p)      // losing TIP only possible via probe
    transition(S_TIP,      S_BRANCH_C, false)  // we would go S_TRUNK_C instead
    transition(S_TIP,      S_TIP_C,    false)  // we would go S_TRUNK_C instead
    transition(S_TIP,      S_TIP_D,    m)      // direct dirty only via MMIO write
    transition(S_TIP,      S_TIP_CD,   false)  // acquire does not make us dirty immediately
    transition(S_TIP,      S_TRUNK_C,  c)      // acquire
    transition(S_TIP,      S_TRUNK_CD, false)  // acquire does not make us dirty immediately

    transition(S_TIP_C,    S_INVALID,  c && p)
    transition(S_TIP_C,    S_BRANCH,   c && p) // losing TIP only possible via probe
    transition(S_TIP_C,    S_BRANCH_C, c && p) // losing TIP only possible via probe
    transition(S_TIP_C,    S_TIP,      c)      // probed while MMIO read || clean release (optional)
    transition(S_TIP_C,    S_TIP_D,    c && m) // direct dirty only via MMIO write
    transition(S_TIP_C,    S_TIP_CD,   false)  // going dirty means we must shoot down clients
    transition(S_TIP_C,    S_TRUNK_C,  c)      // acquire
    transition(S_TIP_C,    S_TRUNK_CD, false)  // acquire does not make us immediately dirty

    transition(S_TIP_D,    S_INVALID,  p)
    transition(S_TIP_D,    S_BRANCH,   p)      // losing D is only possible via probe
    transition(S_TIP_D,    S_BRANCH_C, p && c) // probed while acquire shared
    transition(S_TIP_D,    S_TIP,      p)      // probed while MMIO read || outer probe.toT (optional)
    transition(S_TIP_D,    S_TIP_C,    false)  // we would go S_TRUNK_C instead
    transition(S_TIP_D,    S_TIP_CD,   false)  // we would go S_TRUNK_CD instead
    transition(S_TIP_D,    S_TRUNK_C,  p && c) // probed while acquired
    transition(S_TIP_D,    S_TRUNK_CD, c)      // acquire

    transition(S_TIP_CD,   S_INVALID,  c && p)
    transition(S_TIP_CD,   S_BRANCH,   c && p) // losing D is only possible via probe
    transition(S_TIP_CD,   S_BRANCH_C, c && p) // losing D is only possible via probe
    transition(S_TIP_CD,   S_TIP,      c) // last modifier releases while no read-only copies exist, we perform writethrough.
    transition(S_TIP_CD,   S_TIP_C,    c)  // last modifier releases while read-only copies exist, we perform writethrough but the read-only copies persist.
    transition(S_TIP_CD,   S_TIP_D,    false)      // if we have no more clients, a writethrough should've happened, transitiontiung to S_TIP
    transition(S_TIP_CD,   S_TRUNK_C,  c && p) // probed while acquire
    transition(S_TIP_CD,   S_TRUNK_CD, c)      // acquire

    transition(S_TRUNK_C,  S_INVALID,  c && p)
    transition(S_TRUNK_C,  S_BRANCH,   c && p) // losing TIP only possible via probe
    transition(S_TRUNK_C,  S_BRANCH_C, c && p) // losing TIP only possible via probe
    transition(S_TRUNK_C,  S_TIP,      c)      // MMIO read || clean release (optional)
    transition(S_TRUNK_C,  S_TIP_C,    c)      // bounce shared
    transition(S_TRUNK_C,  S_TIP_D,    c)      // dirty release
    transition(S_TRUNK_C,  S_TIP_CD,   c)      // dirty bounce shared
    transition(S_TRUNK_C,  S_TRUNK_CD, c)      // dirty bounce

    transition(S_TRUNK_CD, S_INVALID,  c && p)
    transition(S_TRUNK_CD, S_BRANCH,   c && p) // losing D only possible via probe
    transition(S_TRUNK_CD, S_BRANCH_C, c && p) // losing D only possible via probe
    transition(S_TRUNK_CD, S_TIP,      c) // probed while MMIO read || outer probe.toT (optional) || released by all holders & WT
    transition(S_TRUNK_CD, S_TIP_C,    false)  // we would go S_TRUNK_C instead
    transition(S_TRUNK_CD, S_TIP_D,    c)      // dirty release -> only WT on last sharer release
    transition(S_TRUNK_CD, S_TIP_CD,   c)      // bounce shared
    transition(S_TRUNK_CD, S_TRUNK_C,  c && p) // probed while acquire
  }

  // Handle response messages
  val probe_bit = params.clientBit(io.sinkc.bits.source)
  val last_probe = (probes_done | probe_bit) === (meta.clients & ~excluded_client)
  val probe_toN = isToN(io.sinkc.bits.param)
  if (!params.firstLevel) when (io.sinkc.valid) {
    params.ccover( probe_toN && io.schedule.bits.b.bits.param === toB, "MSHR_PROBE_FULL", "Client downgraded to N when asked only to do B")
    params.ccover(!probe_toN && io.schedule.bits.b.bits.param === toB, "MSHR_PROBE_HALF", "Client downgraded to B when asked only to do B")
    // Caution: the probe matches us only in set.
    // We would never allow an outer probe to nest until both w_[rp]probeack complete, so
    // it is safe to just unguardedly update the probe FSM.
    probes_done := probes_done | probe_bit
    probes_toN := probes_toN | Mux(probe_toN, probe_bit, 0.U)
    probes_noT := probes_noT || io.sinkc.bits.param =/= TtoT
    w_rprobeackfirst := w_rprobeackfirst || last_probe
    w_rprobeacklast := w_rprobeacklast || (last_probe && io.sinkc.bits.last)
    w_pprobeackfirst := w_pprobeackfirst || last_probe
    w_pprobeacklast := w_pprobeacklast || (last_probe && io.sinkc.bits.last)
    // Allow wormhole routing from sinkC if the first request beat has offset 0
    val set_pprobeack = last_probe && (io.sinkc.bits.last || request.offset === 0.U)
    w_pprobeack := w_pprobeack || set_pprobeack
    params.ccover(!set_pprobeack && w_rprobeackfirst, "MSHR_PROBE_SERIAL", "Sequential routing of probe response data")
    params.ccover( set_pprobeack && w_rprobeackfirst, "MSHR_PROBE_WORMHOLE", "Wormhole routing of probe response data")
    // However, meta-data updates need to be done more cautiously
    when (meta.state =/= INVALID && io.sinkc.bits.tag === meta.tag && io.sinkc.bits.data) { 
      meta.dirty := true.B 
      meta.modified_cores := meta.modified_cores | probe_bit // set modified bit on ProbeAckData
    } // !!!
  }
  when (io.sinkd.valid) {
    when (io.sinkd.bits.opcode === Grant || io.sinkd.bits.opcode === GrantData) {
      sink := io.sinkd.bits.sink
      w_grantfirst := true.B
      w_grantlast := io.sinkd.bits.last
      // Record if we need to prevent taking ownership
      bad_grant := io.sinkd.bits.denied
      // Allow wormhole routing for requests whose first beat has offset 0
      w_grant := request.offset === 0.U || io.sinkd.bits.last
      params.ccover(io.sinkd.bits.opcode === GrantData && request.offset === 0.U, "MSHR_GRANT_WORMHOLE", "Wormhole routing of grant response data")
      params.ccover(io.sinkd.bits.opcode === GrantData && request.offset =/= 0.U, "MSHR_GRANT_SERIAL", "Sequential routing of grant response data")
      gotT := io.sinkd.bits.param === toT
    }
    .elsewhen (io.sinkd.bits.opcode === ReleaseAck) {
      w_releaseack := true.B
    }
  }
  when (io.sinke.valid) {
    w_grantack := true.B
  }
  when (io.storeack) {
    w_store := true.B
  }

  // Bootstrap new requests
  val allocate_as_full = WireInit(new FullRequest(params), init = io.allocate.bits)
  val new_meta = Mux(io.allocate.valid && io.allocate.bits.repeat, final_meta_writeback, io.directory.bits)
  // new_meta.parrp_data_vec := Mux(io.allocate.valid && io.allocate.bits.set)
  val new_request = Mux(io.allocate.valid, allocate_as_full, request)
  val new_needT = needT(new_request.opcode, new_request.param)
  val new_clientBit = params.clientBit(new_request.source)
  val new_skipProbe = Mux(skipProbeN(new_request.opcode, params.cache.hintsSkipProbe), new_clientBit, 0.U)

  val prior = cacheState(final_meta_writeback, true.B)
  def bypass(from: CacheState, cover: Boolean)(implicit sourceInfo: SourceInfo): Unit = {
    if (cover) {
      params.ccover(prior === from.code, s"MSHR_${from}_BYPASS", s"State bypass transition from ${from} ${cfg}")
    } else {
      assert(!(prior === from.code), cf"State bypass from ${from} should be impossible ${cfg}")
    }
  }

  when (io.allocate.valid && io.allocate.bits.repeat) {
    bypass(S_INVALID,   f || p) // Can lose permissions (probe/flush)
    bypass(S_BRANCH,    b)      // MMIO read to read-only device
    bypass(S_BRANCH_C,  b && c) // you need children to become C
    bypass(S_TIP,       true)   // MMIO read || clean release can lead to this state
    bypass(S_TIP_C,     c)      // needs two clients || client + mmio || downgrading client
    bypass(S_TIP_CD,    c)      // needs two clients || client + mmio || downgrading client
    bypass(S_TIP_D,     true)   // MMIO write || dirty release lead here
    bypass(S_TRUNK_C,   c)      // acquire for write
    bypass(S_TRUNK_CD,  c)      // dirty release then reacquire
  }

  when (io.allocate.valid) {
    assert (!request_valid || (no_wait && io.schedule.fire))
    request_valid := true.B
    request := io.allocate.bits
  }

  // Create execution plan
  when (io.directory.valid || (io.allocate.valid && io.allocate.bits.repeat)) { //io.allocate is dequeuing from req queue
    meta_valid := true.B
    meta := new_meta
    probes_done := 0.U
    probes_toN := 0.U
    probes_noT := false.B
    gotT := false.B
    bad_grant := false.B

    // These should already be either true or turning true
    // We clear them here explicitly to simplify the mux tree
    s_rprobe         := true.B
    w_rprobeackfirst := true.B
    w_rprobeacklast  := true.B
    s_release        := true.B
    w_releaseack     := true.B
    s_pprobe         := true.B
    s_acquire        := true.B
    s_flush          := true.B
    w_grantfirst     := true.B
    w_grantlast      := true.B
    w_grant          := true.B
    w_pprobeackfirst := true.B
    w_pprobeacklast  := true.B
    w_pprobeack      := true.B
    s_probeack       := true.B
    s_grantack       := true.B
    s_execute        := true.B
    w_grantack       := true.B
    s_writeback      := true.B
    s_speculativerel := true.B

    // For C channel requests (ie: Release[Data])
    when (new_request.prio(2) && (!params.firstLevel).B) {
      s_execute := false.B
      s_writeback := false.B //we must always touch directory for LRU reasons, even if it is clean.
      // Do we need to go dirty?
      // when (new_request.opcode(0) && !new_meta.dirty) {
      //   s_writeback := false.B
      // }
      // // Does our state change?
      // when (isToB(new_request.param) && new_meta.state === TRUNK) {
      //   s_writeback := false.B
      // }
      // // Do our clients change?
      // when (isToN(new_request.param) && (new_meta.clients & new_clientBit) =/= 0.U) {
      //   s_writeback := false.B
      // }
      assert (new_meta.hit)
      when((new_request.opcode === 7.U && new_meta.state =/= TIP) || (new_request.opcode === 6.U && ((new_meta.modified_cores ^ new_clientBit) === 0.U)) ){ //need to speculatively writeback data from a ReleaseData for tighter WCL, leaves us in TIP state after release. Also release when the last modifier to a cache line releases, leaving us in TIP_C state
        // printf(cf"new_meta.state = ${new_meta.state} , meta.state = ${meta.state}, fmw.state = ${final_meta_writeback.state}\n")
        s_speculativerel := false.B
        w_releaseack := false.B
        w_store := false.B
      }
    }
    // For X channel requests (ie: flush)
    .elsewhen (new_request.control && params.control.B) { // new_request.prio(0)
      s_flush := false.B
      // Do we need to actually do something?
      when (new_meta.hit) {
        s_writeback := false.B //removed implicit wb from flush, so add it explicity.
        s_release := false.B
        w_releaseack := false.B
        // Do we need to shoot-down inner caches?
        when ((!params.firstLevel).B && (new_meta.clients =/= 0.U)) {
          s_rprobe := false.B
          w_rprobeackfirst := false.B
          w_rprobeacklast := false.B
        }
      }
    }
    // For A channel requests
    .otherwise { // new_request.prio(0) && !new_request.control
      s_execute := false.B
      // Do we need an eviction?
      when (!new_meta.hit && new_meta.state =/= INVALID) {
        s_release := false.B
        w_releaseack := false.B
        // Do we need to shoot-down inner caches?
        when ((!params.firstLevel).B & (new_meta.clients =/= 0.U)) {
          s_rprobe := false.B
          w_rprobeackfirst := false.B
          w_rprobeacklast := false.B
        }
      }
      // Do we need an acquire?
      when (!new_meta.hit || (new_meta.state === BRANCH && new_needT)) {
        s_acquire := false.B
        w_grantfirst := false.B
        w_grantlast := false.B
        w_grant := false.B
        s_grantack := false.B
        // s_writeback := false.B
      }
      // Do we need a probe?
      when ((!params.firstLevel).B && (new_meta.hit &&
            (new_needT || new_meta.state === TRUNK) &&
            (new_meta.clients & ~new_skipProbe) =/= 0.U)) {
        s_pprobe := false.B
        w_pprobeackfirst := false.B
        w_pprobeacklast := false.B
        w_pprobeack := false.B
        // s_writeback := false.B
      }
      // Do we need a grantack?
      when (new_request.opcode === AcquireBlock || new_request.opcode === AcquirePerm) {
        w_grantack := false.B
        // s_writeback := false.B
      }
      // Becomes dirty?
      when (!new_request.opcode(2) && new_meta.hit && !new_meta.dirty) {
        // s_writeback := false.B
      }
      // perhaps every request should writeback to directory for parrp purposes... right?
      s_writeback := false.B //schedule a writeback for any access, regardless of physical cache changing or not for LRU & directory purposes.
    }
  }
}
