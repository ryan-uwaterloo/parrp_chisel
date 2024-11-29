package paarp_chisel.util.printf_helper

import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink._

object ReqPrinter{ //this doesn't *really* work because of runtime stuff - it'd hafta be a module.
    def printSrcA(req: TLBundleA): Unit = {
        //val opcode = TLMessages.a(req.opcode)._1
        printf(cf"New Source A Request! opcode: ${TLMessages.a(req.opcode.litValue.toInt)._1}, addr: ${req.address}\n")
    }
}
