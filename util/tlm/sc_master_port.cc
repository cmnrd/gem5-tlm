/*
 * Copyright (c) 2016, TU Dresden
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @date 23.05.2016
 * @author Christian Menard
 */

#include "params/ExternalMaster.hh"
#include "sc_master_port.hh"
#include "sim/system.hh"

namespace Gem5SystemC
{

PacketPtr
MasterPort::generatePacket(tlm::tlm_generic_payload& trans)
{
    Request::Flags flags;
    auto req = new Request(trans.get_address(), trans.get_data_length(), flags,
                           owner.masterId);

    MemCmd cmd;

    switch (trans.get_command()) {
        case tlm::TLM_READ_COMMAND:
            cmd = MemCmd::ReadReq;
            break;
        case tlm::TLM_WRITE_COMMAND:
            cmd = MemCmd::WriteReq;
            break;
        default:
            SC_REPORT_FATAL("MasterPort",
                            "received transaction with unsupported command");
    }

    auto pkt = new Packet(req, cmd);
    pkt->dataStatic(trans.get_data_ptr());

    return pkt;
}

void
MasterPort::destroyPacket(PacketPtr pkt)
{
    delete pkt;
}

MasterPort::MasterPort(const std::string& name_,
                               const std::string& systemc_name,
                               ExternalMaster& owner_,
                               Module& module)
  : ExternalMaster::Port(name_, owner_)
  , tSocket(systemc_name.c_str())
  , peq(this, &MasterPort::peq_cb)
  , waitForRetry(false)
  , pendingRequest(nullptr)
  , needToSendRetry(false)
  , responseInProgress(false)
  , module(module)
{
    auto system =
      reinterpret_cast<const ExternalMasterParams*>(owner_.params())->system;

    /*
     * Register the TLM non-blocking interface when using gem5 Timing mode and
     * the TLM blocking interface when using the gem5 Atomic mode.
     * Then the magic (TM) in simple_target_socket automatically transforms
     * non-blocking in blocking transactions and vice versa.
     */
    if (system->isTimingMode()) {
        SC_REPORT_INFO("MasterPort", "register non-blocking interface");
        tSocket.register_nb_transport_fw(this,
                                         &MasterPort::nb_transport_fw);
    } else if (system->isAtomicMode()) {
        SC_REPORT_INFO("MasterPort", "register blocking interface");
        tSocket.register_b_transport(this, &MasterPort::b_transport);
    } else {
        panic("gem5 operates neither in Timing nor in Atomic mode");
    }

    tSocket.register_transport_dbg(this, &MasterPort::transport_dbg);
}

void
MasterPort::checkTransaction(tlm::tlm_generic_payload& trans)
{
    if (trans.is_response_error()) {
        char txt[100];
        sprintf(txt,
                "Transaction returned with error, response status = %s",
                trans.get_response_string().c_str());
        SC_REPORT_ERROR("TLM-2", txt);
    }
}

tlm::tlm_sync_enum
MasterPort::nb_transport_fw(tlm::tlm_generic_payload& trans,
                                tlm::tlm_phase& phase, sc_core::sc_time& delay)
{
    uint64_t adr = trans.get_address();
    unsigned len = trans.get_data_length();
    unsigned char* byteEnable = trans.get_byte_enable_ptr();
    unsigned width = trans.get_streaming_width();

    // check the transaction attributes for unsupported features ...
    if (byteEnable != 0) {
        trans.set_response_status(tlm::TLM_BYTE_ENABLE_ERROR_RESPONSE);
        return tlm::TLM_COMPLETED;
    }
    if (len > 4 || width < len) {
        trans.set_response_status(tlm::TLM_BURST_ERROR_RESPONSE);
        return tlm::TLM_COMPLETED;
    }

    // ... and queue the valid transaction
    peq.notify(trans, phase, delay);
    return tlm::TLM_ACCEPTED;
}

void
MasterPort::peq_cb(tlm::tlm_generic_payload& trans,
                       const tlm::tlm_phase& phase)
{
    // catch up with SystemC time
    module.catchup();
    assert(curTick() == sc_core::sc_time_stamp().value());

    switch (phase) {
        case tlm::BEGIN_REQ:
            handleBeginReq(trans);
            break;
        case tlm::END_RESP:
            handleEndResp(trans);
            break;
        default:
            panic("unimplemented phase in callback");
    }

    // the functions called above may have scheduled gem5 events
    // -> notify the event loop handler
    module.notify();
}

void
MasterPort::handleBeginReq(tlm::tlm_generic_payload& trans)
{
    sc_assert(!waitForRetry);
    sc_assert(pendingRequest == nullptr);

    trans.acquire();
    auto pkt = generatePacket(trans);

    auto tlmSenderState = new TlmSenderState(trans);
    pkt->pushSenderState(tlmSenderState);

    if (sendTimingReq(pkt)) { // port is free -> send END_REQ immediately
        sendEndReq(trans);
    } else { // port is blocked -> wait for retry before sending END_REQ
        waitForRetry = true;
        pendingRequest = &trans;
    }
}

void
MasterPort::handleEndResp(tlm::tlm_generic_payload& trans)
{
    sc_assert(responseInProgress);

    responseInProgress = false;

    checkTransaction(trans);

    if (needToSendRetry) {
        sendRetryResp();
        needToSendRetry = false;
    }

    trans.release();
}

void
MasterPort::sendEndReq(tlm::tlm_generic_payload& trans)
{
    tlm::tlm_phase phase = tlm::END_REQ;
    auto delay = sc_core::SC_ZERO_TIME;

    auto status = tSocket->nb_transport_bw(trans, phase, delay);
    panic_if(status != tlm::TLM_ACCEPTED,
             "Unexpected status after sending END_REQ");
}

void
MasterPort::b_transport(tlm::tlm_generic_payload& trans,
                        sc_core::sc_time& t)
{
    auto pkt = generatePacket(trans);

    // send an atomic request to gem5
    Tick ticks = sendAtomic(pkt);
    panic_if(!pkt->isResponse(), "Packet sending failed!\n");

    // one tick is a pico second
    auto delay = sc_core::sc_time(static_cast<double>(ticks), sc_core::SC_PS);

    // update time
    t += delay;

    destroyPacket(pkt);

    trans.set_response_status(tlm::TLM_OK_RESPONSE);
}

unsigned int
MasterPort::transport_dbg(tlm::tlm_generic_payload& trans)
{
    auto pkt = generatePacket(trans);

    sendFunctional(pkt);

    destroyPacket(pkt);

    return trans.get_data_length();
}

bool
MasterPort::get_direct_mem_ptr(tlm::tlm_generic_payload& trans,
                               tlm::tlm_dmi& dmi_data)
{
    return false;
}

bool
MasterPort::recvTimingResp(PacketPtr pkt)
{
    // exclusion rule
    // We need to Wait for END_RESP before sending next BEGIN_RESP
    if (responseInProgress) {
        sc_assert(!needToSendRetry);
        needToSendRetry = true;
        return false;
    }

    sc_assert(pkt->isResponse());
    responseInProgress = true;

    // pay for annotaded transport delays
    auto delay =
      sc_core::sc_time::from_value(pkt->payloadDelay + pkt->headerDelay);

    auto tlmSenderState =
      reinterpret_cast<TlmSenderState*>(pkt->popSenderState());
    auto& trans = tlmSenderState->trans;

    // clean up
    delete tlmSenderState;
    destroyPacket(pkt);

    auto status = sendBeginResp(trans, delay);

    if (status == tlm::TLM_UPDATED) {
        responseInProgress = false;
        trans.release();
    }

    return true;
}

tlm::tlm_sync_enum
MasterPort::sendBeginResp(tlm::tlm_generic_payload& trans,
                          sc_core::sc_time& delay)
{
    tlm::tlm_phase phase = tlm::BEGIN_RESP;

    trans.set_response_status(tlm::TLM_OK_RESPONSE);

    auto status = tSocket->nb_transport_bw(trans, phase, delay);
    panic_if(status != tlm::TLM_ACCEPTED && status != tlm::TLM_COMPLETED,
             "Unexpected status after sending BEGIN_RESP");

    return status;
}

void
MasterPort::recvReqRetry()
{
    sc_assert(waitForRetry);
    sc_assert(pendingRequest != nullptr);

    auto& trans = *pendingRequest;

    waitForRetry = false;
    pendingRequest = nullptr;

    // retry
    handleBeginReq(trans);
}

void
MasterPort::recvRangeChange()
{
    SC_REPORT_WARNING("MasterPort",
                      "received address range change but ignored it");
}

class MasterPortHandler : public ExternalMaster::Handler
{
    Module& module;

  public:
    MasterPortHandler(Module& module) : module(module) {}

    ExternalMaster::Port* getExternalPort(const std::string& name,
                                          ExternalMaster& owner,
                                          const std::string& port_data)
    {
        // This will make a new initiatiator port
        return new MasterPort(name, port_data, owner, module);
    }
};

void
MasterPort::registerPortHandler(Module& module)
{
    ExternalMaster::registerHandler("tlm_master",
                                    new MasterPortHandler(module));
}

} // namespace Gem5SystemC
