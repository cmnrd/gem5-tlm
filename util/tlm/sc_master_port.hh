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

#pragma once

#include <tlm_utils/peq_with_cb_and_phase.h>
#include <tlm_utils/simple_target_socket.h>

#include <tlm>

#include <mem/external_master.hh>
#include <payload_event.hh>
#include <sc_module.hh>

namespace Gem5SystemC {

class MasterPort : public ExternalMaster::Port
{
  private:
    struct TlmSenderState : public Packet::SenderState
    {
        tlm::tlm_generic_payload& trans;
        TlmSenderState(tlm::tlm_generic_payload& trans)
          : trans(trans)
        {
        }
    };

    tlm_utils::peq_with_cb_and_phase<MasterPort> peq;

    bool waitForRetry;
    tlm::tlm_generic_payload* pendingRequest;

    bool needToSendRetry;

    bool responseInProgress;

    // Keep a reference to the gem5 SystemC module
    Module& module;

  public:
    tlm_utils::simple_target_socket<MasterPort> tSocket;

  protected:
    // payload event call back
    void peq_cb(tlm::tlm_generic_payload& trans, const tlm::tlm_phase& phase);

    // The TLM target interface
    tlm::tlm_sync_enum nb_transport_fw(tlm::tlm_generic_payload& trans,
                                       tlm::tlm_phase& phase,
                                       sc_core::sc_time& t);
    void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& t);
    unsigned int transport_dbg(tlm::tlm_generic_payload& trans);
    bool get_direct_mem_ptr(tlm::tlm_generic_payload& trans,
                            tlm::tlm_dmi& dmi_data);

    // Gem5 MasterPort interface
    bool recvTimingResp(PacketPtr pkt);
    void recvReqRetry();
    void recvRangeChange();

  public:
    MasterPort(const std::string& name_,
               const std::string& systemc_name,
               ExternalMaster& owner_,
               Module& module);

    static void registerPortHandler(Module& module);

    friend PayloadEvent<MasterPort>;

  private:
    void sendEndReq(tlm::tlm_generic_payload& trans);
    tlm::tlm_sync_enum sendBeginResp(tlm::tlm_generic_payload& trans,
                                     sc_core::sc_time& delay);

    void handleBeginReq(tlm::tlm_generic_payload& trans);
    void handleEndResp(tlm::tlm_generic_payload& trans);

    PacketPtr generatePacket(tlm::tlm_generic_payload& trans);
    void destroyPacket(PacketPtr pkt);

    void checkTransaction(tlm::tlm_generic_payload& trans);
};
}
