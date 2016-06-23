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
 * @date 25.05.2016
 * @author Christian Menard
 */

#include <tlm_utils/peq_with_cb_and_phase.h>
#include <tlm_utils/simple_initiator_socket.h>

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <systemc>
#include <tlm>

#include "sc_master_port.hh"
#include "sc_mm.hh"
#include "sim_control.hh"
#include "stats.hh"

// Defining global string variable decalred in stats.hh
std::string filename;

void
reportHandler(const sc_core::sc_report& report,
              const sc_core::sc_actions& actions)
{
    uint64_t systemc_time = report.get_time().value();
    uint64_t gem5_time = curTick();

    std::cerr << report.get_time();

    if (gem5_time < systemc_time) {
        std::cerr << " (<) ";
    } else if (gem5_time > systemc_time) {
        std::cerr << " (!) ";
    } else {
        std::cerr << " (=) ";
    }

    std::cerr << ": " << report.get_msg_type() << ' ' << report.get_msg()
              << '\n';
}

class TrafficGen : public sc_core::sc_module
{
  private:
    MemoryManager mm;

    tlm::tlm_generic_payload* requestInProgress;

    uint32_t dataBuffer;

    sc_core::sc_event endRequestEvent;

    tlm_utils::peq_with_cb_and_phase<TrafficGen> peq;

  public:
    tlm_utils::simple_initiator_socket<TrafficGen> socket;

    SC_CTOR(TrafficGen)
      : socket("socket")
      , requestInProgress(0),
      peq(this, &TrafficGen::peq_cb)
    {
        socket.register_nb_transport_bw(this, &TrafficGen::nb_transport_bw);
        SC_THREAD(process);
    }

    void process()
    {
        srand(time(NULL));

        unsigned const memSize = (1 << 10); // 512 MB

        while (true) {

            wait(sc_core::sc_time(rand() % 100, sc_core::SC_NS));

            auto trans = mm.allocate();
            trans->acquire();

            std::string cmdStr;
            if (rand() % 2) // Generate a write request?
            {
                cmdStr = "write";
                trans->set_command(tlm::TLM_WRITE_COMMAND);
                dataBuffer = rand();
            } else {
                cmdStr = "read";
                trans->set_command(tlm::TLM_READ_COMMAND);
            }

            trans->set_data_ptr(reinterpret_cast<unsigned char*>(&dataBuffer));
            trans->set_address(rand() % memSize);
            trans->set_data_length(4);
            trans->set_streaming_width(4);
            trans->set_byte_enable_ptr(0);
            trans->set_dmi_allowed(0);
            trans->set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

            // honor the BEGIN_REQ/END_REQ exclusion rule
            if (requestInProgress)
                sc_core::wait(endRequestEvent);

            std::stringstream ss;
            ss << "Send " << cmdStr << " request @0x" << std::hex
               << trans->get_address();
            SC_REPORT_INFO("Traffic Generator", ss.str().c_str());

            // send the request
            requestInProgress = trans;
            tlm::tlm_phase phase = tlm::BEGIN_REQ;
            auto delay = sc_core::SC_ZERO_TIME;

            auto status = socket->nb_transport_fw(*trans, phase, delay);

            // Check status
            if (status == tlm::TLM_UPDATED) {
                peq.notify(*trans, phase, delay);
            } else if (status == tlm::TLM_COMPLETED) {
                requestInProgress = 0;
                checkTransaction(*trans);
                SC_REPORT_INFO("Traffic Generator", "request completed");
                trans->release();
            }
        }
    }

    void peq_cb(tlm::tlm_generic_payload& trans, const tlm::tlm_phase& phase)
    {
        if (phase == tlm::END_REQ ||
            (&trans == requestInProgress && phase == tlm::BEGIN_RESP)) {
            // The end of the BEGIN_REQ phase
            requestInProgress = 0;
            endRequestEvent.notify();
        } else if (phase == tlm::BEGIN_REQ || phase == tlm::END_RESP)
            SC_REPORT_FATAL("TLM-2",
                            "Illegal transaction phase received by initiator");

        if (phase == tlm::BEGIN_RESP) {
            checkTransaction(trans);
            SC_REPORT_INFO("Traffic Generator", "received response");

            // Send end response
            tlm::tlm_phase fw_phase = tlm::END_RESP;

            // stress the retry mechanism by deferring the response
            auto delay = sc_core::sc_time(rand() % 50, sc_core::SC_NS);
            socket->nb_transport_fw(trans, fw_phase, delay);
            trans.release();
        }
    }

    void checkTransaction(tlm::tlm_generic_payload& trans)
    {
        if (trans.is_response_error()) {
            char txt[100];
            sprintf(txt,
                    "Transaction returned with error, response status = %s",
                    trans.get_response_string().c_str());
            SC_REPORT_ERROR("TLM-2", txt);
        }
    }

    virtual tlm::tlm_sync_enum nb_transport_bw(tlm::tlm_generic_payload& trans,
                                               tlm::tlm_phase& phase,
                                               sc_core::sc_time& delay)
    {
        peq.notify(trans, phase, delay);
        return tlm::TLM_ACCEPTED;
    }
};

int
sc_main(int argc, char** argv)
{
    sc_core::sc_report_handler::set_handler(reportHandler);

    SimControl sim_control("gem5", argc, argv);
    TrafficGen traffic_gen("traffic_gen");

    filename = "m5out/stats-systemc.txt";

    tlm::tlm_target_socket<>* mem_port =
      dynamic_cast<tlm::tlm_target_socket<>*>(
        sc_core::sc_find_object("gem5.memory"));

    if (mem_port) {
        SC_REPORT_INFO("sc_main", "Port Found");
        traffic_gen.socket.bind(*mem_port);
    } else {
        SC_REPORT_FATAL("sc_main", "Port Not Found");
        std::exit(EXIT_FAILURE);
    }

    std::cout << "Starting sc_main" << std::endl;

    sc_core::sc_start(); // Run to end of simulation

    SC_REPORT_INFO("sc_main", "End of Simulation");

    CxxConfig::statsDump();

    return EXIT_SUCCESS;
}
