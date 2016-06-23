# Copyright (c) 2016, TU Dresden
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
# OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Christian Menard


ARCH = ARM
VARIANT = opt

SYSTEMC_INC = /opt/systemc/include
SYSTEMC_LIB = /opt/systemc/lib-linux

CXXFLAGS = -I../../../../build/$(ARCH) -L../../../../build/$(ARCH)
CXXFLAGS += -I../../../systemc/
CXXFLAGS += -I../../
CXXFLAGS += -I$(SYSTEMC_INC) -L$(SYSTEMC_LIB)
CXXFLAGS += -std=c++0x
CXXFLAGS += -g
CXXFLAGS += -DSC_INCLUDE_DYNAMIC_PROCESSES -DTRACING_ON

ifeq ($(VARIANT), debug)
	CXXFLAGS += -DDEBUG
endif

LIBS = -lgem5_$(VARIANT) -lsystemc

ALL = gem5.$(VARIANT).sc

all: $(ALL)

.cc.o:
	$(CXX) $(CXXFLAGS) -c -o $@ $<

sc_gem5_control.o: ../../../systemc/sc_gem5_control.cc \
	../../../systemc/sc_gem5_control.hh
sc_logger.o: ../../../systemc/sc_logger.cc ../../../systemc/sc_logger.hh
sc_module.o: ../../../systemc/sc_module.cc ../../../systemc/sc_module.hh
sc_mm.o: ../../sc_mm.cc ../../sc_mm.hh
sc_ext.o: ../../sc_ext.cc ../../sc_ext.hh
sc_master_port.o: ../../sc_master_port.cc ../../sc_master_port.hh \
	../../payload_event.hh
sc_master_port.o: ../../sc_master_port.cc ../../sc_master_port.hh \
	../../payload_event.hh
sc_slave_port.o: ../../sc_slave_port.cc ../../sc_slave_port.hh \
	../../payload_event.hh
stats.o: ../../../systemc/stats.cc ../../../systemc/stats.hh
sim_control.o: ../../sim_control.cc ../../sim_control.hh ../../ \
	../../sc_slave_port.hh ../../sc_master_port.hh
main.o: main.cc ../../../systemc/sc_logger.hh ../../../systemc/sc_module.hh \
	../../../systemc/stats.hh

gem5.$(VARIANT).sc: main.o ../../../systemc/stats.o \
	../../../systemc/sc_gem5_control.o ../../../systemc/sc_logger.o \
	../../../systemc/sc_module.o ../../sc_mm.o ../../sc_ext.o \
	../../sc_slave_port.o ../../sc_master_port.o ../../sim_control.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

clean:
	$(RM) $(ALL)
	$(RM) *.o
	$(RM) ../../*.o
	$(RM) -r m5out
