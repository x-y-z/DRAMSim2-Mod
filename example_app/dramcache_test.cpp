/*********************************************************************************
*  Copyright (c) 2010-2011, Elliott Cooper-Balis
*                             Paul Rosenfeld
*                             Bruce Jacob
*                             University of Maryland
*                             dramninjas [at] gmail [dot] com
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright notice,
*        this list of conditions and the following disclaimer.
*
*     * Redistributions in binary form must reproduce the above copyright notice,
*        this list of conditions and the following disclaimer in the documentation
*        and/or other materials provided with the distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
*  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************/




#include <stdio.h>
#include <iostream>
#include "dramcache_test.h"
#include "CSVWriter.h"

using namespace DRAMSim;

uint64_t cpu_cycle;

/* callback functors */
void some_object::read_complete(unsigned id, uint64_t address, uint64_t clock_cycle)
{
	printf("[Callback] read complete: %d 0x%lx cycle=%lu, cpu=%lu\n", id, address, clock_cycle,cpu_cycle);
}

void some_object::write_complete(unsigned id, uint64_t address, uint64_t clock_cycle)
{
	printf("[Callback] write complete: %d 0x%lx cycle=%lu, cpu=%lu\n", id, address, clock_cycle,cpu_cycle);
}

/* FIXME: this may be broken, currently */
void power_callback(double a, double b, double c, double d)
{
//	printf("power callback: %0.3f, %0.3f, %0.3f, %0.3f\n",a,b,c,d);
}

int some_object::add_one_and_run(DRAMSimInterface *mem, uint64_t addr)
{
    DRAMSimTransaction *trans = NULL;

	/* create a transaction and add it */
	bool isWrite = false;
	mem->addTransaction(isWrite, addr,0,0,0,1);

	// send a read to channel 1 on the same cycle
    //addr = 1LL<<33 | addr;
    //mem->addTransaction(isWrite, addr,0,0,0,2);

	//for (int i=0; i<5; i++)
	//{
        //cpu_cycle++;
		//mem->update();
	//}

	/* add another some time in the future */

	// send a write to channel 0
    //addr = 0x900012;
    //isWrite = true;
    //mem->addTransaction(isWrite, addr,0,0,0,1);


	/* do a bunch of updates (i.e. clocks) -- at some point the callback will fire */
	for (int i=0; i<195; i++)
	{
        cpu_cycle++;
		mem->update();
	}

	/* get a nice summary of this epoch */
	//mem->printStats(true);
    CSVWriter out(std::cout);
    mem->dumpStats(out);

	return 0;
}

int main()
{
	some_object obj;
	TransactionCompleteCB *read_cb = new Callback<some_object, void, unsigned, uint64_t, uint64_t>(&obj, &some_object::read_complete);
	TransactionCompleteCB *write_cb = new Callback<some_object, void, unsigned, uint64_t, uint64_t>(&obj, &some_object::write_complete);

	/* pick a DRAM part to simulate */
	DRAMSimInterface *mem = getMemorySystemInstance("ini/DDR_micron_256.ini", "example_app/system.ini", "..", "example_app", 256);

    cpu_cycle = 0;

	mem->registerCallbacks(read_cb, write_cb, power_callback);
    mem->setCPUClockSpeed(2*1600000000UL);
	//DRAMSimInterface *mem2 = getMemorySystemInstance("ini/DDR2_micron_16M_8b_x8_sg3E.ini", "system.ini", "..", "example_app", 16384);

	//mem2->registerCallbacks(read_cb, write_cb, power_callback);

	printf("dramsim_test main()\n");
	printf("-----MEM1------\n");
	obj.add_one_and_run(mem, 0x2000000000001UL);
	obj.add_one_and_run(mem, 0x200002UL);

	//printf("-----MEM2------\n");
	//obj.add_one_and_run(mem2, 0x300002UL);

    delete mem;
    //delete mem2;
    delete read_cb;
    delete write_cb;

	return 0;
}

