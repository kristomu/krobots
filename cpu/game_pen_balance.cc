// Penalty numbers for various instructions, port accesses, and interrupts.
// These determine how many cycles the instruction requires in order to execute.
// The CPU executes the command first, then it waits for as many CPU cycles as 
// required before executing any new commands.

// We use switches because it's more clear what it does. If that's too slow,
// we might switch to vectors later.

#ifndef _KROB_GPBALANCE
#define _KROB_GPBALANCE

#include "prog_constants.h"

#include <vector>		// for max(x, y)
using namespace std;

class penalty_balance {

	private:
		int get_interrupt_penalty(int int_number);
		int get_port_penalty(int port_number);

	public:
		int get_penalty(command command_in, int argument);
};

int penalty_balance::get_interrupt_penalty(int interrupt_number) {

	// These are given in the ATR2 doc.
	
	switch(interrupt_number) {
		case I_DESTRUCT:	return(0);	// Not applicable
		case I_RESET:		return(10);
		case I_LOCATE:		return(5);
		case I_KEEPSHIFT:	return(2);
		case I_OVERBURN:	return(1);
		case I_ID:		return(2);
		case I_TIMER:		return(2);
		case I_ANGLE:		return(32);
		case I_TARGETID:	return(1);
		case I_TARGETINFO:	return(2);
		case I_GAMEINFO:	return(4);
		case I_ROBOTINFO:	return(5);
		case I_COLLISIONS:	return(1);
		case I_RESETCOLCNT:	return(1);
		case I_TRANSMIT:	return(1);
		case I_RECEIVE:		return(1);
		case I_DATAREADY:	return(1);
		case I_CLEARCOM:	return(1);
		case I_KILLS:		return(3);
		case I_CLEARMETERS:	return(1);
		default:		return(1);
	}
}

int penalty_balance::get_port_penalty(int port_number) {

	// As above, but note that this does not include the execution time
	// required for all port access instructions.
	
	// All ports not included take zero time to access (apart from the
	// common exec. time)
	
	switch(port_number) {
		default:		return(0);
		case P_SCAN:		return(1);
		case P_ACCURACY:	return(1);	// Waste of cycles
		case P_RADAR:		return(3);
		case P_SONAR:		return(40);	// Yipes!
	};
}

int penalty_balance::get_penalty(command command_in, int argument) {

	// Default penalty is 1 (for imaginary opcodes, etc, but not jumps).
	// UNCHECKED, but I'm reasonably sure that's how ATR2 does it, too.
	
	// Note that the assumption that default is 1 is relied on, just not
	// explicitly, but implicitly also; opcodes with T = 1 are omitted
	// here.

	switch(command_in) {
		default:	return(1);
		case CMD_MPY:	return(10);
		case CMD_DIV:	return(10);
		case CMD_MOD:	return(10);
		case CMD_JLS:	return(0);	// Separate counter prevents
		case CMD_JGR:	return(0);	// infinite loops
		case CMD_JNE:	return(0);
		case CMD_JE:	return(0);
		case CMD_JAE:	return(0);
		case CMD_JLE:	return(0);
		case CMD_JZ:	return(0);
		case CMD_JNZ:	return(0);
		case CMD_XCHG:	return(3);
		case CMD_TEST:	return(2);
		case CMD_LOC:	return(2);
		case CMD_GET:	return(2);
		case CMD_INT:	return(get_interrupt_penalty(argument));
		case CMD_IPO:	return(4 + get_port_penalty(argument));
		case CMD_OPO:	return(4 + get_port_penalty(argument));
		case CMD_DELAY:	return(max(1, argument));
		case CMD_ERR:	return(0);
	}
}

#endif
