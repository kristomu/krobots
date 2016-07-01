
#ifndef _KROB_CLOG
#define _KROB_CLOG

#include "cpu.cc"
#include "../robot.cc"
#include <vector>
#include <list>
#include <assert.h>

using namespace std;

class corelogic {

	private:
		CPU execution_unit;
		vector<code_line> program;
		vector<short> pseudo_stack, memory;
		vector<int> numeric_jump_table, alnum_jump_table;

		void postinit_CPU(CPU & target, const vector<code_line> & prog);

	public:
		corelogic(int stack_size, int memory_size, int permitted_jumps);

		// CPUs shouldn't be copied, only the contents
		corelogic(const corelogic & input);
		const corelogic & operator=(const corelogic & source);

		corelogic(int stack_size, int memory_size, 
				vector<code_line> & prog_to_load,
				vector<int> & numeric_jumps, vector<int> &
				alnum_jumps);
		bool execute_one(robot & shell,
				const list<Unit *> & active_robots,
				list<missile> & missiles, list<mine> & mines,
				vector<set<robot *> > & comms_lookup,
				const int matchnum, const int total_matches, 
				const coordinate arena_size, 
				run_error & error_out);
		bool execute_multiple(const int how_many, robot & shell, 
				const list<Unit *> & active_robots,
				list<missile> & missiles, list<mine> & mines,
				vector<set<robot *> > & comms_lookup,
				const int matchnum, const int total_matches,
				const coordinate arena_size, bool ignore_errors,
				run_error & last_error, int & cycles_left);

		// For debugging
		int get_IP() const { return(execution_unit.get_ip()); }
		int get_old_IP() const { return(execution_unit.get_oldip()); }
		code_line get_instruction(const int pos) const;
		code_line get_instr_at_IP() const;
		code_line get_instr_at_oldIP() const;

		// So we don't have to reinit the jump tables every time we
		// have a new round, as that kind of memory copy takes time.
		void reset_CPU();
		void reset_core(bool reset_memory);
};

// Caching and other just-before-launch setup events go here.
void corelogic::postinit_CPU(CPU & target, const vector<code_line> & prog) {
	target.cache_consistency(prog);
}

corelogic::corelogic(int stack_size, int memory_size, vector<code_line> &
		prog_to_load, vector<int> & numeric_jumps, vector<int> &
		alnum_jumps) {
	
	assert(prog_to_load.size() > 0);

	pseudo_stack.resize(stack_size, 0);
	memory.resize(memory_size, 0);
	program = prog_to_load;
	numeric_jump_table = numeric_jumps;
	alnum_jump_table = alnum_jumps;
	postinit_CPU(execution_unit, program);
}

// CPUs shouldn't be copied, only the contents
corelogic::corelogic(const corelogic & input) {

	pseudo_stack = input.pseudo_stack;
	memory = input.memory;
	program = input.program;
	execution_unit.cache_consistency(program);
	numeric_jump_table = input.numeric_jump_table;
	alnum_jump_table = input.alnum_jump_table;
}

const corelogic & corelogic::operator=(const corelogic & input) {
	if (&input == this) return(*this);

	pseudo_stack = input.pseudo_stack;
	memory = input.memory;
	program = input.program;
	execution_unit.cache_consistency(program);
	numeric_jump_table = input.numeric_jump_table;
	alnum_jump_table = input.alnum_jump_table;
}

// Shell is the robot this CPU manipulates. Active_robots is the list of active
// robots, which is used for scanning and other probes. Missiles and mines are
// referenced when shooting or laying a mine, or commanding mines to blow up.
// Comms_lookup is used for transmitting messages and changing comms channels,
// and the ints are for the "poor man's p-space".
bool corelogic::execute_one(robot & shell,
		const list<Unit *> & active_robots, list<missile> & missiles,
		list<mine> & mines, vector<set<robot *> > & comms_lookup,
		const int matchnum, const int total_matches, 
		const coordinate arena_size, run_error & error_out) {

	if (shell.dead()) {
		error_out = ERR_NOT_IMPLEMENTED;
		return(false);
	}

	// Calling this takes a lot of time, for some reason.
	return(execution_unit.execute(program, memory, pseudo_stack, 
			numeric_jump_table, alnum_jump_table, shell,
			active_robots, missiles, mines, comms_lookup,
			matchnum, total_matches, arena_size, error_out));
}

// Here we run multiple instructions on the CPU. If ignore_errors is true,
// we don't break on an error; if it's false, we do break on error (as ATR2
// presumably does -- or rather, ATR2 pauses all execution when giving the
// error). The return value is how many cycles we actually used, which only
// makes sense if we don't ignore errors.
// To emulate ATR2, we should do something like this: break on error, then
// have the caller check if there's a nonzero result. If so, post the error,
// draw, and wait a while, then execute again.

bool corelogic::execute_multiple(const int how_many, robot & shell,
		const list<Unit *> & active_robots, list<missile> & missiles,
		list<mine> & mines, vector<set<robot *> > & comms_lookup,
		const int matchnum, const int total_matches, 
		const coordinate arena_size, bool ignore_errors, 
		run_error & last_error, int & cycles_left) {

	cycles_left = how_many;
	while (cycles_left > 0) {

		// Mop up penalties
		cycles_left -= execution_unit.withdraw_penalty(cycles_left);

		if (cycles_left > 0)
			if (!execute_one(shell, active_robots, missiles, mines,
						comms_lookup, matchnum, 
						total_matches, arena_size,
						last_error) && !ignore_errors)
				return(false);

		// Check that we haven't gone to overheating, which shuts
		// down the CPU completely. (BLUESKY: Retain the instructions
		// so that they can be used when it comes out of "stasis". But
		// it's not clear whether this is the right thing to do.)
		if (!shell.is_CPU_working() || shell.dead())
			cycles_left = 0;

	}

	return(true);
}

code_line corelogic::get_instruction(const int pos) const {
	code_line default_cl;
	if (pos < 0 || pos >= (int)program.size()) return(default_cl);
	else return(program[pos]);
}

code_line corelogic::get_instr_at_IP() const {
	return(get_instruction(get_IP()));
}

code_line corelogic::get_instr_at_oldIP() const {
	return(get_instruction(get_old_IP()));
}

void corelogic::reset_CPU() {
	execution_unit = (CPU());
	// Redo caching
	postinit_CPU(execution_unit, program);
}

void corelogic::reset_core(bool reset_memory) {
	reset_CPU();

	// Null memory and stack. The general rule here is: any array that
	// can be written to by the program should be nulled.
	if (reset_memory) {
		vector<short>::iterator pos;
		for (pos = memory.begin(); pos != memory.end(); ++pos)	
			*pos = 0;
		for (pos = pseudo_stack.begin(); pos != pseudo_stack.end(); 
				++pos)
			*pos = 0;
	}
}

#endif
