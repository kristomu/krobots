
// CPU and "motherboard". CPU does the execution of commands, and contains the
// IP and penalty cycle counter. Motherboard (called Core) has the rest.

// Check performance stuff once we've finished the current test.

#ifndef _KROB_CORE
#define _KROB_CORE

#include "errors.h"
#include "prog_constants.h"
#include "code_line.cc"
#include "game_pen_balance.cc"
#include "../tools.cc"
#include "../robot.cc"
#include "../random.cc"

#include <vector>
#include <assert.h>

using namespace std;

const int micropenalty_limit = 20;

class CPU {

	private:

		penalty_balance pbalance;

	//	code_line our_line;

		bool cached;
		bool all_consistent;

		//vector<char> static_consistency;

		int ip;		// instruction pointer
		int oldip; // For returns that fail. Otherwise -1.
		int penalty;	// cycle penalty
		int micropenalty;	// to avoid infinite loops
	
		void set_ip(int new_ip, int prog_size);
		void set_literal_penalty(int new_penalty);
		void set_penalty(command command_type, int a_field_direct);

		// These return true if the given command writes to the a-field,
		// b-field, or jumps to the label given by the integer value of
		// the a-field, respectively. This lets us bail early on things
		// like mov 3, ax, and handle jumps of the type "jmp ax".
		bool a_field_must_be_rw(const command cmd_in) const;
		bool b_field_must_be_rw(const command cmd_in) const;
		bool jumps_to_label(const command cmd_in) const;
		bool jumps_to_label(const field_entry & in) const;

		int get_reference_count(const field_entry & fe) const;
		bool consistent(const command opcode, 
				const code_line & cmd_line) const;
		bool consistent(const code_line & cmd_line) const;
	
		int resolve_reference(const field_entry & cmd_line, 
				const vector<short> & memory,
				const vector<code_line> & program,
				const int code_size,
				const robot & shell) const;
		int dereference_jump(const field_entry & operand, 
				const int direct, 
				const vector<int> & numeric_table,
				const vector<int> & alnum_table, 
				int prog_size, bool force_n) const;

		inline bool accessible_memory(int location, 
				const vector<short> & memory,
			 	const int code_size, bool write) const;

		int access_memory(int location, const vector<short> & memory,
				const vector<code_line> & code,
				const robot & sensor_info) const;

		int read_hardware(int port_number, robot & hardware_package,
				const list<Unit *> & other_robots, run_error
				& error_out) const;

		run_error write_to_hardware(const int port_number, 
				const int parameter, robot & hardware_package,
				const list<Unit *> & other_robots,
				list<missile> & missiles, list<mine> & mines,
				vector<set<robot *> > & comms_lookup);

		run_error interrupt(int interrupt_number, robot & actor,
				vector<short> & memory, 
				const list<Unit *> & active_robots,
				vector<set<robot *> > & comms_lookup,
				const int game_clock, const int matchnum, 
				const int total_matches, const int prog_size,
				const coordinate arena_size);

			// Or should this be in the parser?
	//	int get_delay_of_command(const command cmd_in);

	public:

		CPU();
		void cache_consistency(const vector<code_line> & prog);
		// pstack because it's a pseudo-stack.
		// Returns true if there was an error, otherwise false.
		bool execute(const vector<code_line> & prog, 
				vector<short> & memory,
				vector<short> & robot_pstack, 
				vector<int> & numeric_jump_table,
				vector<int> & alnum_jump_table, 
				robot & shell,
				const list<Unit *> & active_robots, 
				list<missile> & missiles,
				list<mine> & mines, 
				vector<set<robot *> > & comms_lookup,
				const int matchnum, const int total_matches,
				const coordinate arena_size,
				run_error & error_out);

		// Penalty propagation methods, so that we can avoid calling
		// execute() more times than are necessary. The idea is to peek
		// at the penalty and clear it, then subtract it from the cycles
		// remaining in one go instead of decrementing it by one inside
		// the CPU multiple times.
		int get_penalty() const { return(penalty); }
		void clear_macropenalty() { set_literal_penalty(0); }
		void decrement_macropenalty(int how_much) { penalty = max(0, 
				penalty - how_much); }

		// This decreases the penalty (only macro) by whatever's less
		// of maximum_allowed and the actual penalty we have 
		// accumulated. In concert with corelogic, this lets us emulate
		// the differing CPU speed without having to decrease penalty
		// one at a time.
		int withdraw_penalty(int maximum_allowed);

		void zero_penalty(); // Clears both macro and micro.

		int get_ip() const { return(ip); }
		int get_oldip() const { return(oldip); }

};

CPU::CPU() {
	ip = 0;
	penalty = 0;
	micropenalty = 0;
	cached = false;
	all_consistent = false;
}

void CPU::set_ip(int new_ip, int prog_size) {
	if (new_ip >= prog_size) ip = 0;
	else ip = new_ip;
}

void CPU::set_literal_penalty(int new_penalty) {
	penalty = new_penalty;
}

void CPU::set_penalty(command command_type, int a_field_direct) {
	assert(penalty == 0);
	penalty = pbalance.get_penalty(command_type, a_field_direct);

	// I'm not sure if there should be an ELSE micropenalty = 0 here.
	// What Would ATR2 Do? (It resets. Or, hmm, not easy to say, as there
	// seems to be a bug with the "increase CPU cycle after 20 penalty 0
	// ops" in that it doesn't take #time into account.)
	// Also beware since it's called twice from execute.. Not anymore!
	if (penalty == 0) {
		if (++micropenalty >= micropenalty_limit) {
			micropenalty = 0;
			penalty = 1;
		}
	} else micropenalty = 0;
}

// Flags spec: Bits 15 to 4 aren't touched
// Bit 3: on if both operands are zero
// Bit 2: on if operand 1 > operand 1
// Bit 1: on if operand 1 < operand 2
// Bit 0: on if both are equal
// All comparisons are signed.

// The quickest way to do a jump, I think, would be to have a table of
// IP (instruction pointer) values for each label. This can't be a hash (too
// slow), so we just give each !label a number (resolve) and then rewrite the
// code when we compile.

// Ideas for the code: a_field and b_field are ints. If they are not (bool
// reference is true), resolve reference and call. Only problem with this is
// if we're going to write to something. We could take the other route (write 
// values to a temporary location then reference it; handles read-only 
// gracefully), but that'd be slow. So we'll need a map or function where for 
// each command, it says if it writes to the a-field, etc.

// I think this can work!

// Jumps seem to be translated to absolute references.
// The "stack" is not really a stack, the sp just points to within an array.
// Hm, wonder if we can overflow it. We can read from out of bounds, but not
// write.

bool CPU::a_field_must_be_rw(const command cmd_in) const {
	// These commands write to the a-field, and so must all be V (writable):
	// ADD SUB INC DEC SHL SHR ROL ROR SAL SAR NEG OR AND XOR NOT MPY DIV
	// MOD XCHG MOV LOC GET POP.
	switch(cmd_in) {
		case CMD_ADD:	case CMD_SUB:	case CMD_INC:	case CMD_DEC:
		case CMD_SHL:	case CMD_SHR:	case CMD_ROL:	case CMD_ROR:
		case CMD_SAL:	case CMD_SAR:	case CMD_NEG:	case CMD_OR:
		case CMD_AND:	case CMD_XOR:	case CMD_NOT:	case CMD_MPY:
		case CMD_DIV:	case CMD_MOD:	case CMD_XCHG:	case CMD_MOV:
		case CMD_LOC:	case CMD_GET:	case CMD_POP:
			return(true);
		default: return(false);
	}
}

bool CPU::b_field_must_be_rw(const command cmd_in) const {
	// The following do something with the b-field.
	// IPO (result goes into b-field ref)
	// XCHG
	// LOC (returns address, i.e LEA; but note that r/o addresses are also
	// 	permitted)
	
	//return(cmd_in == CMD_IPO || cmd_in == CMD_XCHG || cmd_in == CMD_LOC);
	switch(cmd_in) {
		case CMD_IPO: case CMD_XCHG: case CMD_LOC: return(true);
		default: return(false);
	}
}

bool CPU::jumps_to_label(const command cmd_in) const {
	// CALL JMP JLS JGR JAE JLE JNE JE JZ JNZ LOOP
	// Switch is barely faster.
	
	switch(cmd_in) {
		case CMD_CALL: case CMD_JMP: case CMD_JLS: case CMD_JGR:
		case CMD_JAE: case CMD_JLE: case CMD_JNE: case CMD_JE:
		case CMD_JZ: case CMD_JNZ: case CMD_LOOP: return(true);
		default: return(false);
	}
}

bool CPU::jumps_to_label(const field_entry & in) const {
	return(jumps_to_label((command)in.value));
}

// Determine how many times a certain operand indirects, for handling things
// like [ax] (which is really [@65]).
int CPU::get_reference_count(const field_entry & fe) const {
	if (!fe.modifier.is_indirect_one && !fe.modifier.is_indirect_two)
		return(0);
	if (fe.modifier.is_indirect_one || fe.modifier.is_indirect_two)
		return(1);
	return(2);
}

bool CPU::consistent(const command opcode, const code_line & cmd_line) const {
	// Check that if the command demands that a field is writable, that
	// field actually points to a reference.
	
	// BLUESKY: Do something with write ops to privileged memory (accuracy
	// etc). ATR2 does nothing, thus hiding bugs (I think the settings get
	// re-set each cycle).
	
	// B-field first is slightly faster.. but why am I optimizing this?
	// I'm going to move it outside of the loop anyway.. eventually, for
	// code that doesn't reference memory.

	if (get_reference_count(cmd_line.get_b_field()) == 0 &&
			b_field_must_be_rw(opcode))
		return(false);

	if (get_reference_count(cmd_line.get_a_field()) == 0 &&
			a_field_must_be_rw(opcode))
		return(false);

	return(true);
}

bool CPU::consistent(const code_line & cmd_line) const {
	return(consistent((command)cmd_line.get_opcode().value, cmd_line));
}

// Returns the last address. If the reference count is 0, the input isn't an
// address, and we return -1. If the count is 1, it's just the input, and if
// 2 or higher (max 2 in original ATR2), then we get somewhat tangled.
// -2 means out of bounds.

int CPU::resolve_reference(const field_entry & argument,
		const vector<short> & memory,
		const vector<code_line> & program,
		const int code_size, const robot & shell) const {

	int reference_count = get_reference_count(argument);

	if (reference_count == 0) return(-1);

	int address = argument.value;
	
	while (reference_count > 0) {
		// Heuristic.
		if (!accessible_memory(address, memory, code_size, false))
			return(-2);
		//cout << "RR: address " << address << "-> ";
		if (--reference_count > 0)
			address = access_memory(address, memory, program,
					shell);
		//cout << address << endl;
	}

	// Hurtsies!
	if (!accessible_memory(address, memory, code_size, false)) return(-2);

	return(address);
}

// Get the address of a jump. Returns -1 if this isn't a jump, -2 if the
// jump is invalid (points outside code, jump table, or there was no label
// by that name), -3 if the jump is an alnum and invalid, else address.
// If force_n is true, then we treat literals as numeric jumps. This is the
// expected treatment for JE, JMP, etc..
// Note: Do not use operand when getting the destination! Instead use the
// supplied dereferenced integer, so things like jmp ax works.
int CPU::dereference_jump(const field_entry & operand, const int direct,
		const vector<int> & numeric_table,
		const vector<int> & alnum_table, int prog_size,
		bool force_n) const {

	int candidate = -1; // default case fail

	if (!operand.modifier.is_n_jump && 
			!operand.modifier.is_a_jump && !force_n)
		return(-1); // Not a jump

	int loc = direct;

	if (operand.modifier.is_a_jump) {
		if (loc < 0 || loc >= (int)alnum_table.size())
			return(-3);
		candidate = alnum_table[loc];
	} else {
		if (operand.modifier.is_n_jump || force_n) {
			if (loc < 0 || loc >= (int)numeric_table.size())
				return(-2);

			candidate = numeric_table[loc];
		}
	}

	// Final check that the jump actually falls within boundaries. This
	// shouldn't trigger in error (it means something's wrong with the
	// compiler), but just to be safe...
	
	// (Exception: if there's no jump corresponding to this label,
	//  merely inform the user.)
	if (candidate < 0)
		return(-2);
	
	assert(candidate >= 0 && candidate < prog_size);
	
	return(candidate);
}

// This returns true if we can read from that location, otherwise false.
inline bool CPU::accessible_memory(int location, const vector<short> & memory,
		const int code_size, bool write) const {

	if (write)
		return(location >= 0 && location < 1024);

	if (location < 0) return(false); // Certainly not!
	if (location < 1024) 
		return (location < memory.size());
	else
		return (((location - 1024) >> 2) < code_size);
}

// Shunt for accessing ROM and sensor memory. If the memory location is within
// a certain area, just access it directly, but if not, substitute the
// appropriate getting procedure.
// I think ATR2 does it "push" instead of "pull", since if we lose lock, we
// keep the old accuracy value. The nice thing about push is that we don't have
// to pass the code itself, but the bad thing is we update all the irrelevant
// variables when something changes. (Our Robot Utopia will require pull.)
//
// Yes, it does use push. The disadvantage of that is that you can overwrite
// things like "ID of last robot scanned" - it won't change until the next scan.
// Pull seems more consistent, but it does break compatibility (technically).
int CPU::access_memory(int location, const vector<short> & memory,
		const vector<code_line> & code, 
		const robot & sensor_info) const {

	// Is it an access on ROM?
	if (location >= 1024) {
		// Yes, find the right line and field type, then extract the
		// raw data from there and output it.

		location -= 1024;
		int line = location >> 2; // Four microcode entries to a line
		if (line >= code.size()) return(0);

		switch(location & 3) {
			case 0: return(code[line].get_opcode_directly());
			case 1: return(code[line].get_modifier_directly());
			case 2: return(code[line].get_a_field_directly());
			case 3: return(code[line].get_b_field_directly());
		}
	}

	// If it wasn't >= 1024, we fall through to here. It can either be a
	// communications queue shunt (512..767), a regular memory access
	// (anything > 13), or a special memory entry (dspd, etc).
	
	// Communications queue shunt
	// To be compatible with ATR2, we output from memory unless there's
	// been a comm message. This lets communications-less robots do "put
	// -2, 603" and the likes, while retaining the weakness where one can
	// spam them with ridiculous communication messages to overwrite the
	// table.
	int comms_start = 512, comms_end = 767;

	if (location >= comms_start && location <= comms_end) {
		int relative_message = location - comms_start;
		if (sensor_info.comms_queue.get_num_accessible_messages()
				> relative_message)
			return(sensor_info.comms_queue.get_at_location(
						relative_message));
	}

	// Ordinary access
	if (location > 13 && (size_t)location < memory.size()) 
		return(memory[location]);

	// So it was neither? Must be a special memory location.
	switch(location) {
		// @0: dspd: Desired speed (throttle) we're trying to achieve
		case 0:	return(round(sensor_info.get_desired_throttle()*100));

		// @1: Dhd: Desired heading
		// Sign adjustment isn't needed; we already return 0-255.
		case 1: return(round(sensor_info.get_desired_heading()));

		// @2: tpos: Current turret offset
		case 2: return(quant_hexangle(sensor_info.turret_heading -
						sensor_info.get_heading()));

		// @3: acc: Accuracy value from last scan
		case 3: return(sensor_info.get_accuracy(false));

		// @4: Temporary space from xchg (forced to 0 since we aren't
		// using it)
		case 4: return(0);

		// @5: ID of last target scanned (what kind of ID?
		// Transponder, apparently.)
		case 5: return(sensor_info.get_last_target_ID(true));

		// @6: Relative heading of last target scanned
		// Assumes 0-255 (i.e, not -128 to 128)
		// This is actually relative to the turret, not to ourselves.
		case 6: return(quant_hexangle(sensor_info.
						 get_last_target_heading()
					 - sensor_info.get_scan_center(true)));

		// @7: Throttle of last target scanned
		case 7: return(round(100 * sensor_info.
						get_last_target_throttle()));

		// @8: Collision count.
		case 8: return(sensor_info.get_num_crashes());

		// @9: Meters travelled. 15 bits used.
		case 9: return((int)round(sensor_info.get_length_traveled())
						& 32767);

		// Are these constants? Check later.
		// Doesn't seem to be so; and seems all of these have
		// origin 0.
		// I think the "base" is the position of the one we
		// last extracted, while the "end" is the position of
		// the last arrived message. Yup.
		// @10: Current base of comm queue (in memory loc.)
		case 10: return(sensor_info.comms_queue.get_last_read_pos());
		// @11: Current end-point of comm queue (in memory loc.)
		case 11: return(sensor_info.comms_queue.get_last_recv_pos());
		// @12: Nothing?
		default: return(0);
		// @13: Absolute speed of last target scanned, in cm/s (so
		// multiply by 100)
		case 13: return(round(sensor_info.get_last_target_abs_speed()
						 * 100));
	}
}

// Input ports
// Some of these require interaction with the robot and arena (e.g scanning),
// and so the robot is not a const.

// DONE: Differentiate errors when we try to access a port that doesn't exist
// and one that is of the wrong type (output for input or vice versa).
// Or return an error type.
// DONE: Read doc about whether relative headings are +/-128 or 0-255. We
// assume the latter here. (That is correct.)

int CPU::read_hardware(int port_number, robot & hardware_package,
		const list<Unit *> & other_robots, run_error & error) const {

	error = ERR_NOERR;

	switch(port_number) {
		// Port 1 - Returns throttle
		case 1: return(round(hardware_package.get_throttle() * 100));
		// Port 2 - Returns heat (0 - 500)
		case 2:	return(round(hardware_package.get_heat() * 500));
		// Port 3 - Returns heading (0 - 255)
		case 3: return(round(hardware_package.get_heading()));
		// Port 4 - Returns turret offset (0 - 255)
		// 		Note the inner round() which is required so that
		// 		the heading is relative to what we get out at
		// 		port 3.
		case 4: return(quant_hexangle(hardware_package.turret_heading
					- round(hardware_package.
						get_heading())));
		// Port 5 - Returns absolute turret heading
		case 5: return(hardware_package.turret_heading);
		// Port 6 - Returns armor level (0 - 100)
		case 6: return(round(hardware_package.get_armor() * 100));
		// Port 7 - Scans and returns range to target in arc (or maxint)
		case 7: hardware_package.do_scan(other_robots);
			return(hardware_package.get_scan_dist(false));
		// Port 8 - Returns accuracy of last scan ( a waste )
		case 8: return(hardware_package.get_accuracy(true));
		// Port 9 - Returns range to nearest target [NO INFRASTRUCTURE]
		case 9: return(hardware_package.do_radar(other_robots));
		// Port 10 - Returns random number (2^16 mapped to signed short)
		case 10: return(hardware_package.usrand());
		// Ports 11 to 15 inclusive: Write-only ports.
		case 11: case 12: case 13: case 14: case 15:
			 error = ERR_WRITE_ONLY_PORT;
			 return(-1);
		// Port 16 - Returns noisy heading to nearest target (sonar)
		case 16: return(hardware_package.do_sonar(other_robots));
		// Port 17 - Returns scanarc span
		case 17: return(hardware_package.get_scan_span(true));
		// Port 18 - Returns overburn status
		case 18: if (hardware_package.is_overburning()) return(1);
				 else return(0);
		// Port 19 - Returns transponder ID
		case 19: return(hardware_package.get_ID());
		// Port 20 - Returns heat shutdown level
		case 20: return(round(500 * hardware_package.
						 get_upper_shutdown_temp()));
		// Port 21 - Returns com-channel #
		case 21: return(hardware_package.get_communications_channel());
		// Port 22 - Returns mines left in storage
		case 22: return(hardware_package.get_mines_available());
		// Port 23 - Returns mines left in arena
		case 23: return(hardware_package.get_mines_deployed());
		// Port 24 - Returns shields on/off.
		case 24: if (hardware_package.is_shielded()) return(1);
				 else return(0);
		default: error = ERR_INVALID_PORT;
			 return(-1); // Error; either read-only or no such port
	}
}

// Output port
run_error CPU::write_to_hardware(const int port_number, const int parameter, 
		robot & hardware_package,
		const list<Unit *> & other_robots,
		list<missile> & missiles, list<mine> & mines,
		vector<set<robot *> > & comms_lookup) {

	// Do range checks on set throttle, set shutdown limit,
	// etc.? ATR2 doesn't. Nah, because they aren't truly errors,
	// just warnings.

	switch(port_number) {
		// 1 to 10: Read-only ports.
		case 1: case 2: case 3: case 4: case 5: case 6: case 7:
		case 8: case 9: case 10:
			return(ERR_READ_ONLY_PORT);
		// 11: Set throttle (-75 - 100), but not explicitly
		case 11:
			//cout << "Init set throttle: " << parameter << " " << parameter / 100.0 << endl;
			hardware_package.set_desired_throttle(parameter / 
					 100.0);
			return(ERR_NOERR);
		// 12: Rotate turret (cumulative)
		case 12: hardware_package.turret_heading = hexangle(
					 hardware_package. turret_heading + 
					 parameter);
			 return(ERR_NOERR);
		// 13: Aim turret (set turret to body angle + x)
		// But what if x < -256? DONE: Handle that.
		case 13: hardware_package.turret_heading = (int)round(hexangle(
						 parameter + hardware_package.
						 get_heading()));
			 return(ERR_NOERR);
		// 14: Turn x number of hexdegrees
		// DONE: Check if this is wrt desired_heading or real heading.)
		// 	Based on "thd", whatever that is. ram[1] is thd, so
		//	it's desired heading.
		// Maybe an angle class would be better?
		case 14: hardware_package.set_desired_heading(normalize_hex(
						 hardware_package.
						 get_desired_heading(), 
						 parameter)); 
			 return(ERR_NOERR);
		// 15: Fire with x in +/- 4 adjustment
		case 15: hardware_package.fire(missiles, parameter);
			 return(ERR_NOERR);
		// 16: Read-only port
		case 16: return(ERR_READ_ONLY_PORT);
		// 17: Set scan-arc span (Max 64, min 0) Error if not?
		case 17: if (parameter >= 0 && parameter <= 64)
				 hardware_package.set_scan_span(parameter);
			 return(ERR_NOERR);
		// 18: Set overburn status (0 = off, else = on)
		case 18: hardware_package.set_overburn(parameter != 0);
			 return(ERR_NOERR);
		// 19: Set transponder ID
		case 19: hardware_package.set_ID(parameter);
			 return(ERR_NOERR);
		// 20: Set shutdown heat level (0-500)
		case 20: if (parameter >= 0)
				 hardware_package.set_shutdown_temp(parameter / 
						 500.0);
			 return(ERR_NOERR);
		// 21: Set comm channel #
		case 21: hardware_package.set_communications_channel(parameter,
					 comms_lookup);
			 return(ERR_NOERR);
		// 22: Lay a mine. Return something if none are left? Hm?
		// DONE: Check if argument defines radius.
		case 22: if(hardware_package.deploy_mine(mines, parameter))
				 return(ERR_NOERR);
			 else	 return(ERR_NOMINES);
		// 23: Blow up all laid mines
		// For each mine, if it's ours, blow it up.
		case 23: { for(list<mine>::iterator p = mines.begin(); 
					 p != mines.end(); ++p) 
				 if (p->layer_UID() == hardware_package.
						 get_UID())
					 p->explode();
			 return(ERR_NOERR); }
		// 24: Turn shield on/off
		case 24: if (hardware_package.get_shield_type() == 0)
				 return(ERR_NOSHIELD);
			 hardware_package.set_shields(parameter != 0); 
			 return(ERR_NOERR);
		default: return(ERR_INVALID_PORT); // See read_hardware
	};
}

// Game_clock is in cycles.
// Perhaps return error value here.

run_error CPU::interrupt(int interrupt_number, robot & actor, 
		vector<short> & memory, const list<Unit *> & active_robots,
		vector<set<robot *> > & comms_lookup,
		const int game_clock, const int matchnum, 
		const int total_matches, const int prog_size, 
		const coordinate arena_size) {

	coordinate destpos;

	// DONE: As above, some way of signaling error.
	
	switch(interrupt_number) {
		// 0: Blow yourself up (inflict max damage)
		case 0: actor.inflict_damage(false, 1.0, actor.get_UID());
			return(ERR_NOERR);
		// 1: Reset robot program
		// (What does this mean? Only the program? Program and memory?
		//  Do we come to a halt when we reset? Probably do this by
		//  have robot and CPU constructors call a function, then just
		//  call those ourselves with data we need to preserve.)
		case 1: actor.reset_software(memory); // Should bot know about mem? Not really.
			zero_penalty();		// See robot::reset_software.
			set_ip(1, prog_size);	// Reset IP state
			return(ERR_NOERR);
		// 2: Set ex, fx, according to xpos, ypos
		case 2: memory[REG_EX] = actor.get_pos().x;
			memory[REG_FX] = actor.get_pos().y;
			return(ERR_NOERR);
		// 3: Set keepshift (0 off, else on; from ax)
		case 3: actor.set_keepshift(memory[REG_AX] != 0);
			return(ERR_NOERR);
		// 4: Set overburn (ditto)
		case 4: actor.set_overburn(memory[REG_AX] != 0);
			return(ERR_NOERR);
		// 5: Return robot transponder ID (into FX)
		case 5: memory[REG_FX] = actor.get_ID();
			return(ERR_NOERR);
		// 6: Return game clock in EX:FX
		case 6:	memory[REG_EX] = (short)(game_clock >> 16);
			memory[REG_FX] = (short)game_clock;
			return(ERR_NOERR);
		// 7: Given EX,FX (x,y), angle (from ourselves) into AX
		// DONE: Clamp coordinate to arena values.
		case 7: destpos = coordinate(memory[REG_EX], memory[REG_FX]);
			destpos = destpos - actor.get_pos();
			// Clamp
			destpos.x = min(arena_size.x - 1, max(0.0, destpos.x));
			destpos.y = min(arena_size.y - 1, max(0.0, destpos.y));
			
			memory[REG_AX] = round(radian_to_hex(atan2(destpos.y,
							destpos.x)));
			return(ERR_NOERR);
		// 8: Return transponder of last robot scanned (in FX)
		// Perhaps return error if there's been no scanning?
		case 8: memory[REG_FX] = actor.get_last_target_ID(true);
			return(ERR_NOERR);
		// 9: Return dir/throttle info of last robot scanned
		// EX = dir, FX = throttle
		// (I assume "dir" means relative heading)
		// Code taken from memory[6].
		// Not so sure if EX's right here. Seems to be.
		case 9: memory[REG_EX] = normalize_hex(actor.
					get_last_target_heading(), -actor.
					get_scan_body_heading(true));
			memory[REG_FX] = round(actor.get_last_target_throttle()
					* 100);
			return(ERR_NOERR);
		// 10: Return meta-round info
		// 	DX: # robots active; EX: Match #, FX: Total # matches
		case 10: memory[REG_DX] = active_robots.size();
			 memory[REG_EX] = matchnum;
			 memory[REG_FX] = total_matches;
			 return(ERR_NOERR);
		// 11: Return time-since-hurt info.
		// Will require modification of robot, but shouldn't be too hard
		// Just add a counter, set, get, mod inflict_damage
		// But we should also check that very peripheral mine explosions
		// don't register as hurting with strength 1e-20. Quick hack
		// would be to only trip when more than 1% damage has been done
		// since last trip, but eh. This is actually done in the
		// register_blast function.
		// DX: Absolute robot speed in cm/s (thus the factor of 100), 
		// EX: cycles since last hurt,
		// FX: cycles since last shot hit someone.
		case 11: 
			 memory[REG_DX] = round(actor.get_absolute_speed() 
					 * 100);
			 memory[REG_EX] = round(actor.get_time() - 
				 actor.get_last_damaged_at());
			 memory[REG_FX] = round(actor.get_time() - actor.
				 get_last_hit_other_at());
			 return(ERR_NOERR);
		// 12: Return collision count
		case 12: memory[REG_FX] = actor.get_num_crashes();
			 return(ERR_NOERR);
		// 13: Reset collision count
		case 13: actor.reset_crash_count();
			 return(ERR_NOERR);
		// 14: Transmit data in AX on current channel
		case 14: actor.transmit(memory[REG_AX], comms_lookup);
			 return(ERR_NOERR);
		// 15: Put next com-queue item into FX
		// Perhaps return false if there's nothing there to get?
		case 15: if (actor.get_last_message((unsigned short &) 
					 memory[REG_FX]))
				 return(ERR_NOERR);
			 else	 return(ERR_COMQUEUE_EMPTY);
		// 16: Return the amount of data in queue
		case 16: memory[REG_FX] = actor.get_num_waiting_messages();
			 return(ERR_NOERR);
		// 17: Empty com. queue
		// Will require modification to comms and passthrough through
		// robot. (Just set number read to number recv'd)
		case 17: actor.null_queue();
			 return(ERR_NOERR);
		// 18: Return #kills, #deaths
		// DX: Kill count for all rounds, EX: Kill count for this round,
		// FX: Number of times we've died.
		// EX not implemented. Add "local_killcount" inaccessible from
		// constructor, starts at 0, to robot, then alter record_kill.
		case 18: memory[REG_DX] = actor.get_all_kills();
			 memory[REG_EX] = actor.get_local_kills();
			 memory[REG_FX] = actor.get_all_deaths();
			 return(ERR_NOERR);
		// 19: Set meters variable to 0 (Odometers - travel count)
		case 19: actor.zero_odometer();
			 return(ERR_NOERR);
		default: return(ERR_INVALID_INT); // no change in vars
	}
}

void CPU::cache_consistency(const vector<code_line> & prog) {

	all_consistent = true;

	// Nothing can change whether a given line is consistent or not,
	// since consistency is just whether there's the right number of
	// indirects for the command, and the program is read-only.
	for (int counter = 0; counter < prog.size() && all_consistent; 
			++counter) {
		// Jumps don't qualify; the CPU aborts before the consistency
		// check on those anyway.
		if (!(prog[counter].get_opcode().modifier.is_n_jump ||
				prog[counter].get_opcode().modifier.is_a_jump))
			all_consistent = consistent(prog[counter]);

		// If the opcode field is an indirect reference, then we have
		// no idea of what opcode will be at the location in question,
		// and since it may be inconsistent, we must break the cache.
		if (get_reference_count(prog[counter].get_opcode()) > 0)
			all_consistent = false;
	}

	if (all_consistent)
		cached = true;

	// DEBUG
	/*cout << "CPU " << (uint64_t)this << " reporting: status: all_consistent: ";
	if (all_consistent) cout << "true, "; else cout << "false, ";
	cout << "cached: ";
	if (cached) cout << "true"; else cout << "false";
	cout << endl;*/
}


// ----------------------- Execution ---------------

// DONE: Simplify stack management functions. Perhaps by having a fakestack
// class? Rolling up robot_pstack into memory would seem to make for a happy
// fast critter, but only improves the runtime by 2%.
// Now uses boolean because otherwise ERROR 0 type statements shadow ERR_NOERR
// and thus don't get reported.
bool CPU::execute(const vector<code_line> & prog, vector<short> & memory,
		vector<short> & robot_pstack, vector<int> & numeric_jump_table,
		vector<int> & alnum_jump_table, robot & shell, 
		const list<Unit *> & active_robots, list<missile> & missiles, 
		list<mine> & mines, vector<set<robot *> > & comms_lookup,
		const int matchnum, const int total_matches, const coordinate
		arena_size, run_error & error_out) {

	// If we have enabled penalty and the penalty counter is above zero,
	// decrement and see if it becomes zero. If not, the robot's still
	// executing, so don't do anything.

	//cout << "Execute: Penalty is " << penalty << endl;

	//cout << "Penalty : " << penalty << " ";

	if (penalty > 0) {
		--penalty;
		return(true);
	}
	
	int prog_size = prog.size();

	// Before we do anything else, set our new IP so that even if we
	// abort with some error, the next cycle will start on another command.
	// (Also, making an error wastes as many cycles as the command would
	// usually take.)
	// Oldip is used as our IP (since we've incremented the ip itself),
	// and for stack trickery with regards to CALL and RET.
	
	oldip = ip;
	set_ip(oldip+1, prog_size);

	// HOTSPOT: Allocation takes too long time!
	code_line our_line = prog[oldip];
	//cout << "Proceeding with line at IP " << oldip << endl;

	// If we've encountered a label, skip early; this costs nothing.
	if (our_line.get_opcode().modifier.is_n_jump || our_line.get_opcode().
			modifier.is_a_jump) {
		set_literal_penalty(0);
	//	cout << "That's a label. Ignore it." << endl;
		return(true);
	}

	// Next, we have to check whether the a- and b-fields are sane with
	// regards to the command (no writing to a literal integer).
	// To do this, we first dereference the opcode (in case of self-
	// modifying code), and then check consistency.
	
	int opcode_address = resolve_reference(our_line.get_opcode(), memory,
			prog, prog_size, shell);
	if (opcode_address == -2) {
		error_out = ERR_CANT_REF_MEM;
		return(false);
	}
	// XXX: Possible bottleneck?
	
	if (opcode_address != -1) { // If it was a reference, dereference.
		field_entry deref;
		deref.value = access_memory(opcode_address, memory, prog,
				shell);
		our_line.set_opcode(deref);
	}

	// Do some sanity checks on the a- and b-field. If we're trying to write
	// to a literal, abort!
	
	// Check_jump is true here because it's faster and we already
	// bailed out on jump earlier.
	bool is_consistent;
	if (cached && all_consistent) 
		is_consistent = true;
	else	is_consistent = consistent(our_line);

	if (!is_consistent) {
		// We still pay the price for the command, even if it fizzled.
		// Same reasoning for the set_penalties below; we have to
		// replicate it instead of setting and resetting since micro-
		// penalties come into play.
		set_penalty((command)our_line.get_opcode().value, 0);
		error_out = ERR_READONLY;
		return(false);
	}

	// Okay, we've passed. Get addresses of a- and b-fields, too, if there
	// are any, and resolve them.
	short a_field_direct, a_field_indirect, b_field_direct, 
	      b_field_indirect;

	a_field_indirect = resolve_reference(our_line.get_a_field(), memory,
			prog, prog_size, shell);
	b_field_indirect = resolve_reference(our_line.get_b_field(), memory,
			prog, prog_size, shell);

	// Were any indirect, but pointed out of memory?
	if (a_field_indirect == -2 || b_field_indirect == -2) {
		set_penalty((command)our_line.get_opcode().value, 0);
		error_out = ERR_CANT_REF_MEM;
		return(false);
	}

	// Dereference if required, and check if the reference goes to somewhere
	// we can't write to if the instruction requires us to write.
	
	if (a_field_indirect == -1)
		a_field_direct = our_line.get_a_field().value;
	else {
		a_field_direct = access_memory(a_field_indirect, memory, prog,
				shell);

		if (a_field_must_be_rw((command)our_line.get_opcode().value) &&
				!accessible_memory(a_field_indirect, memory,
					prog_size, true)) {
			set_penalty((command)our_line.get_opcode().value,
					a_field_direct);
			error_out = ERR_READONLY;
			return(false);
		}
	}

	if (b_field_indirect == -1)
		b_field_direct = our_line.get_b_field().value;
	else {	
		if (b_field_must_be_rw((command)our_line.get_opcode().value) &&
				!accessible_memory (b_field_indirect, memory, 
					prog_size, true)) {
			set_penalty((command)our_line.get_opcode().value,
					a_field_direct);
			error_out = ERR_READONLY;
			return(false);
		}

		b_field_direct = access_memory(b_field_indirect, memory, prog,
			shell);
	}

	// Dereference jumps. These dereferences move into *_field_direct,
	// so that operators like "mov dx, !location" work as expected.
	// Incidentally, this makes JMP and JTL execute the exact same
	// instructions in the big switch table we'll get to soon.
	
	// (Does not support explicit microcode constructions of the form @: )

	int a_jump, b_jump;
	// Since nothing ever treats "opcode af, bf" as "opcode af, :bf",
	// force_n is false for b_jump.
	// Maybe this could be done outside execute -- after all, jumps don't
	// suddenly change locations. Ooh, that's what ATR2 does, and that's
	// why it gets in the microcode-as-jumps problem.
	bool is_resolv_jump = jumps_to_label(our_line.get_opcode());
	a_jump = dereference_jump(our_line.get_a_field(), a_field_direct,
			numeric_jump_table, alnum_jump_table,
			prog_size, is_resolv_jump);
	b_jump = dereference_jump(our_line.get_b_field(), b_field_direct,
			numeric_jump_table, alnum_jump_table,
			prog_size, false);

	// If any of these were successful, replace directs.
	// If it's just a jump, don't complain, merely store the error so that
	// we do complain if the jump's actually taken (like in ATR2).
	
	// Doesn't cover odd constructs like "JTL :label", but those are
	// unconditional and so we don't notice the difference.

	// Let's hope this doesn't kill our page concurrency.

	run_error jump_error = ERR_NOERR;

	if (a_jump == -2 || b_jump == -2) {
		if (!is_resolv_jump) {
			set_penalty((command)our_line.get_opcode().value, 0);
			error_out = ERR_NOLABEL;
			return(false);
		} else	jump_error = ERR_NOLABEL;
	}
	if (a_jump == -3 || b_jump == -3) {
		if (!is_resolv_jump) {
			set_penalty((command)our_line.get_opcode().value, 0);
			error_out = ERR_UNRESOLVED_TXT;
			return(false);
		} else	jump_error = ERR_NOLABEL;
	}

	if (a_jump != -1)
		a_field_direct = a_jump;
	if (b_jump != -1)
		b_field_direct = b_jump;

	// Update what we set as penalty to reflect the direct values we've now
	// divined.
	set_penalty((command)our_line.get_opcode().value, a_field_direct);

	// Finally, check what opcode we have and run the appropriate sequence.

	switch(our_line.get_opcode().value) {
		case CMD_NOP:
			break; // do nothing!
		case CMD_ADD:
			memory[a_field_indirect] = a_field_direct + 
				b_field_direct;
			break;
		case CMD_SUB:
			memory[a_field_indirect] = a_field_direct -
				b_field_direct;
			break;
		case CMD_INC:
			memory[a_field_indirect]++;
			break;
		case CMD_DEC:
			memory[a_field_indirect]--;
			break;
		case CMD_SAL:
			// Check these with negative b fields.
			memory[a_field_indirect] = 
				abs(abs(a_field_direct) << b_field_direct);
			if (a_field_direct < 0) 
				 memory[a_field_indirect] =
				-memory[a_field_indirect];
			break;
		case CMD_SAR:
			memory[a_field_indirect] = 
				abs(abs(a_field_direct) >> b_field_direct);
			if (a_field_direct < 0) 
				 memory[a_field_indirect] =
				-memory[a_field_indirect];
			break;
		case CMD_ROR:
			// check these. The char truncation poses no problem
			// because the memfield size is a power of 2^8.
			memory[a_field_indirect] = rotate_right(
					memory[a_field_indirect], 
					b_field_direct);
			break;
		case CMD_ROL:
			memory[a_field_indirect] = rotate_left(
					memory[a_field_indirect], 
					b_field_direct);
			break;
		case CMD_SHL:
			// Not really sure about these, will have to check later
			memory[a_field_indirect] <<= b_field_direct;
			break;
		case CMD_SHR:
			memory[a_field_indirect] >>= b_field_direct;
			break;
		case CMD_NEG:
			memory[a_field_indirect] = -memory[a_field_indirect];
			break;
		case CMD_OR:
			memory[a_field_indirect] |= b_field_direct;
			break;
		case CMD_AND:
			memory[a_field_indirect] &= b_field_direct;
			break;
		case CMD_NOT:
			// also have to verify this
			memory[a_field_indirect] = !b_field_direct;
			break;
		case CMD_XOR:
			memory[a_field_indirect] ^= b_field_direct;
			break;
		case CMD_MPY:
			memory[a_field_indirect] *= b_field_direct;
			break;
		case CMD_DIV:
			if (b_field_direct == 0) {
				error_out = ERR_DIVIDE_ZERO;
				return(false);
			}
			memory[a_field_indirect] 
				/= b_field_direct; // rounds correctly
			break;
		case CMD_MOD:
			if (b_field_direct == 0) {
				error_out = ERR_DIVIDE_ZERO;
				return(false);
			}
			memory[a_field_indirect] %= b_field_direct;
			break;
		case CMD_XCHG:
			swap(memory[a_field_indirect], 
					memory[b_field_indirect]);
			break;
		case CMD_RET: {
			// pops the IP. Have to add a stack and check
			// what the IP actually is in "real" ATR2.
			// If we can't get anything, this sets IP to 0 and
			// gives the empty/full stack error.
			set_ip(0, prog_size);

			// First check that SP is well conditioned
			if (memory[REG_SP] - 1 < 0) {
				error_out = ERR_STACK_EMPTY;
				return(false);
			}

			if ((size_t)(memory[REG_SP] -1) >= robot_pstack.size()){
				error_out = ERR_STACK_FULL;
				return(false);
			}

			// Oh, I see; what happens in real ATR is probably that
			// it sets the IP unconditionally, then outside of the
			// function a bounds check is done and IP set to 0.
			// But then we should get errors even when it loops 
			// around.
			
			// Now we do that too.

			// The +1 is so we proceed from the location of the CALL
			int dest = robot_pstack[memory[REG_SP]-1] + 1;

			set_ip(dest, prog_size);
			memory[REG_SP]--;
			if (dest < 0 || dest >= prog_size) {
				error_out = ERR_OUT_OF_RANGE;
				return(false);
			}
			break; }
		case CMD_CALL:
			// a field direct is where to jump to
			// This is just a push ip followed by a jump.

			// By logic, if we can't push the location onto the
			// stack, we've got nothing to do inside the CALLed
			// function, and so should just continue onwards.
			// I'm not sure this is what ATR2 proper does, so
			// beware.

			// ATR2 doesn't push anything onto the stack if the
			// destination is not a jump.
			if (jump_error != ERR_NOERR) {
				error_out = jump_error;
				return(false);
			}

			if (memory[REG_SP] < 0) {
				error_out = ERR_STACK_EMPTY;
				return(false);
			}

			if ((size_t)memory[REG_SP] >= robot_pstack.size()) {
				error_out = ERR_STACK_FULL;
				return(false);
			}

			robot_pstack[memory[REG_SP]] = oldip;
			memory[REG_SP]++;
			set_ip(a_field_direct, prog_size);
			break;
		case CMD_JMP:
			if (jump_error != ERR_NOERR) {
				error_out = jump_error;
				return(false);
			}
			set_ip(a_field_direct, prog_size);
			break;
		case CMD_CMP:
			//cout << "Comparison: " << a_field_direct << " vs " << b_field_direct << endl;
			// Bit 3: on if both operands are zero	8
			// Bit 2: on if operand 1 > operand 1	4
			// Bit 1: on if operand 1 < operand 2	2
			// Bit 0: on if both are equal		1
			// First clear the flags
			memory[REG_FLAGS] &= 0xFFF0;
			// Then set them
			if (a_field_direct == 0 && b_field_direct == 0)
				memory[REG_FLAGS] |= 8;
			if (a_field_direct > b_field_direct)
				memory[REG_FLAGS] |= 4;
			if (a_field_direct < b_field_direct)
				memory[REG_FLAGS] |= 2;
			if (a_field_direct == b_field_direct)
				memory[REG_FLAGS] |= 1;
			break;
		// Bluesky: Do something with this cut-and-paste code; it's
		// ugly.
		case CMD_JLS:
			if ((memory[REG_FLAGS] & 2) != 0) 
				if (jump_error != ERR_NOERR) {
					error_out = jump_error;
					return(false);
				} else	set_ip(a_field_direct, prog_size);
			break;
		case CMD_JGR:
			if ((memory[REG_FLAGS] & 4) != 0) 
				if (jump_error != ERR_NOERR) {
					error_out = jump_error;
					return(false);
				} else	set_ip(a_field_direct, prog_size);
			break;
		case CMD_JAE:
			if ((memory[REG_FLAGS] & 5) != 0)
				if (jump_error != ERR_NOERR) {
					error_out = jump_error;
					return(false);
				} else	set_ip(a_field_direct, prog_size);
			break;
		case CMD_JNE:
			if ((memory[REG_FLAGS] & 1) == 0)
				if (jump_error != ERR_NOERR) {
					error_out = jump_error;
					return(false);
				} else	set_ip(a_field_direct, prog_size);
			break;
		case CMD_JE:
			if ((memory[REG_FLAGS] & 1) != 0)
				if (jump_error != ERR_NOERR) {
					error_out = jump_error;
					return(false);
				} else	set_ip(a_field_direct, prog_size);
			break;
		case CMD_JLE:
			if ((memory[REG_FLAGS] & 3) != 0)
				if (jump_error != ERR_NOERR) {
					error_out = jump_error;
					return(false);
				} else	set_ip(a_field_direct, prog_size);
			break;
		case CMD_JZ:
			if ((memory[REG_FLAGS] & 8) != 0)
				if (jump_error != ERR_NOERR) {
					error_out = jump_error;
					return(false);
				} else	set_ip(a_field_direct, prog_size);
			break;
		case CMD_JNZ:
			if ((memory[REG_FLAGS] & 8) == 0)
				if (jump_error != ERR_NOERR) {
					error_out = jump_error;
					return(false);
				} else	set_ip(a_field_direct, prog_size);
			break;
		case CMD_JTL:
			// Set the instruction pointer to the A-field.
			if (a_field_direct < 0 || a_field_direct >= prog_size){
				error_out = ERR_NOLABEL;
				return(false);
			}
			set_ip(a_field_direct, prog_size);
			break;
		case CMD_DO:
			// Sets CX to a-field
			memory[REG_CX] = a_field_direct;
			break;
		case CMD_LOOP:
			// Decrement CX. If CX > 0, go to label specified
			// by a-field
			--memory[REG_CX];
			if (memory[REG_CX] > 0) {
				if (jump_error != ERR_NOERR) {
					error_out = jump_error;
					return(false);
				} else	set_ip(a_field_direct, prog_size);
			}
			break;
		case CMD_TEST: {
			// if a field and b field == b field, then set equals
			// flag. If a field and b field is zero, then set zero
			// flag.
			short k = a_field_direct & b_field_direct;
			memory[REG_FLAGS] &= 0xFFF0;
			if (k == b_field_direct) 
				memory[REG_FLAGS] |= 1;
			if (k == 0)
				memory[REG_FLAGS] |= 8;
			break; }
		case CMD_MOV:
			memory[a_field_indirect] = b_field_direct;
			break;
		case CMD_LOC: // LEA-alike
			memory[a_field_indirect] = b_field_indirect;
			break;
		case CMD_GET:
			if (b_field_direct < 0 || b_field_direct >= 
					memory.size()) {
				error_out = ERR_CANT_REF_MEM;
				return(false);
			}
			memory[a_field_indirect] = 
				memory[b_field_direct];
			break;
		case CMD_PUT:
			if (b_field_direct < 0 || b_field_direct >=
					memory.size()) {
				error_out = ERR_CANT_REF_MEM;
				return(false);
			}
			memory[b_field_direct] = a_field_direct;
			break;
		case CMD_INT:
			error_out = interrupt(a_field_direct, shell, memory, 
					active_robots, comms_lookup, 
					shell.get_time(), matchnum, 
					total_matches, prog_size,
					arena_size);
			if (error_out != ERR_NOERR) return(false);
			break;
		case CMD_IPO:
			memory[b_field_indirect] = read_hardware(a_field_direct,
					shell, active_robots, error_out);
			if (error_out != ERR_NOERR) return(false);
			break;
		case CMD_OPO:
			error_out = write_to_hardware(a_field_direct, 
					b_field_direct, shell, active_robots, 
					missiles, mines, comms_lookup);
			if (error_out != ERR_NOERR) return(false);
			break;
		case CMD_DELAY:
			// No cheating here!
			// (Is it possible to loop endlessly with a DELAY 0
			//  in orig ATR2?)
			// Not required anymore, as the penalty manager
			// class handles this.
			//set_literal_penalty(max((short)1, a_field_direct));
			break;
		case CMD_PUSH:
			// Roll up with CALL into push/pop aux functions?
			if (memory[REG_SP] < 0) {
				error_out = ERR_STACK_EMPTY;
				return(false);
			}
			if ((size_t)memory[REG_SP] >= robot_pstack.size()) {
				error_out = ERR_STACK_FULL;
				return(false);
			}

			robot_pstack[memory[REG_SP]] = a_field_direct;
			memory[REG_SP]++;
			break;
		case CMD_POP: {
				      // Should this use read_memory?
			int memloc = memory[REG_SP] - 1;

			if (memloc < 0) {
				error_out = ERR_STACK_EMPTY;
				return(false);
			}

			if ((size_t) memloc >= robot_pstack.size()) {
				error_out = ERR_STACK_FULL;
				return(false);
			}

			memory[a_field_indirect] = robot_pstack[memloc];
			memory[REG_SP]--;
			break; }
		case CMD_ERR:
			error_out = (run_error)a_field_direct;
			return(false);
		default:
			error_out = ERR_UNKN_CMD;
			return(false);
	}

	return(true);
}

void CPU::zero_penalty() {
	penalty = 0;
	micropenalty = 0;
}

int CPU::withdraw_penalty(int maximum_allowed) {
	if (maximum_allowed < 0 || penalty <= 0) return(0);

	int actual_decrement = min(maximum_allowed, penalty);
	penalty -= actual_decrement;
	return(actual_decrement);
}

#endif
