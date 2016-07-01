#ifndef _KROB_CMDPARSE
#define _KROB_CMDPARSE

#include "prog_constants.h"
#include "errors.h"
#include "error_container.cc"
#include "code_line.cc"
#include "indirect.cc"
#include "../tools.cc"
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace std;

// But what of things like "JMP 3" meaning "JMP to label 3"? That's done in
// the interpreter.

// Note that we might not have needed disassemble (we could just refer to the
// source), but it's better this way.

typedef struct multimap<field_entry, string> RLookup;

class cmd_parse {

	private:
		map<string, field_entry> forward_lookup;
		RLookup reverse_lookup;
		// maybe reverse_port_lookup, reverse_int_lookup for a better
		// debugger? Nah, let's get this working first.

		void add_both_ways(const string symbol, signed short value,
				bool is_indirect);
		void add_both_ways(const string symbol, signed short value);

		string disassemble_single(const field_entry & input) const;

		void add_symbols();

	public:

		cmd_parse() { add_symbols(); }

		error_container lookup_single(bool strict, 
				const string to_check, field_entry & output,
				const vector<int> & numeric_jump_table,
				const vector<int> & alnum_jump_table);

		error_container get_line(bool strict, const string opcode, 
				const string a_field, const string b_field,
				code_line & dest, 
				const vector<int> & numeric_jump_table,
				const vector<int> & alnum_jump_table);
		error_container get_line_from_microcode(const vector<string> &
				tokenized_line, code_line & dest);

		string disassemble(const code_line & source) const;
};

void cmd_parse::add_both_ways(const string symbol, signed short value, 
		bool is_indirect) {

	field_entry to_add;
	to_add.value = value;
	to_add.modifier.is_indirect_one = is_indirect;
	to_add.modifier.is_indirect_two = false;
	to_add.modifier.is_n_jump = false;
	to_add.modifier.is_a_jump = false;

	// Memory leak here. But how?! Not anymore.
	reverse_lookup.insert(make_pair(to_add, symbol));
	forward_lookup.insert(make_pair(symbol, to_add));

	//forward_lookup[symbol] = to_add;
}

void cmd_parse::add_both_ways(const string symbol, signed short value) {
	add_both_ways(symbol, value, false);
}

// Add symbols so that we can do a good lookup later.
// To ATR2, everything's an integer. Internally, we use command, register, 
// port, and interrupt structures, but since any field might be an integer,
// (e.g "0 0 0 " is legal), we resolve first to int when compiling, then to 
// the appropriate type on the run.

void cmd_parse::add_symbols() {

	// Constants.
	add_both_ways("minint", -32768);
	add_both_ways("maxint", 32767);

	// Commands
	add_both_ways("nop", CMD_NOP);
	add_both_ways("add", CMD_ADD);
	add_both_ways("sub", CMD_SUB);
	add_both_ways("or", CMD_OR);
	add_both_ways("and", CMD_AND);
	add_both_ways("xor", CMD_XOR);
	add_both_ways("not", CMD_NOT);
	add_both_ways("mpy", CMD_MPY);
	add_both_ways("div", CMD_DIV);
	add_both_ways("mod", CMD_MOD);
	add_both_ways("ret", CMD_RET);
	add_both_ways("return", CMD_RET);
	add_both_ways("call", CMD_CALL);
	add_both_ways("gsb", CMD_CALL);
	add_both_ways("gosub", CMD_CALL);
	add_both_ways("jmp", CMD_JMP);
	add_both_ways("goto", CMD_JMP);
	add_both_ways("jls", CMD_JLS);
	add_both_ways("jb", CMD_JLS);
	add_both_ways("jgr", CMD_JGR);
	add_both_ways("ja", CMD_JGR);
	add_both_ways("jne", CMD_JNE);
	add_both_ways("je", CMD_JE);
	add_both_ways("jeq", CMD_JE);
	add_both_ways("xchg", CMD_XCHG);
	add_both_ways("swap", CMD_XCHG);
	add_both_ways("do", CMD_DO);
	add_both_ways("loop", CMD_LOOP);
	add_both_ways("cmp", CMD_CMP);
	add_both_ways("test", CMD_TEST);
	add_both_ways("mov", CMD_MOV);
	add_both_ways("set", CMD_MOV);
	add_both_ways("loc", CMD_LOC);
	add_both_ways("get", CMD_GET);
	add_both_ways("put", CMD_PUT);
	add_both_ways("int", CMD_INT);
	add_both_ways("ipo", CMD_IPO);
	add_both_ways("in", CMD_IPO);
	add_both_ways("opo", CMD_OPO);
	add_both_ways("out", CMD_OPO);
	add_both_ways("del", CMD_DELAY);
	add_both_ways("delay", CMD_DELAY);
	add_both_ways("push", CMD_PUSH);
	add_both_ways("pop", CMD_POP);
	add_both_ways("err", CMD_ERR);
	add_both_ways("error", CMD_ERR);
	add_both_ways("inc", CMD_INC);
	add_both_ways("dec", CMD_DEC);
	add_both_ways("shl", CMD_SHL);
	add_both_ways("shr", CMD_SHR);
	add_both_ways("rol", CMD_ROL);
	add_both_ways("ror", CMD_ROR);
	add_both_ways("jz", CMD_JZ);
	add_both_ways("jnz", CMD_JNZ);
	add_both_ways("jae", CMD_JAE);
	add_both_ways("jge", CMD_JAE);
	add_both_ways("jbe", CMD_JLE);
	add_both_ways("jle", CMD_JLE);
	add_both_ways("sal", CMD_SAL);
	add_both_ways("sar", CMD_SAR);
	add_both_ways("neg", CMD_NEG);
	add_both_ways("jtl", CMD_JTL);

	// Interrupts
	add_both_ways("i_destruct", I_DESTRUCT); 
	add_both_ways("i_reset", I_RESET);
	add_both_ways("i_locate", I_LOCATE);
	add_both_ways("i_keepshift", I_KEEPSHIFT);
	add_both_ways("i_overburn", I_OVERBURN);
	add_both_ways("i_id", I_ID);
	add_both_ways("i_timer", I_TIMER);
	add_both_ways("i_angle", I_ANGLE);
	add_both_ways("i_tid", I_TARGETID);
	add_both_ways("i_targetid", I_TARGETID);
	add_both_ways("i_targetinfo", I_TARGETINFO);
	add_both_ways("i_tinfo", I_TARGETINFO);
	add_both_ways("i_ginfo", I_GAMEINFO);
	add_both_ways("i_gameinfo", I_GAMEINFO);
	add_both_ways("i_rinfo", I_ROBOTINFO);
	add_both_ways("i_robotinfo", I_ROBOTINFO);
	add_both_ways("i_collisions", I_COLLISIONS);
	add_both_ways("i_resetcolcnt", I_RESETCOLCNT);
	add_both_ways("i_transmit", I_TRANSMIT);
	add_both_ways("i_receive", I_RECEIVE);
	add_both_ways("i_dataready", I_DATAREADY);
	add_both_ways("i_clearcom", I_CLEARCOM);
	add_both_ways("i_kills", I_KILLS);
	add_both_ways("i_deaths", I_KILLS);
	add_both_ways("i_clearmeters", I_CLEARMETERS);

	// Ports
	add_both_ways("p_spedometer", P_SPEEDOMETER); // [sic]
	add_both_ways("p_heat", P_HEAT);
	add_both_ways("p_compass", P_COMPASS);
	add_both_ways("p_turret_ofs", P_TURRET_OFS);
	add_both_ways("p_turret_abs", P_TURRET_ABS);
	add_both_ways("p_damage", P_DAMAGE);
	add_both_ways("p_armor", P_DAMAGE);
	add_both_ways("p_scan", P_SCAN);
	add_both_ways("p_accuracy", P_ACCURACY);
	add_both_ways("p_radar", P_RADAR);
	add_both_ways("p_random", P_RANDOM);
	add_both_ways("p_rand", P_RANDOM);
	add_both_ways("p_throttle", P_THROTTLE);
	add_both_ways("p_ofs_turret", P_TROTATE);
	add_both_ways("p_trotate", P_TROTATE);
	add_both_ways("p_abs_turret", P_TAIM);
	add_both_ways("p_taim", P_TAIM);
	add_both_ways("p_steering", P_STEERING);
	add_both_ways("p_weap", P_FIRE);
	add_both_ways("p_weapon", P_FIRE);
	add_both_ways("p_fire", P_FIRE);
	add_both_ways("p_sonar", P_SONAR);
	add_both_ways("p_arc", P_ARC);
	add_both_ways("p_scanarc", P_ARC);
	add_both_ways("p_overburn", P_OVERBURN);
	add_both_ways("p_transponder", P_TRANSPONDER);
	add_both_ways("p_shutdown", P_SHUTDOWN);
	add_both_ways("p_channel", P_CHANNEL);
	add_both_ways("p_minelayer", P_MINELAYER);
	add_both_ways("p_minetrigger", P_MINETRIGGER);
	add_both_ways("p_shield", P_SHIELD);
	add_both_ways("p_shields", P_SHIELD);

	// Registers (but note that these are @s)
	add_both_ways("flags", REG_FLAGS, true);
	add_both_ways("ax", REG_AX, true);
	add_both_ways("bx", REG_BX, true);
	add_both_ways("cx", REG_CX, true);
	add_both_ways("dx", REG_DX, true);
	add_both_ways("ex", REG_EX, true);
	add_both_ways("fx", REG_FX, true);
	add_both_ways("sp", REG_SP, true);
	add_both_ways("colcnt", 8, true);
	add_both_ways("meters", 9, true);
	add_both_ways("combase", 10, true);
	add_both_ways("comend", 11, true);
}

// Here we do a lookup of a single value, stripping the special symbols (:, 
// @, and []).
// DONE: Look up user-defined variables first, as per ATR2.
// DONE: Return compiler errors if it fails, not just false/true.
error_container cmd_parse::lookup_single(bool strict, string to_check, 
		field_entry & output, const vector<int> & numeric_jump_table, 
		const vector<int> & alnum_jump_table) {
	field_entry temp;
	to_check = lowercase(to_check);
	string full_name = to_check;

	// If it's an indirect reference (of either kind), remove the reference
	// symbols and set the appropriate bits.
	to_check = strip_indirects(to_check, temp, false);

	// If it's a : jump, and all numbers (needed for compat mode), and
	// actually in our jump table, then set the jump bit.
	
	if (*to_check.begin() == ':') {
		// Remove :
		to_check.erase(to_check.begin());

		// Test
		if (!is_integer(to_check, false))
			// Not an integer, thus not a valid label
			return(error_container(full_name, -1, 
						CER_INVALID_NLABEL));

		int jumploc = stoi(to_check);
		if (jumploc < 0 || jumploc >= (int)numeric_jump_table.size())
			// Out of bounds.
			return(error_container(full_name, -1, 
						CER_NLABEL_OORANGE));

		// If it wasn't declared earlier, can't jump there.
		if (numeric_jump_table[jumploc] == -1)
			return(error_container(full_name, -1, 
						CER_INVALID_NLABEL));

		temp.modifier.is_n_jump = true;
	} else {
		// Same, but with !jumps. This assumes that the rewrite routine
		// has been called to transform alphanumerics into numerals.
		if (*to_check.begin() == '!') {
			to_check.erase(to_check.begin());
			if (!is_integer(to_check, false))
				return(error_container(full_name, -1, 
							CER_XLBL_NOTFOUND));

			int jumploc = stoi(to_check);
			if (jumploc < 0 || jumploc >= (int)alnum_jump_table.
					size())
				return(error_container(full_name, -1, 
							CER_XLBL_OORANGE));
			if (alnum_jump_table[jumploc] == -1)
				return(error_container(full_name, -1,
							CER_XLBL_NOTFOUND));

			temp.modifier.is_a_jump = true;
		}
	}

	//cout << "DEBUG: After stripping: " << to_check << endl;

	// Now it should have been stripped of all the tags. Look it up.
	// If we don't get any hits, then if it's not an integer, abort.
	
	field_entry lookup;

	if (forward_lookup.find(to_check) == forward_lookup.end()) {
		// If it's not strict, and it's not an integer, and it has more
		// than one character, try removing the last. This fixes things
		// that ATR2 accepts among the tournament robots -- like "2@"
		// being reinterpreted as "2".
		if (!strict && !is_integer(to_check, true) && 
				to_check.size() > 1)
			to_check.resize(to_check.size()-1);

		//cout << "No match for " << to_check << ", checking number."
		//	<< "\t";
		if (!is_integer(to_check, true))
			// Invalid reference if it can't be found and isn't int
			return(error_container(full_name, -1, CER_INVALID_REF));
		else	lookup.value = stoi_generalized(to_check);
		//cout << lookup.value << endl;
	} else
		lookup = forward_lookup.find(to_check)->second;

	// Last checks: triple-referencing (@ax) is not permitted.
	if ((lookup.modifier.is_indirect_one && temp.modifier.is_indirect_one) 
			|| (lookup.modifier.is_indirect_two && temp.modifier.
				is_indirect_two) || (lookup.modifier.is_n_jump
					&& temp.modifier.is_n_jump) ||(
						lookup.modifier.is_a_jump &&
						temp.modifier.is_a_jump)) 
		return(error_container(full_name, -1, CER_INVALID_REF)); 

	// Otherwise, merge
	output.modifier.is_n_jump = lookup.modifier.is_n_jump | 
		temp.modifier.is_n_jump;
	output.modifier.is_a_jump = lookup.modifier.is_a_jump |
		temp.modifier.is_a_jump;
	output.value = lookup.value;
	output.modifier.is_indirect_one = lookup.modifier.is_indirect_one |
		temp.modifier.is_indirect_one;
	output.modifier.is_indirect_two = lookup.modifier.is_indirect_two |
		temp.modifier.is_indirect_two;

	return(error_container(CER_NOERR));
}

// This constructs a code_line based on text input. We assume that separators
// and comments have been removed.
// Handle case with four * microcodes. DONE, externally.
error_container cmd_parse::get_line(bool strict, const string opcode, 
		const string a_field, const string b_field, code_line & dest, 
		const vector<int> & numeric_jump_table,
		const vector<int> & alnum_jump_table) {

	field_entry op, a, b;

	error_container ret(CER_NOERR);

	// Look up opcodes, a-field, and b-field; break on error.
	ret = lookup_single(strict, opcode, op, numeric_jump_table, 
			alnum_jump_table);
	if (ret.error != CER_NOERR)	return(ret);
	ret = lookup_single(strict, a_field, a, numeric_jump_table, 
			alnum_jump_table);
	if (ret.error != CER_NOERR)	return(ret);
	ret = lookup_single(strict, b_field, b, numeric_jump_table, 
			alnum_jump_table);
	if (ret.error != CER_NOERR)	return(ret);

	//cout << "DEBUG: After lookups " << op.value << ", " << a.value << ", " << b.value << endl;

	dest.set_opcode(op);
	dest.set_a_field(a);
	dest.set_b_field(b);

	return(error_container(CER_NOERR));
}

error_container cmd_parse::get_line_from_microcode(const vector<string> &
		tokenized_line, code_line & dest) {
	// First of all, if it's not a microcode statement, outta here.
	
	// We don't use the full string, so don't construct it (as it takes
	// too much time here for each line)
	if (tokenized_line[0][0] != '*')
		return(error_container("N/A"/*full*/, -1, CER_MICRO_NONMICRO));
	if (tokenized_line.size() < 4)
		return(error_container("N/A"/*full*/, -1, CER_MICRO_NONMICRO));

	string full;
	for (size_t counter = 0; counter < tokenized_line.size(); ++counter)
		full += " " + tokenized_line[counter];

	// Merge if required. We do this virtually, by applying an offset,
	// since to do otherwise would require a copy of the vector.
	int offset = 0;
	if (tokenized_line[0] == "*")
		offset = 1;

	// Not quite right, but it'll do. (ATR2 doesn't give any error at all)
	if (tokenized_line.size() > (size_t)(4 + offset))
		return(error_container(full, -1, CER_MICRO_DENSE));

	// Okay, now remove the asterisks and check for integer-ness. If
	// these aren't all-int, outta here.
	string a = tokenized_line[0 + offset], b = tokenized_line[1 + offset], 
	       c = tokenized_line[2 + offset], d = tokenized_line[3 + offset];

	if (b[0] == '*' || c[0] == '*' || d[0] == '*')
		return(error_container(full, -1, CER_MICRO_DENSE));

	if (a[0] == '*')
		a.erase(a.begin());

	// ATR2 doesn't do anything here, either.
	if (!is_integer(a, true) || !is_integer(b, true) ||
			!is_integer(c, true) || !is_integer(d, true))
		return(error_container(full, -1, CER_MICRO_NONINT));

	int a_mark = stoi_generalized(a), b_mark = stoi_generalized(b), 
	    c_mark = stoi_generalized(c), d_mark = stoi_generalized(d);

	// All checks OK, alter target.
	dest.set_opcode_directly(a_mark);
	dest.set_modifier_directly(b_mark);
	dest.set_a_field_directly(c_mark);
	dest.set_b_field_directly(d_mark);
	
	return(error_container(CER_NOERR));
}

string cmd_parse::disassemble_single(const field_entry & input) const {
	// First, the core.
	string numeric = itos((signed short)input.value);

	// Is it a @ or [], or jump? If so, append.
	numeric = add_indirects(numeric, input);
	if (input.modifier.is_n_jump) numeric = ":" + numeric;
	if (input.modifier.is_a_jump) numeric = "!" + numeric;

	return(numeric);
}

string cmd_parse::disassemble(const code_line & source) const {
	// This will be tricky. Well, to get any quality will be tricky -
	// doing naive opcode lookup and then ints for the arguments is easy.
	
	// For now, we'll do this compromise (as in ATR2): the first one gets
	// a lookup (to opcode) and number otherwise, the second and third are
	// numbers, and if the first one has any modifier bit (like being
	// a jump or indirect), then it too is a number.
	
	// DONE: If is_jump and any of the other two, then something
	// weird is going on, so print as microcode. (Handled in is_data.)
	
	// Before we do anything else, check if it's data. If so, print as
	// microcode. (Not infallible)
	if (source.is_data()) {
		string val = "*" + itos(source.get_opcode_directly()) + 
			", " + itos(source.get_modifier_directly()) + 
			", " + itos(source.get_a_field_directly()) + 
			", " + itos(source.get_b_field_directly());
		return(val);
	}

	// First, we must get the fields.
	field_entry opcode = source.get_opcode(), a = source.get_a_field(),
		    b = source.get_b_field();
	string opcode_res, a_res, b_res;

	// Opcode
	// (Generalized, we get set cover. We're not letting complexity get
	//  to us yet, and so do it simply, although that won't detect things
	//  like [ax] opcodes.)
	if (reverse_lookup.find(opcode) != reverse_lookup.end()) {
		// Lookup
		RLookup::const_iterator 
			firsti = reverse_lookup.lower_bound(opcode), 
			secondi = reverse_lookup.upper_bound(opcode), after;
		after = secondi;
		++after;

		// UNTESTED
		if (firsti != secondi) {
			// Multiple answers. To get the opcode, we go through
			// all of them and disregard those that are either "i_"
			// or "p_", since those belong to interrupts and ports.
			bool found = false;

			for (RLookup::const_iterator pos = firsti; pos != after 
					&& !found; ++pos) {

				string maybe = pos->second;

				if ((maybe[0] != 'p' || maybe[0] != 'i') &&
						(maybe[1] != '_')) {
					found = true;
					opcode_res = maybe;
				}
			}
		} else  // Only one choice, pick it.
			opcode_res = firsti->second;
	} else
		opcode_res = disassemble_single(opcode);

	a_res = disassemble_single(a);
	b_res = disassemble_single(b);

	return(opcode_res + "  " + a_res + ", " + b_res);
}

#endif
