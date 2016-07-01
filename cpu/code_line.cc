// Microcode, OO style with (some) compatibility with ATR2.
// We use a structure that's based on multiple "field_entry" structs, cast
// to the appropriate type. This differs from ATR2's "opcode, modifier, a-field,
// b-field" type, so we also translate on the fly. Tables are thus permitted,
// but inline microcode won't be decoded to mean the same things, so beware.

// These really ought to be collapsed to one structure - we use a lot of time
// translating back and forth. Then we could just put in interface operators 
// for reading microcode (and writing it).

#include "prog_constants.h"
#include <iostream>
#include <strings.h>

using namespace std;

#ifndef __KROB_MICROCODE
#define __KROB_MICROCODE

// these are private
typedef struct modbits {
	bool is_indirect_one;	// @, first level of pointer reference
	bool is_indirect_two;	// [], second level, used for things like [ax].
	bool is_n_jump;		// for things like mov dx, :10
	bool is_a_jump;		// same, but with mov dx, !tablebegin
};

// Make it an explicit bitfield now.

// These must be powers of two.
typedef enum { MB_OPCODE_INDIR_ONE = 1, MB_OPCODE_INDIR_TWO = 2,
	MB_OPCODE_IS_N_JUMP = 4, MB_OPCODE_IS_A_JUMP = 8, 
	MB_A_FIELD_INDIR_ONE = 16, MB_A_FIELD_INDIR_TWO = 32, 
	MB_A_FIELD_IS_N_JUMP = 64, MB_A_FIELD_IS_A_JUMP = 128,
	MB_B_FIELD_INDIR_ONE = 256, MB_B_FIELD_INDIR_TWO = 512,
	MB_B_FIELD_IS_N_JUMP = 1024, MB_B_FIELD_IS_A_JUMP = 2048 } mb_entry;

class modifier_block {
	private:
		unsigned short value;

	public:
		modifier_block();
		void set_directly(const unsigned short value);
		unsigned short get_directly() const { return(value); }
		void set_flag(const mb_entry field, bool value);
		bool get_flag(const mb_entry field) const;
};

modifier_block::modifier_block() {
	value = 0;
}

void modifier_block::set_directly(const unsigned short value_in) {
	value = value_in;
}

void modifier_block::set_flag(const mb_entry field, bool value_in) {
	if (value_in != ((value & field) != 0))
		value ^= field;		// Set to one if required
}

bool modifier_block::get_flag(const mb_entry field) const {
	return ((value & field) != 0);
}

// This is the kind of field we use when interpreting the assembly.
// It's only a class so that reverse lookups will work (and so that init values
// will all be false).
class field_entry {
	public:
		unsigned short value;
		modbits modifier;
		bool operator < (const field_entry & other) const;
		field_entry();
};

bool field_entry::operator< (const field_entry & other) const {
	if (value < other.value) return(true);
	if (other.modifier.is_indirect_one && !modifier.is_indirect_one) 
		return(true);
	if (other.modifier.is_indirect_two && !modifier.is_indirect_two)
		return(true);
	if (other.modifier.is_n_jump && !modifier.is_n_jump)
		return(true);
	if (other.modifier.is_a_jump && !modifier.is_a_jump)
		return(true);
	return(false);
}

field_entry::field_entry() {
	value = -1;
	modifier.is_indirect_one = false;
	modifier.is_indirect_two = false;
	modifier.is_n_jump = false;
	modifier.is_a_jump = false;
}

// This is how it's stored in memory.
// Maybe store it differently and have the interface on data access later,
// but we'll see.
class code_line {
	private:
		command opcode;
		modifier_block modfield; // really a bitfield
		short a_field, b_field;

		// For detecting whether we're dealing with data microcode
		// (not foolproof)
		modifier_block reconstruct_field(const modifier_block & in) const;

	public:
		code_line();
		code_line(const field_entry opcode_in, 
				const field_entry a_field_in, 
				const field_entry b_field_in);
		code_line(unsigned short opcode_in, unsigned short modifier_in,
				short a_field_in, short b_field_in);

		command get_opcode_value() const { return(opcode); }
		short get_a_field_value() const { return(a_field); }
		short get_b_field_value() const { return(b_field); }

		field_entry get_opcode() const; // reads off modifier field as req'd.
		field_entry get_a_field() const;
		field_entry get_b_field() const;

		void set_opcode(const field_entry opcode_in);
		void set_a_field(const field_entry a_field_in);
		void set_b_field(const field_entry b_field_in); 

		void set_opcode_directly(unsigned short value);
		void set_modifier_directly(short value);
		void set_a_field_directly(short value);
		void set_b_field_directly(short value);

		short get_opcode_directly() const;
		short get_modifier_directly() const;
		short get_a_field_directly() const;
		short get_b_field_directly() const;

		bool is_data() const;
};

// Given a modifier_block, this copies only the explicit bits. We use the
// result to figure out if we're dealing with a data block (when disassembling).
// The reasoning is this: only data have nonzero stuff when the explicit bits
// are discounted. That is, if the reconstructed field is not equal to the
// original modifier block, the code line is actually data.
modifier_block code_line::reconstruct_field(const modifier_block & in) const {
	modifier_block out;
	// This could be done with an AND, but I want to be forwards
	// compatible.
	out.set_flag(MB_OPCODE_INDIR_ONE, in.get_flag(MB_OPCODE_INDIR_ONE));
	out.set_flag(MB_OPCODE_INDIR_TWO, in.get_flag(MB_OPCODE_INDIR_TWO));
	out.set_flag(MB_OPCODE_IS_N_JUMP, in.get_flag(MB_OPCODE_IS_N_JUMP));
	out.set_flag(MB_OPCODE_IS_A_JUMP, in.get_flag(MB_OPCODE_IS_A_JUMP));
	out.set_flag(MB_A_FIELD_INDIR_ONE, in.get_flag(MB_A_FIELD_INDIR_ONE));
	out.set_flag(MB_A_FIELD_INDIR_TWO, in.get_flag(MB_A_FIELD_INDIR_TWO));
	out.set_flag(MB_A_FIELD_IS_N_JUMP, in.get_flag(MB_A_FIELD_IS_N_JUMP));
	out.set_flag(MB_A_FIELD_IS_A_JUMP, in.get_flag(MB_A_FIELD_IS_A_JUMP));
	out.set_flag(MB_B_FIELD_INDIR_ONE, in.get_flag(MB_B_FIELD_INDIR_ONE));
	out.set_flag(MB_B_FIELD_INDIR_TWO, in.get_flag(MB_B_FIELD_INDIR_TWO));
	out.set_flag(MB_B_FIELD_IS_N_JUMP, in.get_flag(MB_B_FIELD_IS_N_JUMP));
	out.set_flag(MB_B_FIELD_IS_A_JUMP, in.get_flag(MB_B_FIELD_IS_A_JUMP));

	return(out);
}

code_line::code_line() {
	// Clear, for data comparison
	// The class now does it itself.
	
	// Defaults
	field_entry in;
	in.value = CMD_NOP;
	in.modifier.is_indirect_one = false;
	in.modifier.is_indirect_two = false;
	in.modifier.is_n_jump = false;
	in.modifier.is_a_jump = false;

	set_opcode(in);
	in.value = 0;
	set_a_field(in);
	set_b_field(in);
}

code_line::code_line(const field_entry opcode_in, const field_entry a_field_in,
		const field_entry b_field_in) {
	set_opcode(opcode_in);
	set_opcode(a_field_in);
	set_opcode(b_field_in);
}

code_line::code_line(unsigned short opcode_in, unsigned short modifier_in,
		short a_field_in, short b_field_in) {
	set_opcode_directly(opcode_in);
	set_modifier_directly(modifier_in);
	set_a_field_directly(a_field_in);
	set_b_field_directly(b_field_in);
}

field_entry code_line::get_opcode() const {
	field_entry toRet;
	toRet.value = (short)opcode;
	toRet.modifier.is_indirect_one = modfield.get_flag(MB_OPCODE_INDIR_ONE);
	toRet.modifier.is_indirect_two = modfield.get_flag(MB_OPCODE_INDIR_TWO);
	toRet.modifier.is_n_jump = modfield.get_flag(MB_OPCODE_IS_N_JUMP);
	toRet.modifier.is_a_jump = modfield.get_flag(MB_OPCODE_IS_A_JUMP);
	return(toRet);
}

field_entry code_line::get_a_field() const {
	field_entry toRet;
	toRet.value = a_field;
	toRet.modifier.is_indirect_one =modfield.get_flag(MB_A_FIELD_INDIR_ONE);
	toRet.modifier.is_indirect_two =modfield.get_flag(MB_A_FIELD_INDIR_TWO);
	toRet.modifier.is_n_jump = modfield.get_flag(MB_A_FIELD_IS_N_JUMP);
	toRet.modifier.is_a_jump = modfield.get_flag(MB_A_FIELD_IS_A_JUMP);
	return(toRet);
}

field_entry code_line::get_b_field() const {
	field_entry toRet;
	toRet.value = b_field;
	toRet.modifier.is_indirect_one =modfield.get_flag(MB_B_FIELD_INDIR_ONE);
	toRet.modifier.is_indirect_two =modfield.get_flag(MB_B_FIELD_INDIR_TWO);
	toRet.modifier.is_n_jump = modfield.get_flag(MB_B_FIELD_IS_N_JUMP);
	toRet.modifier.is_a_jump = modfield.get_flag(MB_B_FIELD_IS_A_JUMP);
	return(toRet);
}

void code_line::set_opcode(const field_entry opcode_in) {
	opcode = (command)opcode_in.value;
	modfield.set_flag(MB_OPCODE_INDIR_ONE, 
			opcode_in.modifier.is_indirect_one);
	modfield.set_flag(MB_OPCODE_INDIR_TWO,
			opcode_in.modifier.is_indirect_two);
	modfield.set_flag(MB_OPCODE_IS_N_JUMP, opcode_in.modifier.is_n_jump);
	modfield.set_flag(MB_OPCODE_IS_A_JUMP, opcode_in.modifier.is_a_jump);
}

void code_line::set_a_field(const field_entry a_field_in) {
	a_field = a_field_in.value;
	modfield.set_flag(MB_A_FIELD_INDIR_ONE,
			a_field_in.modifier.is_indirect_one);
	modfield.set_flag(MB_A_FIELD_INDIR_TWO,
			a_field_in.modifier.is_indirect_two);
	modfield.set_flag(MB_A_FIELD_IS_N_JUMP,
			a_field_in.modifier.is_n_jump);
	modfield.set_flag(MB_A_FIELD_IS_A_JUMP,
			a_field_in.modifier.is_a_jump);
}

void code_line::set_b_field(const field_entry b_field_in) {
	b_field = b_field_in.value;
	modfield.set_flag(MB_B_FIELD_INDIR_ONE,
			b_field_in.modifier.is_indirect_one);
	modfield.set_flag(MB_B_FIELD_INDIR_TWO,
			b_field_in.modifier.is_indirect_two);
	modfield.set_flag(MB_B_FIELD_IS_N_JUMP,
			b_field_in.modifier.is_n_jump);
	modfield.set_flag(MB_B_FIELD_IS_A_JUMP,
			b_field_in.modifier.is_a_jump);
}

void code_line::set_opcode_directly(unsigned short value) {
	opcode = (command)value;
}
void code_line::set_modifier_directly(short value) {
	// ??? Iffy?
	modfield.set_directly(value);
}
void code_line::set_a_field_directly(short value) {
	a_field = value;
}
void code_line::set_b_field_directly(short value) {
	b_field = value;
}

short code_line::get_opcode_directly() const {
	return ((short)opcode);
}

short code_line::get_modifier_directly() const {
	return(modfield.get_directly());
}

short code_line::get_a_field_directly() const {
	return(a_field);
}

short code_line::get_b_field_directly() const {
	return(b_field);
}

// If this returns true, it is data. However, it may return false and still
// be data. Sufficient but not necessary.
// (You may ask, why not set aside a "is_microcode" bit in the class? That
//  would cause false positives if you use microcode to write, well, code.)

// Perhaps set_data_explicitly to propagate data chunks? Hm.

// Iffy when you use -O9. FIXED.
bool code_line::is_data() const {
	modifier_block recreation = reconstruct_field(modfield);

	int r_int = recreation.get_directly();
	int m_int = modfield.get_directly();

	if (r_int != m_int) return(true);

	// Okay, so no unexpected values. Return true if we have a jump
	// (! or :) and one of the indirect fields are set.

	return ((modfield.get_flag(MB_OPCODE_IS_N_JUMP) || 
				modfield.get_flag(MB_OPCODE_IS_A_JUMP)) 
			&& (modfield.get_flag(MB_OPCODE_INDIR_ONE) || 
				modfield.get_flag(MB_OPCODE_INDIR_TWO) || 
				modfield.get_flag(MB_A_FIELD_INDIR_ONE) || 
				modfield.get_flag(MB_A_FIELD_INDIR_TWO) || 
				modfield.get_flag(MB_B_FIELD_INDIR_ONE) || 
				modfield.get_flag(MB_B_FIELD_INDIR_TWO)));

}

#endif
