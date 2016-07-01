// Used by cmd_parse and compiler.
// I can't see how to integrate it properly.

#ifndef _KROB_INDIRECT
#define _KROB_INDIRECT

#include <string>
#include "code_line.cc"

// Given an ATR2 token word with possible [] and @ memory modifiers, set the
// appropriate field_entry booleans (indirect_one for @ and indirect_two for 
// []) and return the token without the modifiers. If strict is on, the only
// nesting permitted is [@], otherwise both @[] and [@] is permitted.
// Perhaps we should use iterators instead of erasing. The function isn't called
// all that often, though.
string strip_indirects(const string input, field_entry & indirects_out,
		bool strict) {

	string toRet = input;

	// If we're not strict, look for a @
	
	string::const_iterator first = input.begin(); // Quicker than []

	if (!strict && *first == '@') {
		indirects_out.modifier.is_indirect_one = true;
		++first;
		toRet.erase(toRet.begin());
	}

	if (*first == '[' && !indirects_out.modifier.is_indirect_two) {
		if (*toRet.rbegin() == ']') {
			indirects_out.modifier.is_indirect_two = true;
			++first;
			toRet.erase(toRet.begin());	// [
			toRet.erase(toRet.end() - 1);	// ]
		}
	}

	if (*first == '@' && !indirects_out.modifier.is_indirect_one) {
		indirects_out.modifier.is_indirect_one = true;
		toRet.erase(toRet.begin());
	}

	return(toRet);
}

// Go the other way
string add_indirects(const string input, const field_entry indirects_in) {

	string out = input;

	if (indirects_in.modifier.is_indirect_one)
		out = "@" + out;

	if (indirects_in.modifier.is_indirect_two)
		out = "[" + out + "]";

	return(out);
}

#endif
