// This turns a written program into the correct code_lines.
// There are three steps. The first step turns unstructured code into
// 2D vectors of strings (done by mass_tokenize). The second step turns !labels
// with names into numbers (and declared variables into memory references) and 
// populates the jump tables. 
// The third step finally turns the 2D vectors of strings into code_lines.

// We may have to write this to output into a class CPU later.

// NOTE: The offset trick means that robots that use literal offsets to access
// tables will fail. This might be unacceptable, and so we should introduce a
// quirks mode later. (BLUESKY that. Use is_data and something.. hm, how would 
// we prevent microcode-as-table from being destroyed but destroy microcode-as-
// code correctly?)
// 	Said runtime quirk could look at direct references (memory > 1024) and
// 	subtract the number of :labels and !labels up to that point (this number
// 	stored in an array). Thus it would only come into play upon reference, 
// 	and not do anything with the microcode. Though that's another reference
// 	to add to execute(, which is spending enough time pushing and popping 
// 	as it is...
// 		Another way would be to have O(n) :/!label counter inside the
// 		memory reference code. That'd be ugly and slow, but we don't
// 		intend on using the quirks mode often anyhow. The only problem
// 		is that there's no evidence that a robot needs said quirks mode
// 		until you see it fail to perform... Therefore, the compensation
// 		would have to be used instead of offset, and hence it'd have to
// 		be on all the time, which excludes O(n).
//	I don't think I'll do this - robots with hardwired locations will just
//	have to fail instead. There aren't many of them anyhow, and to have
//	a translator work as if there were no !labels conceals the code and
//	would add complexity for things like "mov ax, !foo" where !foo would
//	have to be translated /back/ so that the lookup goes to the right
//	place.

// DONE: Append NOP to the list as in ATR2.

// DONE: Abort if we find #lock because we don't support it (out of courtesy
// to the authors who wrote locked robots ten years ago, since DRM can't
// possibly ever work in OSS).

#ifndef __KROB_COMPILER
#define __KROB_COMPILER

#include <string>
#include <vector>
#include <map>
#include "../tools.cc"
#include "../configorder.h"
#include "command_lookup.cc"
#include "error_container.cc"
#include "indirect.cc"
#include "errors.h"
#include <assert.h>

using namespace std;

class compiler {

	private:
		cmd_parse parser;

		vector<string> tokenize(const string input_string,
				const string & delimiters, 
				const char comment_marker,
				const string & preserve);
		
		template<class T> vector<vector<string> > mass_tokenize(T &
				input, const string terminator, 
				bool turn_lowercase, 
				error_container & poss_error, 
				const string preserve,
				vector<int> & line_count);

		vector<vector<string> > separate_compiler_directives(
				vector<vector<string> > & mass_tokenized,
				vector<int> & line_count);

		error_container rewrite_alphanumerics(vector<vector<string > > &
				input, vector<vector<string> > & directives,
				vector<int> & alnum_jump_table, 
				int uservar_mem_start, int uservar_mem_end,
				bool offset);

		error_container populate_n_jumps(bool strict,
				vector<vector<string > > & input, 
				vector<int> & numeric_jump_table, bool offset);

		error_container reassemble_config_tradeoffs(bool strict, const 
				vector<vector<string > > & input, 
				vector<int> & output);

		string get_robot_directive(const vector<vector<string> > & 
				input, const string directive_name);

	public:
		void debug(string fn);
		// For maxlines, -1 means no limit.
		template<typename T> error_container assemble(T & input,
				string input_name, vector<code_line> & output,
				int maxlines, bool count_jumps, 
				int allowed_jumps, int mem_start, int mem_end,
				vector<int> & numeric_jump_table,
				vector<int> & alnum_jump_table, 
				string & message, int & CPU_cycles_per_cycle,
				vector<int> & config_tradeoffs,
				vector<int> & line_numbers,
				bool offset, bool verbose,
				bool strict_compile);
};

// Turn a string separated by one of the delimiters and with a possible end-of-
// line type comment starting with comment_marker into a group of tokens.
// Taken from www.oopweb.com C++ programming howto.
vector<string> compiler::tokenize(string input_string, 
		const string & delimiters, const char comment_marker,
		const string & preserve) {

	// Snip comments first.
	if ((int)input_string.find(comment_marker) != -1)
		input_string.resize(input_string.find(comment_marker));

	// Skip delimiters at beginning.
	string::size_type lastPos = input_string.find_first_not_of(delimiters, 
			0);
	// Find first "non-delimiter".
	string::size_type pos = input_string.find_first_of(delimiters, 
			lastPos);

	vector<string> tokens;

	while (string::npos != pos || string::npos != lastPos) {
		// Found a token, add it to the vector.
		tokens.push_back(input_string.substr(lastPos, pos-lastPos));
		// Special case for preserve; if this token is the first, and
		// it's like preserve, the rest of the line should be preserved
		// completely (apart from comments).
		if (tokens.size() == 1)
			if (tokens[0] == preserve && preserve.size() > 0) {
				tokens.push_back(input_string.substr(pos+1,
							input_string.size()));
				return(tokens);
			}
		// Skip delimiters. Note the "not of"
		lastPos = input_string.find_first_not_of(delimiters, pos);
		// Find next "non-delimiter"
		pos = input_string.find_first_of(delimiters, lastPos);
	}

	return(tokens);
}

// input is the stream that provides the input, terminator is the end tag,
// and turn_lowercase does just that if true.
// If a token group starts with "preserve", then the original string is
// preserved instead. This is used for #msg notes where it's important that
// all punctuation remains as is.
// Now handles lines, too. This is kinda ugly, because the mass_tokenize
// function isn't supposed to have any side effects, but I don't really know
// how to do this more compactly (except by creating a new class).
template<class T> vector<vector<string> > compiler::mass_tokenize(T &
		input, const string terminator, bool turn_lowercase,
		error_container & poss_error, string preserve,
		vector<int> & line_count) {

	string line;

	bool found_terminator = false;
	vector<vector<string> > all_tokenized;

	// Also triggers on no access. Make sure you check whether it's
	// because the file doesn't exist or because it's set to different
	// permissions in that case, elsewhere.
	if (!input) {
		poss_error.error = CER_NOFOUND;
		return(all_tokenized);
	}

	// For all lines, as long as the current line isn't a terminator,
	// get, tokenize, and add to all_tokenized.

	line_count.resize(0);
	int counter = 0;

	while (!input.eof() && !found_terminator) {
		// If the stream has suddenly gone bad, and it's not because
		// of eof (which would make the loop terminate at the while),
		// then something is up; notify the user and bail out.
		// BLUESKY: Add detail (No access, etc.)
		if (!input.good()) {
			poss_error.error = CER_NOACCESS;
			return(all_tokenized);
		}

		++counter;	// Starts at 1 - this is intentional
		getline(input, line);

		if (turn_lowercase) {
			line = lowercase(line);
			preserve = lowercase(preserve);
		}

		// Acceptable delimiters are " ", tab, and ,; we also add 
		// \r and \n to get rid of end-of-line characters and be
		// cross-platform, and the EOF char (0x1A) for the input cases
		// where programmers using COPY CON added one EOF char too much
		// (which happens a few times in the robot library).
		// If preserved, should it really ignore case?
		vector<string> tokenized = tokenize(line, " ,\r\n\t\032", ';',
				preserve);

		if (tokenized.size() == 0) continue;

		if (tokenized[0] == terminator)
			found_terminator = true;
		else {
			all_tokenized.push_back(tokenized);
			line_count.push_back(counter);
		}
	}

	return(all_tokenized);
}

// Here we remove all the compiler directives (whose opcode fields start in a #)
// Since we're dealing with a vector, this is slow, but we don't care (at least
// not yet). The function must be run before doing any jump population, because
// it alters the relative location of the non-directive instructions.
vector<vector<string> > compiler::separate_compiler_directives(
		vector<vector<string> > & mass_tokenized,
		vector<int> & line_count) {

	vector<vector<string> > directives;

	vector<vector<string> > :: iterator pos = mass_tokenized.begin();
	vector<int> :: iterator cpos = line_count.begin();

	while (pos != mass_tokenized.end()) {
		if (*pos->begin()->begin() == '#') {
			directives.push_back(*pos);
			pos = mass_tokenized.erase(pos);
			cpos = line_count.erase(cpos);
		} else {
			++pos;
			++cpos;
		}
	}

	return(directives);
}

// Rewrite alphanumeric references to become numerical references instead.
// The function handles two different alphanumerics: user-defined variables
// and !jumps.
// Uservar_mem_start defines the first memory location available to user
// variables, and uservar_mem_end the last. Note that collisions may happen if
// you address this area yourself. Better versions would have a preliminary
// pass that screens those out, but later.
// If offset is true, we don't store the actual position in the jump table,
// but the position plus one. This is how ATR2 does it; or rather, ATR2
// removes the actual label "opcodes", but because we don't explicitly judge
// microcode as incorrect, we don't do it here (or microcode parsing to :300
// would be removed and thus might mess up lookup to the table it's part of).

// DONE: Somehow generalize [ and @ type stripping, since we're permitted to
// do things like mov ax, [!q].

error_container compiler::rewrite_alphanumerics(vector<vector<string > > & 
		input, vector<vector<string> > & directives, 
		vector<int> & alnum_jump_table, int uservar_mem_start,
		int uservar_mem_end, bool offset) {

	// First go through all fields that have ! in the "opcodes" slot.
	// These are labels. Add them to a map and replace with a numeric ! 
	// with the map's target.
	// DONE: Handle case of preexisting numeric !s correctly.
	// Should work even as it is because the first pass only touches
	// opcode slots, and the second pass only touches non-opcode slots.
	
	map<string, string> lookup;
	int jump_count = 0;

	size_t counter, sec;
	for (counter = 0; counter < input.size(); ++counter) {
		// If there isn't a ! in the opcode slot for this line,
		// continue
		if (input[counter].size() < 1) continue;
		if (input[counter][0][0] != '!') continue;

		string source = input[counter][0];

		// If it's already defined, let the user know.
		if (lookup.find(source) != lookup.end()) 
			return(error_container(source, -1, CER_XLBL_COLLIDE));

		// Okay, all clear, set it up.
		if (offset)
			alnum_jump_table[jump_count] = counter + 1;
		else	alnum_jump_table[jump_count] = counter;

		string replacement = "!" + itos(jump_count++);
		lookup[source] = replacement;
		input[counter][0] = replacement;
	}

	// Then go through all non-opcode slots and replace ! plus something
	// with the map's target. If it's not found in the map's target, then
	// it's referencing a jump we don't have, so bug off.
	
	for (counter = 0; counter < input.size(); ++counter) {
		for (sec = 1; sec < input[counter].size(); ++sec) {
			string current = input[counter][sec];
			field_entry current_indirect;
			// Strip any indirects
			current = strip_indirects(current, current_indirect,
					false);
			
			// Screen out those that don't matter
			if (current[0] != '!') continue;
			
			// If it's not in our table, then bug out.
			if (lookup.find(current) == lookup.end())
				return(error_container(input[counter][sec], -1,
							CER_XLBL_NOTFOUND));

			// Else, replace
			// BLUESKY: If it's a jump-type op, only add the number.
			// (Cosmetic, since while ATR2 doesn't like "jmp :2",
			//  we handle it)
			input[counter][sec] = add_indirects(lookup[current],
					current_indirect);
		}
	}

	// Okay, !jumps are done. Now handle user-defined variables in much
	// the same way.
	map<string, string> userdef_lookup;
	int current_memloc = uservar_mem_start;
	int allowed_vars = (uservar_mem_end - uservar_mem_start) - 1;

	for (counter = 0; counter < directives.size(); ++counter) {
		// If it's not a definition or it has no name, then ignore.
		if (directives[counter][0] != "#def" || 
				directives[counter].size() < 2) continue;

		// Remove any indirects that may obfuscate the definition.
		// While ATR2 permits things like #def [amoeba], regular
		// instructions don't handle them (in this case, "mov [amoeba],
		// 1" returns incorrect identifier "amoeba", and stripping
		// indirects here is the right thing to do anyway, thus we
		// do so.)
		field_entry current_indirect;
		string processed = strip_indirects(directives[counter][1],
				current_indirect, false);

		// If there's no room or it's already been defined, let loose
		// an error.
		if (current_memloc >= uservar_mem_end)
			return(error_container(directives[counter][1], 
						allowed_vars, 
						CER_TOO_MANY_VARS));

		if (userdef_lookup.find(processed) != userdef_lookup.end())
			return(error_container(directives[counter][1], -1,
						CER_DUPLICATE_DEF));

		userdef_lookup[processed] = "@" + itos(current_memloc++);
	}

	// Now replace. (Yes, you can #def 'mov' and it'll work; it does in
	// ATR2. Maybe not good behavior though)
	
	for (counter = 0; counter < input.size(); ++counter)
		for (sec = 0; sec < input[counter].size(); ++sec) {
			// UGLY HACK! Ditto.
			field_entry current_indirect;
			string processed = strip_indirects(input[counter][sec],
					current_indirect, false);

			if (userdef_lookup.find(processed) !=
					userdef_lookup.end()) {
				input[counter][sec] = add_indirects(
						userdef_lookup[processed],
						current_indirect);
			}
		}

	return(error_container(CER_NOERR));
}

// Offset does the same as in rewrite_alphanumerics.

error_container compiler::populate_n_jumps(bool strict,
		vector<vector<string > > & input, 
		vector<int> & numeric_jump_table, bool offset) {

	// Go through all opcode slots, looking for :
	// Any : with a numeric response after is put into the numeric jump
	// table. Any odd responses (out of bounds or not-numeric) gets an
	// error.
	
	for (size_t counter = 0; counter < input.size(); ++counter) {
		if (input[counter][0][0] != ':') continue;
		if (input[counter].size() < 1) continue;

		string source = input[counter][0];
		source.erase(source.begin());

		// Empty labels are ignored if we're in quirks mode.
		if (source.empty() && !strict) continue;

		if (!is_integer(source, false))
			return(error_container(input[counter][0], -1,
						CER_INVALID_NLABEL));

		// DONE: If the jump number's larger than max 16 bit size,
		// we can't reach this jump in any case, so just ignore it.
		// Should also be a special error. "Label out of bounds" or
		// somesuch.. maybe even if the number's larger than maxsize.
		// If it's not strict, we just ignore it. Note that this can
		// cause subtle displacement errors if you're using hardcoded
		// addresses for your data tables.
		int jump_number = stoi(source);
		if (jump_number < 0 || jump_number >= (int)numeric_jump_table.
				size()) {
			if (strict)
				return(error_container(input[counter][0], -1,
							CER_NLABEL_OORANGE));
			else	continue;
		}

		if (offset)
			numeric_jump_table[stoi(source)] = counter + 1;
		else	numeric_jump_table[stoi(source)] = counter;
	}

	return(error_container(CER_NOERR));
}

// Search through the input for #config lines. When we find some, split by =
// and dump to output in the standard order, which is "Scanner Weapon Armor
// Engine Heatsinks Mines Shield". Unspecified values are set to -1 (this means
// there'll be a shadowing of -1, but we return error on any value < 0).
error_container compiler::reassemble_config_tradeoffs(bool strict, const
		vector<vector<string > > & input, vector<int> & output) {

	// First search for all the #configs. Dump #configs into a separate
	// array.
	// Then go through each, concatenating the stuff after #config so that
	// "weapon=5" and "weapon = 5" will be treated equally.
	// Then split by =.
	// Finally, go through each and see if it's shields, weapons, etc.
	// Any unknowns get us dumped with an error.
	
	// This has been rolled into a single loop, thus concat and tokenized
	// are of single lines.

	vector<string> tokenized;

	// index map. This is slow, but it's easier to code.
	// Maybe have a struct with something like DEV_SCANNER, DEV_WEAP, etc.
	map<string, int> index_map;
	index_map["scanner"] = CNF_SCANNER;
	index_map["weapon"] = CNF_WEAPON;
	index_map["armor"] = CNF_ARMOR;
	index_map["engine"] = CNF_ENGINE;
	index_map["heatsinks"] = CNF_HEATSINKS;
	index_map["mines"] = CNF_MINES;
	index_map["shield"] = CNF_SHIELD;

	output = vector<int>(C_NUMDEVICES, -1);

	for (size_t counter = 0; counter < input.size(); ++counter) {
		if (input[counter][0] != "#config") continue;
		// This is a config, so concatenate the rest
		string concat;
		for (size_t sec = 1; sec < input[counter].size(); ++sec)
			concat += input[counter][sec];
		// Tokenize
		tokenized = tokenize(concat, "=", ';', "");

		// Strict: if it's vacant (#config shield=), then it's an
		// invalid association. If non-strict, we just ignore this
		// particular #config.
		if (tokenized.size() < 2) {
			if (strict)
				return(error_container(concat, -1, 
							CER_INV_DEV_ASN));
			else	continue;
		}

		// Then set the value. If we're non-strict and encounter upon
		// a non-integer value, just ignore the thing.
		int val, pos;
		if (!is_integer(tokenized[1], false)) {
			if (strict)
				return(error_container(concat, -1, 
							CER_INV_DEV_ASN));
			else	continue;
		}else	val = stoi(tokenized[1]);

		// Invalid value. If strict, this fails it, otherwise it's
		// set to 0.
		if (val < 0) {
			if (strict)
				return(error_container(concat, -1, 
							CER_INV_DEV_ASN));
			else	val = 0;
		}

		// Can't find the device specified
		if (index_map.find(tokenized[0]) == index_map.end())
			return(error_container(concat, -1, CER_INVALID_DEV));
		else	pos = index_map.find(tokenized[0])->second;

		// Finally.. if it's already set, complain.
		if (output[pos] != -1)
			return(error_container(concat, -1, CER_INVALID_DEV));

		output[pos] = val;
	}

	return(error_container(CER_NOERR));
}

// Determines a single-parameter robot #directive, or "" if none. Fairly simple,
// just put here so that we don't clutter assemble() itself.
string compiler::get_robot_directive(const vector<vector<string> > & input,
		string search_for) {

	string cand = "";

	// Get the value of the last directive.
	for (size_t counter = 0; counter < input.size(); ++counter)
		if (input[counter][0] == search_for)
			// Spared from word tokenization because of "preserved".
			cand = input[counter][1];

	// Remove \r\n. Better may be to remove all non-printables.
	
	string output;
	for (size_t counter = 0; counter < cand.size(); ++counter)
		if (cand[counter] != '\r' && cand[counter] != '\n')
			output.push_back(cand[counter]);

	return(output);
}

// Putting it all together...

template<typename T> error_container compiler::assemble(T & input,
		string input_name, vector<code_line> & output,
		int maxlines, bool count_jumps, int allowed_jumps, 
		int mem_start, int mem_end, vector<int> & numeric_jump_table,
		vector<int> & alnum_jump_table, string & message,
		int & CPU_cycles_per_cycle, vector<int> & config_tradeoffs, 
		vector<int> & line_numbers, bool offset, bool verbose, 
		bool strict_compile) {

	// First slurp from the file, and separate code and directives.

	numeric_jump_table.resize(allowed_jumps, -1);
	alnum_jump_table.resize(allowed_jumps, -1);

	if (verbose)
		cout << "ASSEMBLE: Mass tokenizing...." << flush;

	// Tokenized doesn't alter the error reference, only the error value.
	// Therefore we set it to whatever value we were passed as input name.
	// (Not opening the file directly permits us to use whatever stream
	//  comes our way; it's the responsibility of the function calling this
	//  to invent an appropriate name in that case.)
	error_container retval(input_name, -1, CER_NOERR);

	// ???: Maybe add a #begin token which discards everything prior to it,
	// and then ignores further #begin tokens? Could make it easier to
	// include code in news posts, but it wouldn't be backwards compatible.
	vector<int> lines;
	vector<vector<string> > tokenized = mass_tokenize(input, "#end", true,
			retval, "#msg", lines);

	if (retval.error != CER_NOERR) {
		if (verbose)
			cout << "\tfailed!" << endl;
		return(retval);
	}

	if (verbose)
		cout << "\tdone." << endl <<
			"ASSEMBLE: Separating compiler directives...." << flush;
	// Has side effect
	vector<vector<string> > directives = separate_compiler_directives(
			tokenized, lines);
	if (verbose)
		cout << "\tdone." << endl <<
			"ASSEMBLE: Reading #config directives..." << flush;

	retval = reassemble_config_tradeoffs(strict_compile, directives, 
			config_tradeoffs);
	if (retval.error != CER_NOERR) {
		if (verbose)
			cout << "\tfailed!" << endl;
		return(retval);
	}

	if (verbose)
		cout << "\tdone." << endl <<
			"ASSEMBLE: Determining robot #msg, #time, and #lock " <<
			"directives..." << flush;
	
	// Perhaps this could be unified into a map. Then it would be k log n
	// instead of kn.
	
	// Check for locked bots first.
	string locked = get_robot_directive(directives, "#lock");
	if (locked == "")
		locked = get_robot_directive(directives, "#lock2");
	if (locked == "")
		locked = get_robot_directive(directives, "#lock3");

	if (locked != "") {
		// Not exactly "failed" - we found what we were looking for..
		if (verbose) cout << "\tdone." << endl;
		retval.error = CER_LOCKED;
		return(retval);
	}

	// Okay, unlocked, now look for msg and time stuff.
	message = get_robot_directive(directives, "#msg");
	string time_dir = get_robot_directive(directives, "#time");

	if (is_integer(time_dir, false))
		CPU_cycles_per_cycle = stoi(time_dir);
	else	CPU_cycles_per_cycle = -1;

	// Guard against odd input.
	if (CPU_cycles_per_cycle < 0)
		CPU_cycles_per_cycle = -1;

	if (verbose)
		cout << "\tdone." << endl <<
			"ASSEMBLE: Rewriting !jumps..." << flush;

	// Translate characters into numbers
	// Shouldn't directives be const?
	retval = rewrite_alphanumerics(tokenized, directives, alnum_jump_table,
			mem_start, mem_end, offset);
	if (retval.error != CER_NOERR) {
		if (verbose) 
			cout << "\tfailed!" << endl;
		return(retval);
	}
	if (verbose)
		cout << "\tdone." << endl << 
			"ASSEMBLE: Populating jump tables..." << flush;

	retval = populate_n_jumps(strict_compile, tokenized, numeric_jump_table,
			offset);
	if (retval.error != CER_NOERR) {
		if (verbose)
			cout << "\tfailed!" << endl;
		return(retval);
	}

	if (verbose)
		cout << "\tdone." << endl <<
			"ASSEMBLE: Constructing microcode..." << flush;

	// Finally, assemble all.
	//cmd_parse parser;
	output.resize(tokenized.size()+1);
	// Keep count of how many non-jump lines we have.
	int operating_lines = 0;

	// Create mapping from "effective lines" (lines of code) to "real lines"
	// (lines of source). We do this by adding the line number array (lines
	// of source with #directives removed) corresponding to line x when
	// line x is added to the assembled program.
	line_numbers.resize(0);

	for (size_t counter = 0; counter < tokenized.size(); ++counter) {
		string opcode = "0", a_field = "0", b_field = "0";
		opcode = tokenized[counter][0];
		if (tokenized[counter].size() > 1)
			a_field = tokenized[counter][1];
		if (tokenized[counter].size() > 2)
			b_field = tokenized[counter][2];

		// If in quirks mode, ignore empty labels. Otherwise,
		// make the program break so that ! doesn't inadvertently
		// get translated to !0 and mask the error.
		if ((opcode == ":" || opcode == "!") &&
				tokenized[counter].size() < 2) {
			if (strict_compile)	a_field = "";
			else			continue;
		}

		// First try the microcode.
		// DONE: Check what happens with the assembled code when we
		// have empty lines in the source. After all, it seems to have
		// a 1:1 mapping here. Perhaps empty lines are excluded
		// earlier..
		// We need to figure this out in order to be sure that
		// line_numbers work as expected.
		retval = parser.get_line_from_microcode(tokenized[counter],
				output[counter]);
		line_numbers.push_back(lines[counter]);

		// If that wasn't applicable, try the regular way.
		if (retval.error == CER_NOT_APPLICABLE || retval.error == 
				CER_MICRO_NONMICRO)
			retval = parser.get_line(strict_compile, opcode, 
					a_field, b_field, output[counter], 
					numeric_jump_table, alnum_jump_table);

		if (retval.error != CER_NOERR) {
			if (verbose)
				cout << "\tfailed!" << endl;
			return(retval);
		}
		
		// Note that this does open a form of loophole with jump
		// tables beyond the limit and then jmp ax.
		field_entry our_opcode = output[counter].get_opcode();
		if ((!our_opcode.modifier.is_a_jump && 
					!our_opcode.modifier.is_n_jump) ||
				count_jumps)
			++operating_lines;
	}

	if (maxlines != -1 && operating_lines > maxlines) {
		if (verbose)
			cout << "\tfailed!" << endl;
		return(error_container(itos(operating_lines), maxlines,
					CER_LONG_PROGRAM));
	}

	if (verbose)
		cout << "\tdone." << endl;

	// Add the NOP to prevent off-by-one errors. This shouldn't return
	// an error; if it does, something's really wrong and we break early.
	assert( parser.get_line(false, "nop", "0", "0", 
				output[tokenized.size()],
				numeric_jump_table, alnum_jump_table).error == 
			CER_NOERR);

	return(CER_NOERR);
}

#endif
