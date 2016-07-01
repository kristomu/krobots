#ifndef _KROB_COREST
#define _KROB_COREST

#include <vector>
#include <fstream>
#include <assert.h>
#include "cpu/compiler.cc"
#include "cpu/corelogic.cc"
#include "cpu/command_lookup.cc"
#include "cpu/error_container.cc"

// Stored core logic. This is a list of CPUs, as well as some instructions
// to compile robots.

// We should be able to change messages and device weightings. The latter
// comes in use for our optimization programs.

class core_storage {

	private:
		vector<vector<int> > device_weighting;
		vector<vector<int> > line_numbers; // Mapping from IP to source
		vector<int> CPU_speed_info;
		vector<string> messages;
		// Perhaps vector<int> upper_cpu_speed_bound ? #time. That's
		// not really about cores, but neither is device_weighting
		// and messages.

		// Length of auxiliary memory structures for each CPU.
		int jumptable_len, pstack_len, memory_len;
		
		// ATR2 constants.
		int uservar_mem_start, uservar_mem_end;

		void set_meta(int jumptable_length, int pstack_length,
				int memory_length);
		int get_weighting_sum(const vector<int> & weighting) const;

	public:
		vector<corelogic> cores; // Eww?

		core_storage(int jumptable_length, int pstack_length, 
				int memory_length);
		core_storage(int pstack_length);

		error_container insert_core(ifstream & program,
				string stream_name, int permitted_points, 
				int permitted_lines, bool verbose,
				bool strict_compile);

		error_container replace_weighting(int index, vector<int> &
				new_weighting, int permitted_points);

		// These are all by reference; it's too costly to duplicate
		// them "each and every time".
		const string & get_message(int index) const;
		const vector<int> & get_weighting(int index) const;
		int get_CPU_speed(int index, int default_CPU_speed,
				int max_CPU_speed) const;
		corelogic & get_core(int index);
		size_t get_num_cores() const { return(cores.size()); }
		
		int lookup_line_number(int core_number, int effective_line);

		// Between rounds.
		void reset_cores();
};

void core_storage::set_meta(int jumptable_length, int pstack_length, 
		int memory_length) {
	jumptable_len = jumptable_length;
	pstack_len = pstack_length;
	memory_len = memory_length;

	// ATR2 constants - beginning and end of user-reserved memory.
	uservar_mem_start = 128;
	uservar_mem_end = 385;
}

int core_storage::get_weighting_sum(const vector<int> & weighting) const {
	// Used to verify points used, to check that the robot isn't trying
	// to cheat.

	int sum = 0;
	for (int counter = 0; counter < weighting.size(); ++counter) 
		sum += max(0, weighting[counter]);

	return(sum);
}

core_storage::core_storage(int jumptable_length, int pstack_length, int
		memory_length) {
	set_meta(jumptable_length, pstack_length, memory_length);
}

core_storage::core_storage(int pstack_length) {
	set_meta(32767, pstack_length, 1024); // ATR2 standard.
}

error_container core_storage::insert_core(ifstream & program,
		string stream_name, int permitted_points, int permitted_lines,
		bool verbose, bool strict_compile) {

	cmd_parse parser;
	vector<int> num_jump_table(jumptable_len, -1),
		alnum_jump_table(jumptable_len, -1);

	vector<short> pstack(pstack_len, 0), memory(memory_len, 0);

	string message_out;
	vector<int> device_weighting_out;
	int CPU_speed_out;

	compiler xyz;
	vector<code_line> out;

	vector<int> these_line_numbers;

	error_container errors = xyz.assemble(program, stream_name, out,
			permitted_lines, false,	jumptable_len, 
			uservar_mem_start, uservar_mem_end,
			num_jump_table, alnum_jump_table, message_out,
			CPU_speed_out, device_weighting_out, 
			these_line_numbers, true, verbose, strict_compile);

	// If it's file related, find out what's the problem, and return that
	// error with the filename in question.
	if (errors.error == CER_NOFOUND || errors.error == CER_NOACCESS) {
		compile_error differentiation = check_filename(stream_name,
				CER_NOERR);

		// No error (HUH?) or some other error than NOFOUND/NOACCESS
		if (differentiation == CER_NOERR)
			return(error_container(stream_name, -1, errors.error));
		else	return(error_container(stream_name, -1, 
					differentiation));
	}

	line_numbers.push_back(these_line_numbers);

	game_balance limits;
	limits.complete_config_values(device_weighting_out);
	if (!strict_compile)
		limits.minimize_config_values(device_weighting_out);

	// Did it compile? If so, aggregate the tables and compiled program
	// (actually core), into the list. If not, return the error instead.
	
	if (errors.error != CER_NOERR)
		return(errors);

	// Check if the device #CONFIG weighting is permitted. If not,
	// bail out with a cheater's error.
	
	int pts = get_weighting_sum(device_weighting_out);
	if (pts > permitted_points)
		return(error_container(itos(pts), permitted_points, 
					CER_CHEATER));

	cores.push_back(corelogic(pstack.size(), memory.size(), out,
				num_jump_table, alnum_jump_table));
	device_weighting.push_back(device_weighting_out);
	messages.push_back(message_out);
	CPU_speed_info.push_back(CPU_speed_out);

	return(error_container(CER_NOERR));
}

error_container core_storage::replace_weighting(int index, vector<int> &
		new_weighting, int permitted_points) {

	assert (index >= 0 && index < device_weighting.size());

	int pts = get_weighting_sum(new_weighting);
	if (pts > permitted_points)
		return(error_container(itos(pts), permitted_points,
					CER_CHEATER));

	device_weighting[index] = new_weighting;

	return(error_container(CER_NOERR));
}

const string & core_storage::get_message(int index) const {
	assert (index >= 0 && index < messages.size());

	return(messages[index]);
}

const vector<int> & core_storage::get_weighting(int index) const {
	assert (index >= 0 && index < device_weighting.size());

	return(device_weighting[index]);
}

int core_storage::get_CPU_speed(int index, int default_CPU_speed, 
		int max_CPU_speed) const {
	assert (index >= 0 && index < CPU_speed_info.size());

	int CPU_speed;

	if (CPU_speed_info[index] == -1)
		CPU_speed = default_CPU_speed;
	else	CPU_speed = CPU_speed_info[index];

	return(max(0, min(CPU_speed, max_CPU_speed)));
}

corelogic & core_storage::get_core(int index) {
	assert (index >= 0 && index < cores.size());

	return(cores[index]);
}

int core_storage::lookup_line_number(int core_number, int effective_line) {
	if (core_number < 0 || core_number >= cores.size())
		return(-1);
	if (effective_line < 0 || effective_line >= line_numbers[core_number].
			size())
		return(-1);

	return(line_numbers[core_number][effective_line]);
}

void core_storage::reset_cores() {
	for (vector<corelogic>::iterator pos = cores.begin(); 
			pos != cores.end(); ++pos)
		pos->reset_core(true);
}


#endif
