// "Scorekeeper" that keeps statistical information between rounds.
// First, there's a local stats class corresponding to the information
// in /R4, and then there's a global stats class that contains local stats
// information for each round (along with round number and matchid), as well
// as a sum which is used for initing the robot parameters at the beginning
// of another round.

// This is per-robot.

// (Other interesting ideas: have a map of where we moved, where we got hit,
//  etc.; probability distributions.)

#ifndef _KROB_STATS
#define _KROB_STATS

#include <sstream>
#include <vector>
#include <string>
#include <math.h>
#include <list>

using namespace std;

// For single-round cases, things like victories, runs, etc, are either 0 or 1.
// We keep it as int because then we can do a sum without much fuss.

// Now uses arrays so we don't have to reference the variables one by one.

typedef enum info_ref { RI_VICTORY = 0, RI_RUNS = 1, RI_KILLS = 2, 
	RI_DEATHS = 3, RI_ENDARMOR = 4, RI_ENDHEAT = 5, RI_SHOTSFIRED = 6,
	RI_HITS = 7, RI_MINESLAID = 8, RI_MINEHITS = 9, RI_DAMAGE = 10, 
	RI_LIFESPAN = 11, RI_ERRORS = 12, META_RI_ALL = 13
}; 

// Unreferenced calls: record_kill, record_victory, record_error,
// 			increment_round_count
// Half-unreferenced: RI_LIFESPAN (for last man standing)
// Not written RI_KILLS (record_kill not called) FIXED

class round_info {
	private:
		bool set_all(int vict_in, int runs_in, int kills_in, 
				int deaths_in, int endarmor_in, int endheat_in,
				int shots_in, int hits_in, int dmg_in,
				int span_in, int error_in, string name_in);

	public:
		vector<long double> data; // store a lot of data
		string robot_name;
		round_info(int vict_in, int runs_in, int kills_in, int
				deaths_in, int endarmor_in, int endheat_in,
				int shots_in, int hits_in, int dmg_in,
				int span_in, int error_in, string name_in);
		round_info(string name_in);
		// returns true if this field should be an average. (??)
		//string get_r4_info(const double sum_of_how_many) const;
		//string get_r4_info() const;

		long double get_one(int index) const;

		void operator+= (const round_info in);
};

bool round_info::set_all(int vict_in, int runs_in, int kills_in, int deaths_in, 
		int endarmor_in, int endheat_in, int shots_in, int hits_in, 
		int dmg_in, int span_in, int error_in, string name_in) {

	data[RI_VICTORY] = vict_in; data[RI_RUNS] = runs_in; 
	data[RI_KILLS] = kills_in; data[RI_DEATHS] = deaths_in; 
	data[RI_ENDARMOR] = endarmor_in; data[RI_ENDHEAT] = endheat_in;
	data[RI_SHOTSFIRED] = shots_in; data[RI_HITS] = hits_in;
	data[RI_DAMAGE] = dmg_in; data[RI_LIFESPAN] = span_in; 
	data[RI_ERRORS] = error_in;
	robot_name = name_in;
	return(true);
}

round_info::round_info(int vict_in, int runs_in, int kills_in, int deaths_in,
		int endarmor_in, int endheat_in, int shots_in, int hits_in,
		int dmg_in, int span_in, int error_in, string name_in) {
	data.resize(META_RI_ALL);
	set_all(vict_in, runs_in, kills_in, deaths_in, endarmor_in, endheat_in,
			shots_in, hits_in, dmg_in, span_in, error_in, name_in);
}


round_info::round_info(string name_in) {
	data.resize(META_RI_ALL);
	set_all(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, name_in);
}

long double round_info::get_one(int index) const {
	if (index < 0 || index >= data.size()) return(-1);
	return(data[index]);
}

void round_info::operator+= (const round_info in) {

	// Manage the entire array
	for (size_t counter = 0; counter < data.size(); ++counter)
		data[counter] += in.data[counter];
}

class global_round_info {
	private:
		// Useful for determining CW, bothersome rounds, etc
		// Obviously this will gobble up memory. Perhaps have a
		// switch regarding whether we should keep past record or not.
		list<round_info> past_records;
		round_info sum;
		string get_name() { return(sum.robot_name); }

		int number_added;

	public:
		global_round_info(string robot_name);
		// If log_round is false, only the sum is updated. Otherwise,
		// the per-round info is updated as well.
		void add_information(const round_info & to_add, bool log_round);
		const round_info * get_sum() const { return(&sum); }

		// Some way of fetching stuff from past records goes here.

		// These are accessed by robots inside the round, and thus have
		// shortcuts. Writing up all the setters and getters isn't
		// needed yet.
		int get_total_kills() const { return(sum.data[RI_KILLS]); }
		int get_total_deaths() const { return(sum.data[RI_DEATHS]); }
};


global_round_info::global_round_info(string robot_name) : sum(robot_name) {
	number_added = 0;
}

void global_round_info::add_information(const round_info & to_add, bool
		log_round) {
	if (log_round)
		past_records.push_back(to_add);
	sum += to_add;
	number_added++;
}

#endif
