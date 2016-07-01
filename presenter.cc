// Score presenting functions: show scores in ATR2 format or any of the /r s
// besides r4. Maybe Corewar format later?

// XYZ by Whoever scores N
// Results: v_against[0], v_against[1], v_against[2].. Nah.

#ifndef _KROB_PRESEN
#define _KROB_PRESEN

#include "global_stats.cc"
#include "tools.cc"
#include <math.h>
#include <string>

using namespace std;

class presenter {
	private:
		int num_len, namelen, score_len, win_len, match_len, armor_len,
		    kill_len, death_len, shots_len;

		string name_t, score_t, win_t, match_t, armor_t, kill_t,
		       death_t, shots_t;

		string separator, num_bind;

	public:
		presenter();
		string right_just(const string input, const int maxlen);
		string left_just(const string input, const int maxlen);
		string local_header();
		string single_summary(const round_info & source_local, 
				const round_info & source_global, int idx);
		string global_header();
		string global_summary(const round_info & source, int idx);
		string get_tournament_line(const round_info & source,
				int detail_level, const double sum_of_how_many);
};

presenter::presenter() {

	name_t = "Robot";
	score_t = "Scored";
	win_t = "Wins";
	match_t = "Matches";
	armor_t = "Armor";
	kill_t = "Kills";
	death_t = "Deaths";
	shots_t = "Shots";

	separator = "  ";
	num_bind = " - ";


	// Format info
	num_len = 3;	// 0 - 999, though there's no reason it couldn't be
			// larger. at eta 0.5, radius 4, 1500x1500 arena,
			// absolute number <= 10 000 or so.
	namelen = 10;
	score_len = 6; // 1m
	win_len = 6;
	match_len = 6;
	armor_len = 3; // 0-100
	kill_len = 6;
	death_len = 6;
	shots_len = 8;

	// Santiy checks to make right/left-adjustment always line up.
	namelen = max((size_t)namelen, name_t.size() - num_bind.size());
	score_len = max((size_t)score_len, score_t.size());
	win_len = max((size_t)win_len, win_t.size());
	match_len = max((size_t)match_len, match_t.size());
	armor_len = max((size_t)armor_len, armor_t.size());
	kill_len = max((size_t)kill_len, kill_t.size());
	death_len = max((size_t)death_len, death_t.size());
	shots_len = max((size_t)shots_len, shots_t.size());
}

string presenter::right_just(const string input, const int maxlen) {
	if (maxlen <= input.size()) return(input);
	string out(maxlen - input.size(), ' ');
	out += input;
	return(out);
}

string presenter::left_just(const string input, const int maxlen) {
	if (maxlen <= input.size()) return(input);
	string out(maxlen - input.size(), ' ');
	out = input + out;
	return(out);
}

string presenter::local_header() {
	string toRet = left_just("Robot", num_len + 3 + namelen) + separator +
		right_just("Scored", score_len) + separator +
		right_just("Wins", win_len) + separator +
		right_just("Matches", match_len) + separator +
		right_just("Armor", armor_len + 1) + separator +
		right_just("Kills", kill_len) + separator +
		right_just("Deaths", death_len) + separator +
		right_just("Shots", shots_len);

	return(toRet);
}

string presenter::single_summary(const round_info & source_local, 
		const round_info & source_global, int idx) {
	// Index, - , name, scored, wins, matches, armor, kills, deaths, shots.
	
	// [Wins, matches, kills, deaths] are global. The others are local.
	
	long long wins = source_global.get_one(RI_VICTORY),
	    matches = source_global.get_one(RI_RUNS),
	    kills = source_global.get_one(RI_KILLS),
	    deaths = source_global.get_one(RI_DEATHS);

	double armor = source_local.get_one(RI_ENDARMOR);

	long long scored = source_local.get_one(RI_VICTORY),
	    shots = source_local.get_one(RI_SHOTSFIRED);

	// Truncate robot name if there isn't any space for the full name.
	string trunc_robot_name = source_local.robot_name;
	if (trunc_robot_name.size() > namelen) {
		trunc_robot_name.resize(namelen-3);
		trunc_robot_name += "...";
	}

	string toRet = right_just(itos(idx), num_len) + num_bind + 
		left_just(trunc_robot_name, namelen) + separator +
		right_just(lltos(scored), score_len) + separator +
		right_just(lltos(wins), win_len) + separator +
		right_just(lltos(matches), match_len) + separator +
		// Armor etc..
		right_just(itos(armor * 100), armor_len) + "%" + separator +
		right_just(lltos(kills), kill_len) + separator +
		right_just(lltos(deaths), death_len) + separator +
		right_just(lltos(shots), shots_len);

	return(toRet);
}

string presenter::global_header() {

	string toRet = left_just("Robot", num_len + num_bind.size() + namelen) 
		+ separator + right_just("Wins", win_len) + separator +
		right_just("Matches", match_len) + separator +
		right_just("Kills", kill_len) + separator +
		right_just("Deaths", death_len) + separator +
		right_just("Shots", shots_len);

	return(toRet);
}

string presenter::global_summary(const round_info & source, int idx) {

	long long wins = source.get_one(RI_VICTORY),
	     matches = source.get_one(RI_RUNS),
	     kills = source.get_one(RI_KILLS),
	     deaths = source.get_one(RI_DEATHS),
	     shots = source.get_one(RI_SHOTSFIRED);

	string trunc_robot_name = source.robot_name;
	if (trunc_robot_name.size() > namelen) {
		trunc_robot_name.resize(namelen-3);
		trunc_robot_name += "...";
	}

	string toRet = right_just(itos(idx), num_len) + " - " +
		left_just(trunc_robot_name, namelen) + separator +
		right_just(lltos(wins), win_len) + separator +
		right_just(lltos(matches), match_len) + separator +
		right_just(lltos(kills), kill_len) + separator +
		right_just(lltos(deaths), death_len) + separator +
		right_just(lltos(shots), shots_len);

	return(toRet);
}

// /R :  Wins Trials Name
// /R2:  Wins Trials Kills Deaths Name
// /R3:  Wins Trials Kills Deaths EndingArmor EndingHeat ShotsFired Name
// /R4:  Wins Trials Kills Deaths EndingArmor EndingHeat ShotsFired Hits 
// 		DamageTotal CyclesLived ErrorCount Name
string presenter::get_tournament_line(const round_info & source,
		int detail_level, const double sum_of_how_many) {

	long long wins = source.get_one(RI_VICTORY),
	     trials = source.get_one(RI_RUNS),
	     kills = source.get_one(RI_KILLS),
	     deaths = source.get_one(RI_DEATHS),
	     armor = (source.get_one(RI_ENDARMOR)/sum_of_how_many) * 100,
	     heat = (source.get_one(RI_ENDHEAT)/sum_of_how_many) * 500,
	     shotsfired = source.get_one(RI_SHOTSFIRED),
	     hits = source.get_one(RI_HITS),
	     damagetotal = source.get_one(RI_DAMAGE) * 100,
	     cycleslived = source.get_one(RI_LIFESPAN),
	     errorcount = source.get_one(RI_ERRORS);

	string name = source.robot_name;

	string report = "";

	if (detail_level < 1) detail_level = 1;
	if (detail_level > 4) detail_level = 4;

	// Fallthrough makes this shorter.
	switch(detail_level) {
		case 4:
			report = lltos(hits) + " " + lltos(damagetotal) + " " +
				lltos(cycleslived) + " " + lltos(errorcount) 
				+ " " + report;
		case 3:
			report = lltos(armor) + " " + lltos(heat) + " " + 
				lltos(shotsfired) + " " + report;
		case 2:
			report = lltos(kills) + " " + lltos(deaths) + " " +
				report;
		case 1:
			report = lltos(wins) + " " + lltos(trials) + " " +
				report;
			break;
	}

	report += name;

	return(report);
}

#endif
