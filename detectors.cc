// Radar and sonar
// Radar returns range to closest enemy robot (with no limit on range, but also
// no angle/ID/etc information). Sonar returns heading to closest enemy robot
// within range; either noisy or heavily quantized (ATR2 doc says +/- 32
// hexdegrees, but doesn't say whether there's randomness or just quantization).

// All return a negative value if nothing's in range. (DONE: Check what ATR2's
// radar does in this case; that is, just after the other one blows up. It
// returns -1.)

#ifndef _KROB_DETECT
#define _KROB_DETECT

#include "object.cc"
#include "random.cc"
#include <vector>
#include <list>

using namespace std;

// The check* functions also set cycle_last_*, so they can't be const. This is
// a side effect; the proper way would be to have these in robot.

class detector {
	private:
		int sonar_quant, sonar_maxrange;
		int last_detected_transponder;
		list<Unit *>::const_iterator closest_robot;

		int cycle_last_sonar, cycle_last_radar;
		coordinate pos_last_sonar, pos_last_radar;

		// If record_as_radar is false, *_last_radar isn't set; this
		// is used when we call radar from sonar.
		int internal_check_radar(const list<Unit *> & robots,
				const coordinate radar_pos,
				int current_cycle, bool record_as_radar);

		single_rand randomizer;

	public:
		detector(int sonar_q_in, int sonar_maxrange_in, int type,
				int rng_matchid);

		int check_radar(const list<Unit *> & robots, 
				const coordinate radar_pos,
				int current_cycle);
		int check_sonar(const list<Unit *> & robots,
				const coordinate sonar_pos,
				int current_cycle);

		int get_last_detected_transponder() const;

		int get_last_radar_time() const { return(cycle_last_radar); }
		int get_last_sonar_time() const { return(cycle_last_sonar); }

		int get_sonar_quant() const { return(sonar_quant); }
		int get_sonar_maxrange() const { return(sonar_maxrange); }

		coordinate get_last_sonar_pos() const { return(pos_last_sonar);}
		coordinate get_last_radar_pos() const { return(pos_last_radar);}
};

detector::detector(int sonar_q_in, int sonar_maxrange_in, 
		int matchid, int type) : randomizer(matchid, type, RND_SONAR) {
	sonar_maxrange = sonar_maxrange_in;
	sonar_quant = sonar_q_in;
	cycle_last_sonar = -1;
	cycle_last_radar = -1;
}

int detector::internal_check_radar(const list<Unit *> & robots, 
		const coordinate radar_pos, int current_cycle, bool
		record_as_radar) {
	// For all robots that's not our own, update record.
	// Return -1 if nobody else is around (shouldn't matter, but we're
	// completist).
	
	double record_distance = INFINITY;
	list<Unit *>::const_iterator record = robots.end();

	if (record_as_radar) {
		cycle_last_radar = current_cycle;
		pos_last_radar = radar_pos;
	}

	for (list<Unit *>::const_iterator cur = robots.begin(); cur !=
			robots.end(); ++cur) {
		if ((*cur)->get_pos() == radar_pos) continue;

		double this_distance = (*cur)->get_pos().sq_distance(radar_pos);

		if (this_distance < record_distance) {
			record_distance = this_distance;
			record = cur;
			last_detected_transponder = (*cur)->get_ID();
		}
	}

	closest_robot = record;

	if (record == robots.end()) return(-1);
	else return(sqrt(record_distance));
}

int detector::check_radar(const list<Unit *> & robots,
		const coordinate radar_pos, int current_cycle) {

	return(internal_check_radar(robots, radar_pos, current_cycle,
				true));
}

int detector::check_sonar(const list<Unit *> & robots, coordinate sonar_pos,
		int current_cycle) {
	// DONE: Make this set memory location 5, as ATR2 does (albeit
	// undocumented except for the "(for all scans)").
	// DONE: Don't do anything above 250 meters. If nothing was found,
	// return -1. DONE, although range has been parameterized.
	
	// First find the closest robot (by radar), then determine its heading
	// in a polar coordinate system with ourselves as center.
	// We can afford the excess because real robots will be calling the
	// sonar very seldomly.

	cycle_last_sonar = current_cycle;
	pos_last_sonar = sonar_pos;

	int closest_dist = internal_check_radar(robots, sonar_pos, 
			current_cycle, false);

	if (closest_dist == -1 || closest_dist > sonar_maxrange)
		return(-1); // nothing found

	// Else closest_robot is, yes, closest robot.
	
	coordinate normalized = (*closest_robot)->get_pos() - sonar_pos;

	// Get unfiltered angle.
	double angle = radian_to_hex(atan2(normalized.y, normalized.x));

	// Then distort it randomly (as ATR2 does) and return.
	// FIXED bug where random was signed and so blew up sonar_quant by 2.
	
	// Beware: setting this to double makes the return value become
	// 2^32 in some cases. I have no idea why that happens, but setting
	// it to int makes it go away. (Huh???)
	int impede = (randomizer.irand() % sonar_quant) - (sonar_quant / 2);
	return(round(normalize_hex(impede, angle)));
}

int detector::get_last_detected_transponder() const {
	return(last_detected_transponder); 
}

#endif
