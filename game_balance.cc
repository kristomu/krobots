// This class houses the #CONFIG game balance constants. They may be turned 
// into vars later.
// It also contains the speed and health penalty settings for hot robots.

// These soak up the LOCs, and I know of a better way. But later.. see
// something return and again?

// Relevant robot stats:
// 	Normal missile velocity: 32 m/c
// 	Maximum non-overburn speed: 4 m/c
// 	Turn rate: 8 hexdegrees per cycle
// 	Acceleration: 4% of max forward speed.

#ifndef _KROB_GBALANCE
#define _KROB_GBALANCE

#include <math.h>
#include <vector>
#include "configorder.h"
#include <assert.h>

using namespace std;

class game_balance {
	private:
		bool is_equal_setting(config_order referent, int original,
				int new_setting) const;
	public:
		int get_scanner_range(int points) const;
		double get_weapon_multiplier(int points) const;

		// More heavily armored robots have more armor (imagine that!)
		// but move more slowly.
		double get_armor_armor_factor(int points) const;
		double get_armor_speed_factor(int points) const;

		double get_engine_multiplier(int points) const;
		double get_heatsink_multiplier(int points) const;
		int get_num_mines(int points) const;
		int get_shield_type(int points) const;

		// Heat-related tradeoffs.
		double get_speed_penalty(double heat) const;
		double get_damage(double heat) const;

		// Default #config values
		vector<int> get_default_config_values() const;
		void complete_config_values(vector<int> & specified) const;
		void minimize_config_values(vector<int> & values) const;

		// Other defaults
		const double get_min_throttle() { return(-0.75); }
		const double get_max_throttle() { return(1.0); }
		const double get_speed_multiplier() { return(4.0); }
		// % of max throttle
		const double get_accel_value() { return(0.04); }
		const double get_turn_rate() { return(8); } // hexdegrees/cycle
		const double get_heat_shutdown() { return(400/500.0); }
		// How much it must cool down before it can work again
		const double get_heat_hysteresis() { return(50/500.0); }
		const double get_sonar_range() { return(250); }
		// Bluesky: figure out the value for "crazy missiles".
		const double get_missile_speed() { return(32); }
};

bool game_balance::is_equal_setting(config_order referent, int original, 
		int new_setting) const {

	switch(referent) {
		case CNF_SCANNER:
			return(get_scanner_range(original) == 
					get_scanner_range(new_setting));
		case CNF_WEAPON:
			return(get_weapon_multiplier(original) ==
					get_weapon_multiplier(new_setting));
		case CNF_ARMOR:
			return(get_armor_armor_factor(original) ==
					get_armor_armor_factor(new_setting) &&
					get_armor_speed_factor(original) ==
					get_armor_speed_factor(new_setting));
		case CNF_ENGINE:
			return(get_engine_multiplier(original) ==
					get_engine_multiplier(new_setting));
		case CNF_HEATSINKS:
			return(get_heatsink_multiplier(original) ==
					get_heatsink_multiplier(new_setting));
		case CNF_MINES:
			return(get_num_mines(original) ==
					get_num_mines(new_setting));
		case CNF_SHIELD:
			return (get_shield_type(original) ==
					get_shield_type(new_setting));
		default:
			return(false);
	}
}


int game_balance::get_scanner_range(int points) const {
	switch(points) {
		case 0: return(250);
		case 1: return(350);
		case 2: return(500);
		case 3: return(700);
		case 4: return(1000);
		case 5: return(1500);
		default: return(-1);
	}
}

double game_balance::get_weapon_multiplier(int points) const {
	switch(points) {
		case 0: return(0.5);
		case 1: return(0.8);
		case 2: return(1.0);
		case 3: return(1.2);
		case 4: return(1.35);
		case 5: return(1.5);
		default: return(-1);
	}
}

double game_balance::get_armor_armor_factor(int points) const {
	switch(points) {
		case 0: return(0.5);
		case 1: return(0.66);
		case 2: return(1.0);
		case 3: return(1.2);
		case 4: return(1.3);
		case 5: return(1.5);
		default: return(-1);
	}
}

double game_balance::get_armor_speed_factor(int points) const {
	switch(points) {
		case 0: return(1.33);
		case 1: return(1.2);
		case 2: return(1.0);
		case 3: return(0.85);
		case 4: return(0.75);
		case 5: return(0.66);
		default: return(-1);
	}
}

double game_balance::get_heatsink_multiplier(int points) const {
	double lookup[6] = { 0.75, 1.0, 1.125, 1.25, 1.33, 1.5 };
	if (points < 0 || points > 5) return(-1);
	return(lookup[points]);
}

double game_balance::get_engine_multiplier(int points) const {
	double lookup[6] = {0.5, 0.8, 1.0, 1.2, 1.35, 1.5 };
	if (points < 0 || points > 5) return(-1);
	return(lookup[points]);
}

int game_balance::get_num_mines(int points) const {
	int lookup[6] = {2, 4, 6, 10, 16, 24};
	if (points < 0 || points > 5) return(-1);
	return(lookup[points]);
}

int game_balance::get_shield_type(int points) const {
	int lookup[6] = {0, 0, 0, 1, 2, 3};
	if (points < 0 || points > 5) return(-1);
	return(lookup[points]);
}

// Heat is normalized so that 1.0 is 500, where the robot explodes.
double game_balance::get_speed_penalty(double heat) const {

	// The ATR2 doc states as follows for heat levels:
	// >= 80: (0.16)	Max speed is 98%
	// >= 100:(0.20)	Max speed is 95%
	// >= 150:(0.30)	Max speed is 85%
	// >= 200:(0.40)	Max speed is 70%
	// >= 250:(0.50)	Max speed is 50%
	// (Higher levels cause damage)
	
	if (heat < 0.16) return(1);
	if (heat < 0.20) return(0.98);
	if (heat < 0.30) return(0.95);
	if (heat < 0.40) return(0.85);
	if (heat < 0.50) return(0.70);
	return (0.50);
}

double game_balance::get_damage(double heat) const {
	// Damage is per cycle, and heat is normalized so that 1.0 is 500.
	// The ATR2 doc states the follows for damage due to heat:
	// >= 300:(0.60)	Heat starts burning armor off (the hotter the
	// 				quicker)
	// >= 500:(1.00)	Boom.
	
	// To emulate self-destruction, we inflict +INFINITY which will push
	// the health down below zero whatever it was. (We could inflict 1,
	// since health is normalized, but this makes it more clear).
	
	if (heat < 0.60) return(0);
	if (heat >= 1) return(INFINITY);

	// Okay, so we're in the "burn armor off" regime, but not to the extent
	// that we'll immediately self destruct.
	// The schedule is this: 
	// 	Heat >= 300: one unit (1/100) every 64 cycles	(0.015 * 0.01)
	// 	Heat >= 350: one unit every 32 cycles		(0.031 * 0.01)
	// 	Heat >= 400: one unit every 16 cycles		(0.0625* 0.01)
	// 	Heat >= 475: one unit every 4 cycles		(0.25 * 0.01)
	
	// BLUESKY would be to create a curve that approximates this, with 1 at
	// heat = 500.
	
	// Reformulated:
	// Heat >= 475 (0.95): 1/400.0
	// Heat >= 400 (0.8):  1/1600.0
	// Heat >= 350 (0.7):  1/3200.0
	// Heat >= 300 (0.6):  1/6400.0

	if (heat >= 0.95) return(1/400.0);
	if (heat >= 0.80) return(1/1600.0);
	if (heat >= 0.70) return(1/3200.0);
	return(1/6400.0);
}

// These give default #config values and merge them with user-specified ones,
// respectively. The order of the values is in standard FAQ format as defined
// by configorder.h.
vector<int> game_balance::get_default_config_values() const {
	vector<int> toRet(C_NUMDEVICES, -1);
	toRet[CNF_SCANNER] = 5;
	toRet[CNF_WEAPON] = 2;
	toRet[CNF_ARMOR] = 2;
	toRet[CNF_ENGINE] = 2;
	toRet[CNF_HEATSINKS] = 1;
	toRet[CNF_MINES] = 0;
	toRet[CNF_SHIELD] = 0;
	return(toRet);
}

void game_balance::complete_config_values(vector<int> & specified) const {
	
	vector<int> defaults = get_default_config_values();

	assert (specified.size() == defaults.size());

	for (size_t counter = 0; counter < specified.size(); counter++)
		if (specified[counter] == -1)
			specified[counter] = defaults[counter];
}

// Minimize config values: Used to emulate ATR2 "if the robot's ''cheating'', 
// ignore excess shield points that doesn't give it a shield" behavior under
// lenient compilation rules.
void game_balance::minimize_config_values(vector<int> & values) const {

	// This is generic so that if we alter the tradeoffs, it still works.
	
	vector<double> offset(C_NUMDEVICES, 0);

	int counter;
	
	for (counter = 0; counter < offset.size(); ++counter) {

		while (values[counter] - offset[counter] >= 0 &&
				is_equal_setting((config_order)counter,
					values[counter], values[counter] -
					offset[counter]))
			++offset[counter];

		--offset[counter]; // Counter off-by-one
	}

	for (counter = 0; counter < values.size(); ++counter)
		values[counter] = max(0.0, values[counter] - offset[counter]);

}

#endif

