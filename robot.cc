// Relevant robot stats: 
// Normal missile velocity: 32 m/s
// Maximum non-overburn speed: 4 m/s
// Turn rate: 8 hexdegrees
// Acceleration: 4% of max forward speed.
// 		Unless overburn or config, that's 4% of 4.

// For the arena, should we have list<robot> instead of vector<robot>? Would
// make it simpler to do collision detection, etc. I think that would be
// a good idea; however, there's one dealbreaker. When we resize windows after
// a robot is dead, we must redraw the stats of the dead robot. However, we
// do not have the information if we just eliminate it from the list.
// (If vector turns out to be unacceptably slow, we can store the dead ones
//  in a separate list, but this gets complex and thus we won't do it now.)

// This will eventually become a fully fledged robot class. The reason we use
// .h here instead of the usual convenient .cc is that Scanner refers to robot
// and vice versa, and we can't include the code twice.

// The transmissions system is a bit icky, I'd say, and we can't remove someone
// from a channel yet.

// BLUESKY: "GAME_RULES" class for the various constants. [DONE for some]

// Overburn effects:
// 	faster driving speed (130%)	IMPLEMENTED	(advance_internally)
// 	weapon does more damage(125%)	IMPLEMENTED	(get_damage_multiplier)
// 	projectiles move faster(125%) 	IMPLEMENTED	(fire)
// 	weap generates more heat(150%)	IMPLEMENTED	(fire)
//	nastier death boom (130%)	IMPLEMENTED	(die)
//	worse heat dissipation (66%)	IMPLEMENTED	(cool_down)

// It might be better to set heat and armor to unnormalized (that is, 0..500
// and 0..100 respectively). Also check if the heat box is drawn to something >
// 500 in "real" ATR2.

// DONE: Init last_successful_scan properly (to preemptively avoid valgrind
// errors).

// DONE: Check that these clocks stay synchronized with the global clock.
// Maybe something like (for each robot, assert get_time() == global_time)
// then fix until it goes away.

// Unchecked: Possible problems when there's nonzero scanner residue: the
// scanner and fire checks may behave oddly, and hit outside of where we fired.
// We'll be consistent by not adding residue to the firing angle resolution,
// but it may make the scan jump around if keepshift is off.

#ifndef _KROB_ROBOT
#define _KROB_ROBOT

#include "object.cc"
#include "color.cc"
#include "scanner.cc"
#include "detectors.cc"
#include "missile.cc"
#include "game_balance.cc"
#include "blast.cc"
#include "mine.cc"
#include "comms.cc"
#include "configorder.h"
#include "global_stats.cc"
#include <assert.h>
#include <set>

// This resides here instead of in Scanner because position and clock aren't
// particular to the scanner but the robot.
typedef struct scanner_result {
	coordinate position;	// Where we were when we scanned
	int span;		// Span at the time of scanning
	int clock;		// When we scanned
	unsigned char angle;	// "Heading" of scanner.
	unsigned char our_angle; // Our heading.

	// These parameters aren't provided directly to the programmer or the
	// code running on the robot. Undefined if !found.
	unsigned char target_angle; // absolute
	double target_throttle;
	double target_abs_speed;
	short target_transponder;

	int range;		// range to target
	int accuracy;		// Sector at which it was found
	bool found;		// Whether anything was actually found.
};

// DONE: Routine updates (heat dissipation, etc.)
// DONE: Sonar [DONE], and keepshift off.

const int DMG_UNSPEC = -1; // Damage from unspecified or ourselves.

class robot : public Unit {

	private:
		int UID;	// not modifiable. Robust way of ensuring
				// missiles and mines don't blow up on owner
				// (IFF).

		// Init parameters (construction details)
		string name, message;
		int shield_type;	// Uncoupled - no setters yet
		bool old_shield;	// If true, invulnerable when shield's
					// on.
		int CPU_speed;		// In CPU cycles per game cycle.
		double armor_thickness; // 1 = standard armor, 2 = half damage
		double heatsink_factor; // 1 = standard, 2= cools down at 2x 
		double weapon_power;    // multiplier to damage, speed, heat
		double base_engine_speed; // Max speed, neither encumbered by
					  // heat nor enhanced by overburn.
		double base_missile_speed; // ditto

		double CPU_time_residue; // Used for any leftover time, so that
					 // the CPU eventually gets all the
					 // cycles due to it.

		// We store both last_scan and last_successful scan because of
		// ATR2 memory positions that outputs things like "Relative
		// heading of last target scanned".
		scanner_result last_scan, last_successful_scan;
		Scanner scan_sensor;
		detector radar_sonar;
		int last_detected_transponder; // also set upon sonar and radar.
		double clock;		// Counts cycles that have passed.
					// Note ATR2 doc about wraparound in
					// memory.
		// Random number generator
		single_rand hardware_rand;

		// Body stats. These are normalized to 0...1 per ATR2; maybe
		// something clever with max[heat,health] and cur[heat,health]
		// later? No, we must keep backwards compatibility with
		// port 6.
		double armor;
		double heat;		// normalized to 0...1 
					//	(but what of CONFIG?)
		double shutdown_temperature;
		double shutdown_margin;
		// More stats (whatever they might be) here.
		// Keepshift not implemented yet. DONE: Implement! Needs to
		// be done before we can test.
		bool overburning, shields_up, keepshift;
		round_info local_stats;
		const global_round_info * global_stats;	// updated afterwards
		//int local_kills, kills, deaths, wins;
		int last_damaged_by, last_damaged_at;
		int was_killed_by;	// -1 if nobody or self, otherwise
					// ID of other, as passed by
					// inflict_damage
		int crashes;
		// These are used for checking whether we're permitted to
		// deploy a mine, and to be able to inform the program how
		// many mines we do have, without having to do a traversal of
		// the entire list.
		int mines_available, mines_deployed;

		// Comms. The default comms channel is the UID, which is also
		// the default transponder value (you can't change UID but you
		// can change transponder).
		unsigned short comms_channel;

		// History parameters.
		// transp_last_impacted is the transponder ID of whatever bot 
		// was last hit by a missile; transp_last_blown is the analogy 
		// for mines.
		// If more than one were hit, which is recorded is undefined.
		// Note that "blown" doesn't imply "killed". 
		int transp_last_impacted, transp_last_blown, last_hit_at, 
		    last_blown_at;

		// This may have to be fixed later to abstract interpreter
		// properly, but then we just have to change the get_error
		// functions.
		int last_error;		// Set by the interpreter.
		bool error;

		// Code and data here.
		
		// Meta.
		scanner_result aggregate_scan_info();

		// This is for the path-dependence of the shutdown function,
		// which turns on at a certain level and then keeps the CPU
		// and body shut off until it falls below a certain other
		// level.
		bool has_shutdown;

		// For CPU interconnection.
		double CPU_cycles_available;
		int CPU_cycles_per_cycle;

		// Miscellaneous: surrogate scan_result for sonar
		int last_sonar_val;

		// Cached collision data. -1 if we don't know (set to -1 if we
		// alter speed or direction).
		double time_of_edge_collision;

	public:
		bool is_CPU_working() const; // For CPU interconnection

		// 32 is default, I think. But should that be here?
		// But what's the initial frequency?
		robot(int matchid, int UID_in, string namei, double smin, 
				double smax, double multiplier, double dps, 
				double ups, double start_heading, 
				double start_throttle, int radius, 
				int scanrange, int detrange,  int mines_avail, 
				int CPU_speed_in, double shutdown_temp, 
				double shutdown_m, int sonar_range, double
				missile_speed, bool old_shield_in,
				global_round_info * global_stat);

		void set_abs_edge_collision_time(double absolute_time) {
			time_of_edge_collision = absolute_time; }
		void set_rel_edge_collision_time(double relative_time);
		double get_edge_collision_time() const { 
			return(time_of_edge_collision); }

		void reset_software(vector<short> & memory);

		// Adjust game balance based on the points information and
		// balancer.
		void adjust_balance(vector<int> & points, game_balance & 
				balancer);

		int get_UID() const { return(UID); }

		unsigned char turret_heading;	// Updates instantly.
		double turret_residue; // For keepshift

		void adjust_peripherals(const double old_heading);

		comms comms_queue;	// Is this bad, exposing internals?
		Color assigned_color, shield_color;

		void set_colors(Color body, Color shield);

		// Settings
		// DONE: Set defaults on constructor.
		void set_armor_thickness(double fact) { armor_thickness = fact;}
		void set_heatsink_factor(double fact) { heatsink_factor = fact;}
		void set_weapon_power(double fact) { weapon_power = fact; }
		void set_shield_type(int type) { shield_type = type; }
		void set_num_mines(int avail) { mines_available = avail; }

		double get_armor_thickness() const { return(armor_thickness); }
		double get_heatsink_factor() const { return(heatsink_factor); }
		double get_weapon_power() const { return(weapon_power); }
		double get_shield_type() const { return(shield_type); }

		// Game technical events.
		void heat_up(const double amount);
		void cool_down(const double amount);

		// These return the damage actually taken.
		double inflict_damage(bool blockable, double amount, 
				int origin);
		double register_blast(double distance, double multiplier, 
				int max_range, int origin);

		// Register crashes, destroyed mines, and turning.
		void register_crash(double relative_time);
		void register_destroyed_mine();

		void receive_transmission(unsigned short data);

		// .. and consequences
		bool die(blasts & explosions, vector<robot> & other_robots);
			// returns false if already dead.

		// Update internal parameters like heat (cool off). This will
		// also include CPU runs later.
		bool advance_internally(const double time_elapsed, 
				const game_balance & heat_balancer, 
				blasts & explosions, vector<robot> & robots);
		int withdraw_CPU_cycles(); // Get available CPU cycles and
						// set those as used.

		// Sensor-reading functions
		bool do_scan(const list<Unit *> & active_robots);
		bool do_scan(const list<Unit *> & active_robots, int span);
		bool get_scan_hit() const; // True if the scan found anything

		int do_radar(const list<Unit *> & active_robots);
		int do_sonar(const list<Unit *> & active_robots);

		// Passing through - used for showing when a sonar's called.
		int get_last_radar_time() const;
		int get_last_sonar_time() const;
		int get_last_sonar_angle() const { return(last_sonar_val); }
		coordinate get_last_sonar_pos() const;
		int get_sonar_maxrange() const;
		
		// If any of these have hit = true, returns information for last
		// target, else last scan.
		int get_scan_dist(bool hit) const; // Dist if anything found, or 32767
		int get_accuracy(bool hit) const;
		int get_scan_span(bool hit) const;
		int get_scan_time(bool hit) const;
		int get_scan_center(bool hit) const; // Where the scanner was 
					             // aimed at last scan
		int get_scan_body_heading(bool hit) const; // Where we were
							// aimed then,
		coordinate get_scan_pos(bool hit) const; // and where we were  
						         // at the time.

		double get_last_target_throttle() const;
		int get_last_target_heading() const;
		short get_last_target_ID(bool any_scan) const;
		double get_last_target_abs_speed() const;

		void set_scanrange(int scanrange);
		int get_scanrange() const { return(scan_sensor.get_radius()); }
		void set_scan_span(int span);
		void set_scan_drange(int det_range); // detection range

		void set_missile_speed(double mspd);
		double get_missile_speed() const { return(base_missile_speed); }
		
		// Other sensors.
		void set_clock(int clock_signal);
		void tick(double delta);        // advance the clock

		unsigned short usrand() { return(hardware_rand.irand()); }
		double get_time() const;
		int get_time_int() const;
		int get_num_crashes() const { return(crashes); }
		void reset_crash_count() { crashes = 0; }
		double get_armor() const { return(armor); }
		double get_heat() const { return(heat); }
		int get_mines_available() const { return(mines_available); }
		int get_mines_deployed() const { return(mines_deployed); }

		int get_last_damaged_by() const { return(last_damaged_by); }
		int get_last_damaged_at() const { return(last_damaged_at); }

		// Messages both new and old..
		bool get_last_message(unsigned short & output_loc); // -1 if no
		// These aren't implemented, but don't seem to be needed
		/*
		bool get_earlier_message(int pos, unsigned short & output_loc) const;*/
		// UNTESTED as of 2008-05-06.
		int get_num_waiting_messages() const;
		void null_queue() { comms_queue.null(); }

		// Actuators
		bool is_overburning() const { return(overburning); }
		bool has_keepshift() const { return(keepshift); }
		bool is_shielded() const { return(shields_up); }
		double get_upper_shutdown_temp() const {return(shutdown_temperature);}
		double get_lower_shutdown_temp() const {return(max(0.0, shutdown_temperature - shutdown_margin)); }

		bool set_shields(bool up);
		void set_keepshift(bool state) { keepshift = state; }
		void set_overburn(bool state) { overburning = state; }
		void set_shutdown_temp(double hot) {shutdown_temperature = hot;}
		void set_shutdown_margin(double m) {shutdown_margin = m;}

		bool fire(list<missile> & add_to, int offset_to_turret);
		bool deploy_mine(list<mine> & add_to, int radius);

		// Now works.
		void remove_communications_link(vector<set<robot *> > & lookup);
		void set_communications_channel(int new_channel, 
				vector<set<robot *> > & lookup);
		int get_communications_channel() const{ return(comms_channel); }
		void transmit(unsigned short signal, vector<set<robot *> > &
				lookup);

		// Inter-round data.
		// DONE: Check that record_death/record_kill/record_victory are
		// actually called!
		// Now kill is called as long as the source IDs for ordnance is
		// properly set, and record_death is called on death. Victory 
		// must be called by the match-keeper.
		void record_death();
		// Call it for those that survive to the end. See func for
		// parameter explanation.
		void record_end_stats(bool record_true_age, int maxcycles);
		void record_damage_to_others(double how_much);
		void record_damage_to_others(const robot & who, 
				double how_much);
		void record_kill() { local_stats.data[RI_KILLS]++; }
		void record_victory() { local_stats.data[RI_VICTORY]++; }
		void record_error() { local_stats.data[RI_ERRORS]++; }
		void increment_round_count() { local_stats.data[RI_RUNS]++; }
		void record_bot_shot(const Unit * other_bot);
		void record_bot_mine_hurt(const Unit * other_bot);
		void record_killed_by(int src) { was_killed_by = src; }

		// DONE: Figure which of these are global and which are local.
		// // shots_hit is local, mines_hit also!
		// Get_all_x is all /for this robot/.
		int get_local_deaths() const;
		int get_local_kills() const;
		int get_local_shots_fired() const;
		int get_local_shots_hit() const;
		int get_local_mines_hit() const;

		round_info get_local_stats() const { return(local_stats); }

		int get_all_victories() const;
		int get_all_deaths() const;
		int get_all_kills() const;
		int get_all_shots_fired() const;
		int get_all_shots_hit() const;
		int get_all_mines_hit() const;

		int get_last_hit_other_at() const { return(last_hit_at); }
		int get_last_blown_at() const { return(last_blown_at); }
		int get_ID_last_impacted() const {return(transp_last_impacted);}
		int get_ID_last_blown() const { return(transp_last_blown); }
		
		bool killed_by_other() const { return(was_killed_by != -1); }
		int get_killer_ID() const { return(was_killed_by); }

		// Name and message.
		void set_name(string name_in) { name = name_in; }
		string get_name() const { return(name); }
		void set_message(string message_in) { message = message_in; }
		string get_message() const { return(message); }

		// Debug info.
		bool has_error() const { return(error); }
		int get_last_error() const { return(last_error); }
		void update_error(int new_error);

		// Debug commands
		void set_heat(double h_in) { heat = h_in; }
		void set_armor(double a_in) { armor = a_in; }

};

	// PRIVATE

bool robot::is_CPU_working() const { 
	return(!has_shutdown);
}

	// PUBLIC

robot::robot(int matchid, int UID_in, string namei, double smin,
		double smax, double multiplier, double dps, double ups, 
		double start_heading, double start_throttle, int radius,
		int scanrange, int detrange, int mines_avail, int CPU_speed_in, 
		double shutdown_temp, double shutdown_m, int sonar_range, 
		double missile_speed, bool old_shield_in,
		global_round_info * global_stat) :
	Unit(smin, smax, multiplier, dps, ups, start_heading, start_throttle, 
			radius, UID_in, false), 
	radar_sonar(64, sonar_range, matchid, UID_in), 
	hardware_rand(matchid, UID_in, RND_ROBOT), local_stats(namei),
	comms_queue(256) {

	UID = UID_in; 
	// Set scanner parameters.
	set_scan_drange(detrange); 
	set_scanrange(scanrange); 
	set_scan_span(8); 

	name = namei;
	error = false;
	last_error = 0;
	
	// Set relative clock and other physical parameters.
	set_shutdown_temp(shutdown_temp);
	set_shutdown_margin(shutdown_m);
	set_keepshift(false);
	set_clock(0);

	armor = 1; 
	heat = 0; 
	armor_thickness = 1; 
	heatsink_factor = 1;
	base_engine_speed = multiplier;
	base_missile_speed = missile_speed;
	is_dead = false;

	mines_available = mines_avail;
	mines_deployed = 0;

	// Set status
	global_stats = global_stat;
	overburning = false; shields_up = false; has_shutdown = false;
	last_damaged_at = 0; // not damaged yet
	last_hit_at = 0;
	last_blown_at = 0;
	transp_last_impacted = 0;
	transp_last_blown = 0;
	last_damaged_by = UID_in;
	was_killed_by = -1;
	time_of_edge_collision = -1;
	last_sonar_val = -1;
	last_detected_transponder = 0;
	reset_crash_count();

	// Set default comms and CPU stats.
	comms_channel = UID;
	CPU_speed = CPU_speed_in; 
	CPU_time_residue = 0;
	CPU_cycles_available = 0;

	// Assume most barebones robot; rest will have to be set by 
	// adjust_balance.
	set_shield_type(0);
	old_shield = old_shield_in;
	
	// Clean up some structs and internal variables.
	bzero(&last_scan, sizeof(last_scan));
	bzero(&last_successful_scan, sizeof(last_successful_scan));
	turret_residue = 0;
}

void robot::set_rel_edge_collision_time(double relative_time) {
	set_abs_edge_collision_time(relative_time + get_time());
}

void robot::reset_software(vector<short> & memory) {
	// According to ATR2, resetting the software implies:
	// - memory and stack is cleared
	// - Scan span set to a default of 8
	// - Desired heading is set to current heading, and desired speed to 0
	// - Relative turret offset set to 0, last error set to none
	// - Overburn and keepshift is disabled
	// - IP (instruction pointer) set to beginning, last accuracy set to 0
	// - Meters traveled set to 0
	// - Software gets ready to run on the next turn
	// - Shields down.
	for (size_t counter = 0; counter < memory.size(); ++counter)
		memory[counter] = 0;
	CPU_cycles_available = 0; // so that zero_penalty doesn't go into
				  // infinite loop.
	// Stack here -- but that's just part of memory.
	set_scan_span(8);
	set_desired_heading(get_heading());
	set_desired_throttle(0);
	turret_heading = get_heading();
	set_overburn(false); set_keepshift(false);
	error = false;
	zero_odometer();
	// Software gets ready to run on the next turn - here.
	// use CPU::zero_penalty()
	set_shields(false);
}


// Given a points distribution (no check is done with regards to whether there
// are too many or too few points), we set the appropriate multipliers according
// to what the game balancer tells us.
void robot:: adjust_balance(vector<int> & points, game_balance & balancer) {
	// The ordering is this (as per FAQ): Scanner Weapon Armor Engine
	// 					Heatsinks Mines Shield.
	
	set_scanrange(balancer.get_scanner_range(points[CNF_SCANNER]));
	set_weapon_power(balancer.get_weapon_multiplier(points[CNF_WEAPON]));
	set_armor_thickness(balancer.get_armor_armor_factor(points[CNF_ARMOR]));
	// Perhaps in a separate function?
	// Heh, I forgot to multiply by base_engine_speed. No wonder it was
	// so slow.
	set_speed_multiplier(balancer.get_armor_speed_factor(points[CNF_ARMOR])
			* balancer.get_engine_multiplier(points[CNF_ENGINE])
			* base_engine_speed);
	set_heatsink_factor(balancer.get_heatsink_multiplier(
				points[CNF_HEATSINKS]));
	set_num_mines(balancer.get_num_mines(points[CNF_MINES]));
	set_shield_type(balancer.get_shield_type(points[CNF_SHIELD]));
}

// Adjust turret_heading (which is an absolute value) to account for keepshift
// being off, if it is (in which case the turret stays the same relative to the
// body). We might need to refactor to have it work the other way around.
void robot::adjust_peripherals(double old_heading) {
	// Ugly hack with residue; the point is to let the program address
	// the turret in ints even with keepshift off.
	
	if (has_keepshift()) return;

	double difference = get_heading() - old_heading;
	if (difference == 0) return;
	int floordiff = floor(difference);

	turret_residue += (difference - floordiff);
	int floor_resid = floor(turret_residue);
	turret_residue -= floor_resid;

	turret_heading += floordiff + floor_resid;
}

void robot::set_colors(Color body, Color shield) {
	assigned_color = body;
	shield_color = shield;
}

// After doing a scan, this constructs the last_scan struct that we'll be using
// for miscellaneous getters and setters later on, so that we can refer to the
// position/etc at the point of scan rather than at the current time.
scanner_result robot::aggregate_scan_info() {
	scanner_result toRet;

	toRet.position = get_pos();
	toRet.span = scan_sensor.get_span();
	toRet.clock = get_time_int();
	toRet.angle = turret_heading;
	toRet.our_angle = get_heading();

	toRet.range = scan_sensor.get_range();
	// BLUESKY: Make accuracy tunable. This is set to BLUESKY because
	// it would involve the inverse of the zeta function.
	toRet.accuracy = scan_sensor.get_accuracy(); 
	toRet.found = scan_sensor.detected();

	// Update target data. BLUESKY: Store pointer to the scanned target
	// instead and query it directly; more modular.
	
	// Target data required
	const Unit * target = scan_sensor.target();
	if (target != NULL) {
		toRet.target_angle = target->get_heading();
		toRet.target_throttle = target->get_throttle();
		toRet.target_transponder = target->get_ID();
		toRet.target_abs_speed = target->get_absolute_speed();
	}
	//unsigned char target_angle; // absolute
	//        double target_throttle;
	//	        short target_transponder;


	return(toRet);
}

void robot::heat_up(const double amount) { 
	heat = min(1.0, heat + amount);		// 1 corresponds to 500.

	// If we passed the boundary, shut down!
	has_shutdown = (heat > get_upper_shutdown_temp());

}

void robot::cool_down(const double amount){

	double adjusted_heatsink_factor = heatsink_factor;
	// If we're overburning, we dissipate heat at 66% speed of usual.
	if (is_overburning()) 
		adjusted_heatsink_factor *= 2 / 3.0;

	// Can't cool down when shielded
	if (!is_shielded()) {
		heat -= amount * adjusted_heatsink_factor;
		if (heat < 0) heat = 0;
	}
}


// This whittles down armor. If blockable is true, it'll convert as required
// into heat if we're shielded, otherwise the damage is inflicted with no
// escape.
// DONE: Account for armor differences with differing CONFIGs. ATR2 just
// divides damage by a factor, but this will cause problems here since we
// use inflict_damage for crash damage (which is always 1 point).
// (Is it always 1 point? I think ATR2 always inflicts 1 point, too. It does,
// 	though the intention of the code is not to, just that the rounding
// 	rounds up to 1 if 0 < damage < 1.)
//  Now it does (amount / armor_thickness) down when calculating actual damage,
//  including, yes, crashes.
// DONE: Figure out if the heatsink factor is applied prior to shields or
// after it (i.e, does shields heat up more quickly with thin armor?).
// Makes most sense for it not to, but we don't know the quirks of ATR2.
// 	ATR2 is sensible here.

// Returns the damage actually inflicted, which means RI_DAMAGE counts net,
// and not gross (unlike ATR2).

double robot::inflict_damage(bool blockable, double amount, int origin) {
	if (amount == 0) return(0);	// No point in tinkering if it does no
					// damage!
	// The following is quite rational and also prevent ourselves from
	// "taking damage" from our death explosion.
	if (dead()) return(0);		 

	if (blockable && shields_up) {
		// Okay, so we have shields to block our pain. The shield
		// configurations in ATR2 are:
		//
		// Weak:	2/3 damage gets through, 2/3 goes to heat
		// Medium:	1/2 damage gets through, 1/2 goes to heat
		// Strong:	1/3 damage gets through, 1/3 goes to heat
		//
		// Although these have configs 3, 4, 5, they're set to 1,2,3
		// here for extensibility reasons.
		//
		// But note that our heatsink is normalized so that 500 = 1
		// while health is normalized so that 100 = 1. Fix that.
		// [DONE, though messily.]

		double renorm_damage = amount * 1 / 5.0;

		// If it's an old shield, we feel no damage and don't heat up
		// from being hit. I wonder if ATR2 shields mine explosion
		// damage.. It does.
		// (For that matter, the ATR2 docs say that the shield work
		//  by dissipating missile plasma. But then, why would it
		//  defend against mines?)
		if (old_shield)
			renorm_damage = 0;

		switch(shield_type) {
			case 1: heat_up(renorm_damage * 2/3.0);
				return(inflict_damage(false, amount * 2/3.0, 
							origin));
			case 2:	heat_up(renorm_damage * 1/2.0);
				return(inflict_damage(false, amount * 1/2.0, 
							origin));
			case 3:	heat_up(renorm_damage * 1/3.0);
				return(inflict_damage(false, amount * 1/3.0, 
							origin));
		}
	}

	// Otherwise, just decrease, clamping to zero if required.
	// Attenuation based on armor config goes here. [DONE]
	
	// First check if it's already dead or likely to be. If it's dead
	// or as good as (negative or zero armor), then we shouldn't register
	// this hit as the one that killed the robot, no matter who did it.
	bool applicable = (armor > 0 && !dead());

	armor = max(armor - amount / armor_thickness, 0.0);

	// DONE: Check that this doesn't change when a mine on the other end
	// of the map detonates.
	
	if (amount > 0 && origin != DMG_UNSPEC) {
		last_damaged_by = origin;
		last_damaged_at = clock;
		if (applicable) {
			// If armor is now <= 0, which means we'll die,
			// then register that we were killed.
			if (armor <= 0) 
				record_killed_by(origin);
		}
	}

	if (applicable)	return(amount);
	else		return(0);
}

// This is called when a blast hits the robot from some distance. 
// The max_range specifies the distance above which no damage occurs.
// (More realistic simulation would have inverse square or whatnot, but we'll 
//  emulate ATR2 first.)
// Returns the damage inflicted.
double robot::register_blast(double distance, double multiplier, int max_range,
		int origin) {
	double points = max_range - distance;

	if (points < 0) return(0); // No damage inflicted.

	return(inflict_damage(true, (multiplier * points) / 100.0, origin));
}


// This function reduces velocity and desired velocity to zero upon a crash
// (just as is done in Mover), inflicting one point (1/100) of direct 
// (unblockable) damage.

void robot::register_crash(double relative_time) {
	// Increment crash counter.
	crashes++;

	// Were we going too fast?
	// BLUESKY: Replace this with something vectory. If relative throttle
	// between the two (or even better, relative speed) is above a certain
	// level, boom. That way, a throttle 0.8 bot rear-ending a throttle 0.5
	// bot doesn't give you unintuitive results. This scenario is unlikely,
	// though, and probably not worth the cycles.
	if (get_throttle() > 0.5) {
		// Yes, inflict 1 unblockable damage from whoever we crashed
		// with, or ourselves if it's the wall.
		int source = get_crash_origin();
		if (source == CRASH_EDGE) source = get_UID();

		inflict_damage(false, 1/100.0, source);
	}

	// Similarly, if we crashed into something (and crashing is reciprocal,
	// so it'll be triggered here, though that breaks realism somewhat),
	// credit ourselves with the damage it'll do.
	if (get_crash_origin() != CRASH_EDGE)
		if (get_crash_target_throttle() > 0.5)
			record_damage_to_others(1/100.0);

	crash_halt(); // Stop moving.
}

// This is called when a mine belonging to this robot is destroyed, to keep
// count of how many mines remain. DONE: actually call this.
void robot::register_destroyed_mine() {
	assert(mines_deployed > 0);
	mines_deployed--;
}

void robot::receive_transmission(unsigned short data) {
	comms_queue.add(data);
}

bool robot::die(blasts & explosions, vector<robot> & robots) {
	// When we die, this happens:
	// Health (armor) is set to zero. So is heat, and we immediately stop.
	// Also, the dead parameter is set to true, we generate an explosion
	// at our current location, and credit the robot that killed us if it
	// wasn't ourselves (suicides don't count in ATR2).
	// BLUESKY: Move the "inflict splash damage" thing elsewhere so we don't
	// have to reimplement and reimplement. The problem here is that if we
	// put it inside blast, blast needs to refer to vector<robot>, but
	// robot needs to refer to blasts, creating a loop.
	
	if (dead()) return(false);

	//cout << "Dammit, I'm dying here." << endl;

	record_death();
	crash_halt(); // .. Kinda. 

	// Warning: ATR2 constant here.
	double max_hit_radius = 25;
	double multiplier = 1;
	if (is_overburning()) multiplier = 1.3;

	explosions.add_blast(get_pos(), B_ROBOT, max_hit_radius);

	// Attribution is the killer if we were killed by someone, otherwise
	// self. This makes sure that if A shoots B which explodes and destroys
	// C, then A gets credit for both kills.
	int attribution = was_killed_by;

	// ??? Should there be a time designator here? "Seconds elapsed"?
	// But then we have to know the exact moment it died. Note this
	// when doing asynchronous checks in a future version. (Might be easier
	// just to asynch check on collisions, not on anything else)
	for (size_t counter = 0; counter < robots.size(); ++counter)
		record_damage_to_others(robots[counter], robots[counter].
				register_blast(get_pos().distance(
						robots[counter].get_pos()), 
					multiplier, max_hit_radius, 
					attribution));

	//cout << "Boom." << endl;

	bool found_killer = false;

	if (was_killed_by != get_UID())
		for (size_t counter = 0; counter < robots.size() && 
				!found_killer; ++counter)
			if (robots[counter].get_UID() == was_killed_by) {
				robots[counter].record_kill();
				found_killer = true;
			}

	armor = 0;
	return(true);
}

// Advance the internal state: cool down and run program.
// Note that all of this must be rephrased for the robot to work in an
// asynchronous regime - especially with regard to speed changes.
// Isn't it a bit iffy to give access to the v<robot> array here? But to do
// it any other way requires an external structure for explosions, not just
// explosion effects.
bool robot::advance_internally(const double time_elapsed, const game_balance &
		heat_balancer, blasts & explosions, 
		vector<robot> & other_robots) {

	if (dead()) return(false);

	double base_bonus;

	if (is_overburning()) 
		base_bonus = 1.30;
	else	base_bonus = 1;

	// Adjust heat. If we're shielded, heat up by internal_heat, else cool
	// by internal_cool. These are fitted to give the right results, and
	// are as such kind of "ATR2 constants".
	
	double internal_heat = 0.25;
	// Cool down by 1 unit (1 out of 500 in normalized terms) per cycle,
	// with an addition of 1/8 if throttle is between -0.25 and 0.25,
	// inclusive.
	// Adjusted partly because of floating point effects. Arguably, this
	// might be a case where the ATR2 standard is wrong, but we'll keep to
	// it for now.
	double internal_cool = 1.059;

	// If it's going slowly, it cools down more quickly. This being
	// throttle is probably the most intuitive, since one'd expect better
	// (faster) engines to be engineered to run cooler, too.
	if (fabs(get_throttle()) <= 0.25) internal_cool += 1/8.0;

	// If we're shielded, add internal heat. The "realistic" way of
	// modeling this would be to add it in any case, and offset cooling
	// to compensate.
	if (is_shielded())
		heat_up(time_elapsed * internal_heat / 500.0);
	else	cool_down(time_elapsed * internal_cool / 500.0);

	set_speed_bonus(base_bonus * heat_balancer.get_speed_penalty(heat));
	// If we're too much in the red, inflict damage.
	// Don't know if ATR2 counts damage from heat as damage by oneself
	// when asking when we last got hurt and by who, but we assume it
	// doesn't.
	inflict_damage(false, heat_balancer.get_damage(heat) * time_elapsed,
			DMG_UNSPEC);

	// If we tripped the shutdown earlier, check if we're out of the
	// danger zone. If not, keep shutdown on, disable shields, and set
	// throttle to 0.
	if (has_shutdown && heat > get_lower_shutdown_temp()) {
		has_shutdown = true;
		set_desired_throttle(0);
		// Disable shields.
		set_shields(false);
		// Does it coast to a stop, or does this happen?
		// It coasts to a stop.
	} else {
		has_shutdown = false;
	}

	//cout << "Armor at this stage: " << get_armor() << endl;
	//cout << "Heat at this stage: " << heat << endl;

	// This isn't needed, but just to be safe..
	if (heat >= 1.0 || get_armor() < 0)
		armor = 0;

	// Something here about scheduling an explosion if we're dead.
	// Also something about inflicting damage on all the bystanders if
	// we die.
	// This happens in the die() subroutine.
	
	// We should also shut down completely and not pose any problem to the
	// other contestants. We do this by setting our location off the arena
	// and setting all throttle parameters to zero. Perhaps move this to
	// another function. DONE.
	if (get_armor() == 0) {
		die(explosions, other_robots);
		return(false);
	}

	if (!has_shutdown)
		// Okay, we're allowed to process code. Find out how many CPU
		// cycles we're going to allot and do so.
		CPU_cycles_available += time_elapsed * CPU_speed;

	return(true);
}

int robot::withdraw_CPU_cycles() {
	if (!is_CPU_working() || CPU_cycles_available == 0) return(0);

	int toRet = floor(CPU_cycles_available); // get cycles available
	CPU_cycles_available -= toRet; // subtract them as used
	// And return!
	return(toRet);
}

// Sensor actors, setters, and getters.

// Should these be const vector<const Unit *> ?

bool robot::do_scan(const list<Unit *> & active_robots) {
	// Set our position for the past record.
	
	// Update the scanner with the angle of the turret it's attached to.
	scan_sensor.set_center_hexangle(turret_heading);

	bool retval = scan_sensor.scan(active_robots, get_pos());

	// We've scanned, so set up our snapshot of the events.
	last_scan = aggregate_scan_info();
	if (last_scan.found) {
		last_successful_scan = last_scan;
		last_detected_transponder = get_last_target_ID(false);
	}
	
	return(retval);
}

bool robot::do_scan(const list<Unit *> & active_robots, int span) {
	if (span < 0 || span > 256) return(false); // Sanity check

	set_scan_span(span);
	do_scan(active_robots);

	return(true);
}

// DONE: Visual confirmation when these are run. Not sure how to do it,
// though; we'll have to pass a "did_radar_at" and "did_sonar_at" parameter up
// for the main program to query.
// Perhaps we should make the robot bodies flash? The source would flash more
// slowly (or from max to min), while the target (the one hit by the ping, as
// it were) would flash from half max to min.. Nah, that'll be too complex;
// we'll just set ourselves, not the target.
int robot::do_radar(const list<Unit *> & active_robots) {
	int last_radar_val = radar_sonar.check_radar(active_robots, get_pos(), 
			get_time());

	// If we found something, set the last detected transponder variable
	if (last_radar_val != -1)
		last_detected_transponder = radar_sonar.
			get_last_detected_transponder();

	return(last_radar_val);
}

int robot::do_sonar(const list<Unit *> & active_robots) {
	last_sonar_val = radar_sonar.check_sonar(active_robots, get_pos(), 
			get_time());

	if (last_sonar_val != -1)
		last_detected_transponder = radar_sonar.
			get_last_detected_transponder();

	return(last_sonar_val);
}

int robot::get_last_radar_time() const {
	return(radar_sonar.get_last_radar_time());
}

int robot::get_last_sonar_time() const {
	return(radar_sonar.get_last_sonar_time());
}

coordinate robot::get_last_sonar_pos() const {
	return(radar_sonar.get_last_sonar_pos());
}

int robot::get_sonar_maxrange() const {
	return(radar_sonar.get_sonar_maxrange());
}

// A whole bunch of scanner-related interfaces.

bool robot::get_scan_hit() const { return(last_scan.found); }

int robot::get_scan_dist(bool hit) const { 
	if (hit) return(last_successful_scan.range);
	else	return(last_scan.range); 
}

int robot::get_accuracy(bool hit) const { 
	if (hit) return(last_successful_scan.accuracy);
	else	return(last_scan.accuracy); 
}

int robot::get_scan_span(bool hit) const { 
	if (hit) return(last_successful_scan.span);
	else	return(last_scan.span); 
}

int robot::get_scan_time(bool hit) const {
	if (hit) return(last_successful_scan.clock);
	else	return(last_scan.clock);
}

int robot::get_scan_center(bool hit) const { 
	if (hit) return(last_successful_scan.angle);
	else	return(last_scan.angle);
}

int robot::get_scan_body_heading(bool hit) const {
	if (hit) return(last_successful_scan.our_angle);
	else	return(last_scan.our_angle);
}

coordinate robot::get_scan_pos(bool hit) const { 
	if (hit) return(last_successful_scan.position);
	else	return(last_scan.position); }

// Information about the last target we scanned, at the moment we scanned it.
double robot::get_last_target_throttle() const {
	return(last_successful_scan.target_throttle);
}

int robot::get_last_target_heading() const {
	return(last_successful_scan.target_angle); 
}

// If any_scan is true, also returns sonar and radar hits (as ATR2 does, but
// only for transponder!)
short robot::get_last_target_ID(bool any_scan) const {
	if (any_scan)
		return(last_detected_transponder);
	else	return(last_successful_scan.target_transponder);
}

double robot::get_last_target_abs_speed() const {
	return(last_successful_scan.target_abs_speed);
}

void robot::set_scanrange(int scanrange) { scan_sensor.set_radius(scanrange); }
void robot::set_scan_span(int span) { scan_sensor.set_span(span); }

void robot::set_scan_drange(int det_range) { 
	scan_sensor.set_detection_radius(det_range);
}

void robot::set_missile_speed(double mspd) {
	base_missile_speed = mspd;
}

// Other sensors - setting.
// Maybe timer should be centralized? Hm.

void robot::set_clock(int clock_signal) {
	clock = clock_signal;
}

void robot::tick(double delta) {
	clock += delta;
}

// Required for caching.
double robot::get_time() const {
	return(clock);
}

// This returns int because we want the external appearance to only give whole
// cycles.
int robot::get_time_int() const {
	return(round(clock));
}

// Comms backlog retrieval
bool robot::get_last_message(unsigned short & output_loc) { // -1 if none
	if (!comms_queue.is_message_pending())
		return(false);
	
	output_loc = comms_queue.get_last_msg();
	return(true);
}
/*
bool robot::get_earlier_message(int pos, unsigned short & output_loc) {
	// peek
	// Return false if pos > count or something (rollover problem).
	assert(1 != 1);
}*/

int robot::get_num_waiting_messages() const {
	return(comms_queue.get_num_accessible_messages());
}



void robot::update_error(int new_error) {
	// This means we have an error, so set that to true.
	error = true;
	last_error = new_error;
	// And record this error for the stats.
	record_error();
}

// Only allow shields to be raised if we have any shields /to/ raise.
bool robot::set_shields(bool up) {
	if (shield_type == 0 && up) return(false);
	shields_up = up;
	return(true);
}

// Offset_to_turret is the adjustment allowed with respect to a fixed turret
// location. For ATR2, this is +/- 4, double accuracy, which is checked
// here.
bool robot::fire(list<missile> & add_to, int offset_to_turret) {
	// Or does it just fire at ott 0 in ATR2? I think it truncates.
	if (offset_to_turret < -4) offset_to_turret = -4;
	if (offset_to_turret >  4) offset_to_turret =  4;

	double missile_speed = base_missile_speed * weapon_power;
	if (is_overburning())
		missile_speed *= 1.25;

	// Generate missile with appropriate stats at this location
	add_to.push_back(missile(get_pos(), missile_speed, 1, 
				hexangle(turret_heading + offset_to_turret), 
				is_overburning(), weapon_power, get_UID()));
	
	// This ain't free; add waste heat. 20 points base (out of 500), 
	// multiplied by 1.5 if on overburn.
	
	// ATR2 native reports 19 for some reason.
	
	if (is_overburning())
		heat_up(0.04 * 1.5 * weapon_power);
	else	heat_up(0.04 * weapon_power);

	local_stats.data[RI_SHOTSFIRED]++;

	return(true);
}

bool robot::deploy_mine(list<mine> & add_to, int detection_radius) {
	// First, check if we actually have any mines to lay. If not, 
	// return false. Otherwise, increase mines_deployed and, well,
	// lay it!
	
	if (mines_available <= 0) return(false);
	++mines_deployed;
	--mines_available;
	//cout << "Laying mine: we now have " << mines_available << " available." << endl;

	add_to.push_back(mine(get_pos(), get_UID(), detection_radius));
	local_stats.data[RI_MINESLAID]++;

	//cout << "Add_to says: " << add_to.size() << " mines left " << endl;

	return(true);
}

// We should do a unit test on these. Later, I'm dog tired.

void robot::remove_communications_link(vector<set<robot *> > & lookup) {
	lookup[comms_channel].erase(this);
}

void robot::set_communications_channel(int new_channel, vector<set<robot *> > &
		lookup) {

	// Remove ourselves from the old channel and set type to the new 
	// channel, then insert ourselves there.
	//lookup[comms_channel].erase(this);
	remove_communications_link(lookup); // old channel
	comms_channel = new_channel;
	lookup[comms_channel].insert(this);
}

void robot::transmit(unsigned short signal, vector<set<robot *> > &
		lookup) {
	// For all the robots on the channel (except ourselves), invoke
	// receive.
	
	for (set<robot *>::iterator pos = lookup[comms_channel].begin(); pos !=
			lookup[comms_channel].end(); ++pos) {
		robot * ptr = *pos;
		if (ptr == this) continue; // not ourselves
		ptr->receive_transmission(signal);
	}
}

void robot::record_death() {
	if (is_dead) return;
	local_stats.data[RI_DEATHS]++;
	local_stats.data[RI_ENDHEAT] = min(1.0, get_heat());
	local_stats.data[RI_ENDARMOR] = max(0.0, get_armor());
	local_stats.data[RI_LIFESPAN] = get_time();
	is_dead = true;
}

// If record_true_age is on, we record the number of cycles that passed until
// the round was done. If not, we record it as having lived maxcycles, by the
// reasoning that there's nothing to stop it from doing so since it has no
// competition. In the case of a tie, the current time is max time and so it
// makes no difference.
void robot::record_end_stats(bool record_true_age, int maxcycles) {
	increment_round_count();
	if (is_dead) return;	// stats were already recorded at time of death

	local_stats.data[RI_ENDHEAT] = min(1.0, get_heat());
	local_stats.data[RI_ENDARMOR] = max(0.0, get_armor());
	if (record_true_age)
		local_stats.data[RI_LIFESPAN] = get_time();
	else	local_stats.data[RI_LIFESPAN] = maxcycles;
}

void robot::record_damage_to_others(double how_much) {
	// No healing in this game!
	local_stats.data[RI_DAMAGE] += max(0.0, how_much);
}

// Don't count if it's already dead.
void robot::record_damage_to_others(const robot & who, double how_much) {
	if (who.dead()) return;
	record_damage_to_others(how_much);
}

// NOTE that these must only be called once per detonation/explosion.
// We can get around this later (if we want to record damage, etc) by recording
// where it came from last so that the same missile doesn't count more than
// once.
// We can't use if (clock != last_hit_at) because, although unlikely, two
// missiles may hit at the exact same time. Also the clock is within the
// granularity of the robot's clock, so it may be off by a cycle or two (worse
// in the Utopian construction).
void robot::record_bot_shot(const Unit * other_bot) {
	local_stats.data[RI_HITS]++;
	transp_last_impacted = other_bot->get_ID();
	last_hit_at = clock;
}

// if (clock != last_blown_at) works even worse here since the owner may
// detonate all his mines with an interrupt.
void robot::record_bot_mine_hurt(const Unit * other_bot) {
	local_stats.data[RI_MINEHITS]++;
	transp_last_blown = other_bot->get_ID();
	last_blown_at = clock;
}

// Getters for the various statistics.

int robot::get_local_deaths() const {	return(local_stats.data[RI_DEATHS]); }
int robot::get_local_kills() const {	return(local_stats.data[RI_KILLS]); }
int robot::get_local_shots_fired() const { 
	return(local_stats.data[RI_SHOTSFIRED]); 
}

int robot::get_local_shots_hit() const { return(local_stats.data[RI_HITS]); }
int robot::get_local_mines_hit() const { return(local_stats.data[RI_MINEHITS]);}

int robot::get_all_victories() const {
	return(local_stats.data[RI_VICTORY] + global_stats->get_sum()->
			data[RI_VICTORY]);
}

int robot::get_all_deaths() const {
	return(get_local_deaths() + global_stats->get_sum()->data[RI_DEATHS]);
}

int robot::get_all_kills() const {
	return(get_local_kills() + global_stats->get_sum()->data[RI_KILLS]);
}

int robot::get_all_shots_fired() const {
	return(get_local_shots_fired() + global_stats->get_sum()->
			data[RI_SHOTSFIRED]);
}

int robot::get_all_shots_hit() const {
	return(get_local_shots_hit() + global_stats->get_sum()->data[RI_HITS]);
}

int robot::get_all_mines_hit() const {
	return(get_local_mines_hit() + global_stats->get_sum()->data[RI_MINEHITS]);
}

#endif
