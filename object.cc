// Somethint that physically exists, possibly with an engine.
// Inherits Mover because of the need for advance() when doing collision
// detection.

// Scuttled? Ponder how to incorporate this, because unifying scanning and such
// would be very nice.

// Should perhaps be the other way around? All Movers are Units, not all Units
// are Movers.

// Coordinates have now been moved into Mover because move is called an
// extreme number of times, and that way we don't have to alloc and dealloc
// every single time.

// Because of scan limitations, each unit is also fitted with a transponder.
// It breaks the hierarchy a bit, but we'll permit it for now. For objects with
// no transponder, we give -(UID of parent). Since UID can't be changed, this
// might be an unacceptable data leak; set the transfer function to -1 if so.
// (No scanner can read the ID of missiles, currently, though.)

// Also kludged together a dead boolean and is_dead. We really should fix that,
// because missiles are always "live" until they're destroyed.
// Best probably to use list<robot *> instead of list<unit *> but err.. why
// have unit at all then?

#ifndef _KROB_OBJECT
#define _KROB_OBJECT

#define CRASH_EDGE -2

#include "coordinate.cc"
#include "mover.cc"
#include <vector>

using namespace std;

class Unit : public Mover {

	private:
		//coordinate position;
		bool cloaked;		// Invisible on a scan?
		unsigned int radius;	// All objects are round.
		short transponder;

		double move_tus;	// replacement for time_units[x],
					// this gives for how long we're moving
					// this cycle. Cleanup later.

		// Crash-related calculations
		int crashed_with;
		double other_throttle; // Used for recording that we did dmg

	protected:
		bool is_dead;		// OW OW OW!

	public:
		short get_ID() const { return(transponder); }
		void set_ID(short val) { transponder = val; }

		void set_time_units(double tu) { move_tus = tu; }
		double time_units() const { return(move_tus); }

		// Set_crash is UID, or -1 if no crash, or -2 if with the edge.
		void set_crash(int whom) { crashed_with = whom; }
		void set_crash_target_vel(double throttle) {
			other_throttle = throttle; }
		void clear_crash() { crashed_with = -1; }
		bool crashed() const { return(crashed_with != -1); }
		int get_crash_origin() const { return(crashed_with); }
		double get_crash_target_throttle() { return(other_throttle); }

		bool dead() const { return(is_dead); }

		Unit(double min_epower, double max_epower, double max_speed,
				double turning_rate, double acceleration_rate,
				double start_heading, double start_throttle,
				int radius_in, int transponder_in, 
				bool is_cloaked) :
			Mover(min_epower, max_epower, max_speed, turning_rate,
					acceleration_rate, start_heading,
					start_throttle, coordinate(0,0)) {
				radius = radius_in; cloaked = is_cloaked; 
			transponder = transponder_in; is_dead = false; }

		Unit(const coordinate & start_pos, double min_epower, 
				double max_epower, double max_speed,
				double turning_rate, double acceleration_rate,
				double start_heading, double start_velocity,
				int radius_in, int transponder_in, 
				bool is_cloaked) :
			Mover(min_epower, max_epower, max_speed, turning_rate,
					acceleration_rate, start_heading,
					start_velocity, start_pos) {
				radius = radius_in; cloaked = is_cloaked; 
			transponder = transponder_in; }

		//void move(double seconds_elapsed);

		void cloak() { cloaked = true; }
		void uncloak() { cloaked = false; }
		bool is_cloaked() { return(cloaked); }

		int get_radius() { return(radius); }
		void set_radius(unsigned int radius_in) { radius = radius_in; }
};

#endif
