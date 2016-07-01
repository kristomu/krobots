
// Something that moves; used for robots and missiles.
// Maybe have hexdegrees and tables here later.

// Throttle is the fraction of speed multiplier currently applied: for forwards
// motion, it goes from 0 to 1.
// Speed_multiplier is the absolute speed corresponding to throttle=1.

// Note that you *must* call set_desired_throttle/set_desired_heading
// to properly change throttle and heading, since these will trip "altered",
// thus notifying the collision system to recalculate collision times.

#ifndef _KROB_MOVER
#define _KROB_MOVER

#include "tools.cc"
#include "coordinate.cc"
#include <math.h>
#include <assert.h>
#include <iostream>	// required for min, for some reason

class Mover {

	private:
		coordinate position;

		double speed_multiplier;	// m/s corresponding to full 
						// blast
		double speed_bonus;		// For overheating penalties,
						// overburning missiles.
		double min_speed;		// Usually -0.75, makes no
						// sense for missiles
		double max_speed;		// 1.
	
		double throttle;
		double desired_throttle;

		double heading;			// This is in hexdegrees.
		double desired_heading;		// So is this.

		// Constraints
		double degs_per_sec;		// How much we can turn in a sec
		double units_per_sec;		// How much we can speed up in
						// a sec
		
		// Auxiliary info
		double odometer;
		bool altered;			// To trip cached collisions.
		bool does_crash;

		// Cached cosine and sine values for current direction.
		double cached_heading;
		double cached_mulcos, cached_mulsin;

		double get_avg_speed(const double old_throttle, 
				const double new_throttle) const;
	
		coordinate predict_motion(const coordinate & old_pos,
				const double old_heading, 
				const double new_heading,
				const double old_throttle, 
				const double new_throttle,
				const double time_elapsed) const;

	
	protected:
		void crash_halt();		// Immediately stops moving.

	public:
		void clear_altered() { altered = false; }
		void set_altered() { altered = true; }
		void set_does_crash(bool crashes) { does_crash = crashes; }
		bool get_does_crash() { return(does_crash); }
		bool has_changed_route() { return(altered); }

		void set_speed_info(double smin, double smax, 
				double multiplier);
		void set_speed_multiplier(double multiplier);
		double get_speed_multiplier() const{ return(speed_multiplier); }
		void set_response_limit(double degs_per_sec_in,
				double units_per_sec_in);

		Mover(double smin, double smax, double multiplier,
				double dps, double ups,
				double start_heading, double start_throttle,
				coordinate start_at);
		virtual ~Mover() {} //Unlikely to be needed, but be aware of it.

		// Put/get
		void set_desired_heading(double dshi);
		void set_desired_throttle(double dsv);
		double get_heading() const { return(heading); }
		double get_desired_heading() const { return(desired_heading); }
		double get_throttle() const { return(throttle); }
		double get_desired_throttle() const { return(desired_throttle);}
		void set_speed_bonus(double bonus); 
		double get_speed_bonus() const { return(speed_bonus); }
		double get_absolute_speed() const;
		double get_worst_case_abs_speed() const; // But beware overburn

		// Consts
		double get_min_throttle() const { return(min_speed); }
		double get_max_throttle() const { return(max_speed); }

		coordinate get_pos() const { return(position); }
		coordinate get_pos_at(double time) const;
		void set_pos(const coordinate & in);

		void zero_odometer() { odometer = 0; }
		void increment_odometer(double trip_len) {odometer += trip_len;}
		double get_length_traveled() const { return(odometer); }
		
		// Sets both desired and current throttle to zero.
		// Can be overloaded by missiles and mines (that just explode
		// instead). Not yet, though (have to handle edge collisions
		// and so on).
		virtual void register_crash(double relative_time);

		// These output new heading and throttle parameters.
		double turn(const double seconds_passed, 
				const double degs_per_sec) const;
		double accelerate(const double seconds_passed, 
				const double units_per_sec) const;

		// This is used to adjust scanners, etc. with particular
		// keepshift values. Basically, it's a "did turn" callback.
		virtual void adjust_peripherals(const double old_heading) {}

		void move(const double cycles_passed);
		// Used for collision detection. Returns predicted position
		// after seconds_passed, but doesn't actually update any
		// parameters.
		coordinate predict_motion(const coordinate & old_pos,
				const double seconds_passed) const;
};

// Setters
void Mover::set_desired_heading(double dshi) {
	if (desired_heading == dshi) return;
	if (desired_throttle != 0 || throttle != 0)
		set_altered();
	desired_heading = dshi;
}

void Mover::set_desired_throttle(double dsv) {
	double new_throttle = min(max_speed, max(min_speed, dsv));
	if (new_throttle == desired_throttle) return;
	set_altered();
	desired_throttle = new_throttle;
}

void Mover::set_speed_bonus(double bonus) { 
	assert(bonus > 0);
	if (speed_bonus == bonus) return;
	set_altered();
	speed_bonus = bonus;
}

// Find out the average absolute throttle over the distance. This is used in
// motion prediction and in increasing our travel length count. Estimate high,
// as in ATR2.
double Mover::get_avg_speed(double old_throttle, double new_throttle) const {
	return(new_throttle * speed_multiplier * speed_bonus);
}

// Instantaneous
double Mover::get_absolute_speed() const {
	return(get_throttle() * speed_multiplier * get_speed_bonus());
}

// Overburn shouldn't be a problem, since it's turned on and off directly and
// thus updates speed_multiplier, but I'm not completely sure about that, just
// almost sure.
double Mover::get_worst_case_abs_speed() const {
	return(max(get_throttle(), get_desired_throttle()) * speed_multiplier *
			get_speed_bonus());
}

// Coordinate set/get
void Mover::set_pos(const coordinate & in) {
	position = in;
}

coordinate Mover::get_pos_at(double time) const {
	return(predict_motion(get_pos(), time));
}

// Predict where we'll end up with the given changes in heading and throttle.
// The heading/throttle settings are at old_heading/old_throttle at t = 0
// and at new_heading/new_throttle at t = epsilon for some very low value
// epsilon.
// The collision detection mechanism will not work with curved paths, so this
// is necessary for heading, but might not be for throttle.
coordinate Mover::predict_motion(const coordinate & old_pos, 
		const double old_heading, const double new_heading, 
		const double old_throttle, const double new_throttle, 
		const double seconds_passed) const {

	if (seconds_passed == 0) return(old_pos);

	const double abs_throttle = get_avg_speed(old_throttle, new_throttle);
	double cosval, sinval;
	if (new_heading == cached_heading) {
		cosval = cached_mulcos;
		sinval = cached_mulsin;
	} else {
		cosval = cos(hex_to_radian(new_heading));
		sinval = sin(hex_to_radian(new_heading));
	}
	
	return(coordinate(
				old_pos.x + abs_throttle * seconds_passed 
					* //cos(hex_to_radian(new_heading)),
					  //vcos64(new_heading),
					  cosval,
				old_pos.y + abs_throttle * seconds_passed 
					* //sin(hex_to_radian(new_heading))
					  //vsin64(new_heading)
					  sinval
			));
}

// Immediately stops moving, as in a crash. Used by inheritors to call the
// common part of register_crash without having to copy the code.
void Mover::crash_halt() {
	set_altered();
	set_desired_throttle(0);
	throttle = 0;
}

// Set the speed limits for the mover.
// DONE: do sanity checks (smin < smax)
void Mover::set_speed_info(double smin, double smax, double multiplier) {
	// Mines have multipliers of 0, since they don't move.
	assert (smin <= smax && multiplier >= 0);
	assert (finite(smin) && finite(smax) && finite(multiplier));
	min_speed = smin;
	max_speed = smax;
	speed_multiplier = multiplier;
}

void Mover::set_speed_multiplier(double multiplier) {
	assert (multiplier > 0);
	assert (finite(multiplier));

	speed_multiplier = multiplier;
}

void Mover::set_response_limit(double degs_per_sec_in, 
		double units_per_sec_in) {
	degs_per_sec = degs_per_sec_in;
	units_per_sec = units_per_sec_in;
}

Mover::Mover(double smin, double smax, double multiplier, 
		double dps, double ups, 
		double start_heading, double start_throttle,
		coordinate start_at) {

	assert (start_throttle >= smin && start_throttle <= smax);

	zero_odometer();
	set_speed_info(smin, smax, multiplier);
	set_response_limit(dps, ups);
	set_pos(start_at);
	heading = start_heading;
	desired_heading = start_heading;
	throttle = start_throttle;
	desired_throttle = start_throttle;
	speed_bonus = 0; // fixes valgrind error
	cached_heading = -1;

	set_speed_bonus(1); // Nothing special.
	set_altered();
}

// If we crash, our throttle (and desired throttle) comes to an abrupt halt.
// More realistic would be to set throttle to 0 but not desired throttle, but
// ATR2 sets both to 0.
void Mover::register_crash(double relative_time) {
	crash_halt();
}

// This adjusts "heading" towards "desired_heading", either all the way or in
// an increment of degs_per_sec * seconds_passed, whichever is smaller. Since
// we can move both clockwise and counterclockwise, we construct two 
// hypothetical new headings and check which gets closer to the target.

double Mover::turn(const double seconds_passed, const double degs_per_sec) const {

	if (heading == desired_heading) return(heading);
	if (seconds_passed <= 0) return(heading);

	// Get the shortest increment.
	double increment = std::min(fabs(desired_heading - heading),
			                        seconds_passed * degs_per_sec);

	// Normalize desired heading so that our current heading is at 0.
	//double norm_dh = fmod(256 + desired_heading - heading, 256);
	double norm_dh = hexangle(desired_heading - heading);
	// Now, if dh is > 128, go left, otherwise go right.
	if (norm_dh <= 128)
		return(hexangle(heading + increment));
	else	return(hexangle(heading - increment));
	/*if (norm_dh <= 128)
		return(fmod(256 + heading + increment, 256));
	else	return(fmod(256 + heading - increment, 256));*/
	
}

double Mover::accelerate(const double seconds_passed, 
		const double units_per_sec) const {

	// First find out the absolute difference in throttle permitted for
	// this period. If we can reach target throttle, that's obviously it;
	// otherwise, it's units_per_sec * seconds_pased. The fabs is so that
	// negative velocities work as well.
	
	if (throttle == desired_throttle) return(throttle);
	if (seconds_passed <= 0) return(throttle); // DOH!

	// Shouldn't this be ups * elapsed * elapsed ? If we have to be jittery,
	// we should set the acceleration so it's a point at the smooth curve..
	// No, since throttle is the first derivative of position, and
	// acceleration is the second; we deal with acceleration wrt
	// throttle, not wrt position.
	// But this means that, to be correct, advance must use ^2 (since it
	// deals with position). That may cost too much.
	double diff = std::min(fabs(throttle - desired_throttle),
			units_per_sec * seconds_passed);

	if (desired_throttle > throttle)
		return(std::min(max_speed, throttle + diff));
	else	return(std::max(min_speed, throttle - diff));

}

coordinate Mover::predict_motion(const coordinate & old_pos, 
		const double seconds_passed) const {

	if (get_throttle() == 0 && get_desired_throttle() == 0)
		return(old_pos);

	// Get new headings and velocities
	double new_heading = turn(seconds_passed, degs_per_sec);
	double new_throttle = accelerate(seconds_passed, units_per_sec *
			speed_bonus);

	return(predict_motion(old_pos, heading, new_heading, throttle,
				new_throttle, seconds_passed));
}

// DONE: Handle intersection problem where robot won't turn if it's stationary.
// Problem is that the collision routine returns 0 in this case (collides at 0
// or never collides), which means that we turn for 0 seconds, which is no
// turn at all.
// 	Solved within collision: we check whether +epsilon goes towards away
// 	from crash or towards it.
void Mover::move(const double cycles_passed) {

	// First of all, turn and accelerate if required (checks inside the
	// relevant functions)
	double new_heading = turn(cycles_passed, degs_per_sec);
	// Assumption that'll get altered if throttle or desired_throttle
	// isn't 0.
	double new_throttle = 0;

	// Get cheap speed benefits against sduck: only recalc position if
	// absolutely required.
	if (get_throttle() != 0 || get_desired_throttle() != 0) {

		new_throttle = accelerate(cycles_passed, units_per_sec *
				speed_bonus);

		if (new_heading != cached_heading) {
			cached_heading = new_heading;
			cached_mulcos = cos(hex_to_radian(cached_heading));
			cached_mulsin = sin(hex_to_radian(cached_heading));
		}

		// DEBUG
		/*string f = "Oldpos " + itos(position.x) + ", " + itos(position.y) + " - new position:";*/
		position = predict_motion(position, heading, new_heading,
				throttle, new_throttle, cycles_passed);
		/*if (position.x < 0)
			cout << f << position.x << ", " << position.y << " after t + " << cycles_passed << endl;
		assert (position.x >= 0);*/

		// Adjust odometer
		increment_odometer(get_avg_speed(throttle, new_throttle));
	}

	/*cout << "Adjustment: throttle was " << throttle << " but now is " << new_throttle << endl;
	cout << "Adjustment: Old pos " << old_pos.x << ", " << old_pos.y << endl;
	cout << "Adjustment: New pos " << newpos.x << ", " << newpos.y << endl;
	cout << "Adjustment: Old heading: " << heading << endl;
	cout << "Adjustment: New heading: " << new_heading << endl;
	cout << "Adjustment: Time elapsed: " << seconds_passed << endl << endl;
	*/

	// Set the new headings/velocities if required.
	if (heading != new_heading || throttle != new_throttle) {
		set_altered();
		throttle = new_throttle;

		if (heading != new_heading) {
			swap(heading, new_heading);

			// this is actually with old heading.
			adjust_peripherals(new_heading); 
		}
	}

}

#endif
