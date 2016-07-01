// Rudimentary missile class.
// NOTE: Missiles explode when crashing against a robot, but not against a wall.
// I suppose it's more correct to say that missiles don't explode, they just
// hurt the point of impact, shearing through.

// Do missiles explode against center or against shield when the robot is 
// shielded?

#ifndef _KROB_MISSILE
#define _KROB_MISSILE

#include "mover.cc"
#include "object.cc"

class missile : public Unit {

	private:
		bool overburning;
		int shooter;		// But what about dead robots?
		double hit_at;		// Time it hits, when this is the case.
		Unit * target;		// NULL if nothing has been hit, else 
					// pointer to target.
		const Unit * source;	// for identifying a source. Is this 
					// ugly?

		double base_weapon_power;


	public:
		// Missiles can't go backwards. Neither can they turn.
		// They don't show up on a scan either.
		// They also have zero radius (being point sources) and start
		// off at full speed.
		missile(coordinate center, double max_speed, 
				double acceleration_rate, 
				double start_heading, bool overburn_in,
				double base_power, int fired_by) :
			// Note that start_velocity is really throttle; that's
			// why it's 1 here - missile starts at full throttle.
			Unit(0, 1, max_speed, 0, acceleration_rate,
					start_heading, 1,
					0, -fired_by, true) { set_pos(center); 
				hit_at = -1; set_shooter(fired_by); 
				overburning = overburn_in; 
				base_weapon_power = base_power;}

		void set_shooter(int fired_by) { shooter = fired_by; }
		int get_shooter() { return(shooter); }
		bool has_overburn() { return(overburning); }

		// For target effects.. kinda kludgy. Record who we hit so that
		// we can figure out the explosion center within the collision
		// detection subroutine. Note that when we use this, 
		// time_on_target will refer to the time we impacted the 
		// target, not when the missile exploded inside.
		void set_target(Unit * to_set) { target = to_set; }
		Unit * get_target() { return(target); }

		void update_time_on_target(double relative_time);
		double get_hit_time() { return(hit_at); }

		double get_damage_multiplier(); 

		// Determine the point at which the missile explodes, given the
		// penetration point; the missile explodes at the closest point
		// to the robot core. We could replace this with a golden 
		// section search when preconditions no longer hold true. The 
		// preconditions are: the missile's significantly faster than 
		// the robot, and the missile doesn't turn inside the bot. Note
		// that this isn't called often, so the calculation itself can 
		// safely be expensive (even if it isn't, currently). Also note
		// that it doesn't check for edges or collisions with multiple
		// robots at once (which never happens).
		coordinate get_explosion_point(const coordinate impact_point,
				double impact_time, const coordinate 
				robot_center_mass);

};

void missile::update_time_on_target(double relative_time) {
	// Register that this is the time we hit.
	hit_at = relative_time;
}

double missile::get_damage_multiplier() {
	// Another ATR2 constant.
	if (has_overburn()) return(1.25 * base_weapon_power);
	return(1.0 * base_weapon_power);
}

// Not sure if this works right, yet. See tstmotion match 263.
coordinate missile::get_explosion_point(coordinate impact_point, 
		double impact_time, const coordinate robot_center_mass) {

	// Our strategy is simple. Find out the impact point and another point 
	// (to determine ascent rate), then find the closest point on the 
	// infinite line so defined wrt the robot center point. That's the 
	// explosion point.
	
	// We use an infinite line because it would be unfair for the missile 
	// to be cheated of its ability to get all the way to the center of the
	// robot simply because it did so at the end of its "turn".
	
	if (impact_point == robot_center_mass) return(impact_point);

	// Find the other point on the line.
	// Pick a time that can't have been the impact time.
	coordinate line_sketch = get_pos_at(0.01); 

	double denom = line_sketch.sq_distance(impact_point);
	assert (denom != 0); // Should always pass unless missile doesn't move.

	// The center is at (x3, y3), and the two line points at (x1, y1) and 
	// (x2, y2).
	//     (x3 - x1) (x2 - x1) + (y3 - y1) (y2 - y1)
	// u = -----------------------------------------
	// 		denom
	
	double diff_x = (line_sketch.x - impact_point.x),
	       diff_y = (line_sketch.y - impact_point.y); 

	double u = (robot_center_mass.x - impact_point.x) * diff_x +
		(robot_center_mass.y - impact_point.y) * diff_y;
	u /= denom;

	coordinate closest(impact_point.x + u * diff_x,
			impact_point.y + u * diff_y);

	return(closest);
}
#endif
