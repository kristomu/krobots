// Collision detection functions. 

// Doesn't use Unit, uses Mover instead (missiles aren't round.)
// We can use ordinary collision detection for missiles too. When we get
// within radius p, boom! Just set min_distance to zero and then check how
// close we got.

#include "coordinate.cc"
#include "coord_tools.cc"
#include "missile.cc"
#include "robot.cc"
#include "blast.cc"
#include <assert.h>
#include <vector>
#include <list>

using namespace std;

class collider {

	private:
		// Line-line intersection tool.
		coord_tool intersect_test;

		// Determines the closest approach after seconds_passed,
		// where the robots are known to collide (or time out) after
		// a_max and b_max respectively. Perturbation is a hack for
		// handling parallel lines in the case they have an infinity
		// of "closest" points. Returns special values CT_DLL_* when
		// required.
		double get_closest_approach(const Unit * a, const Unit * b, 
				double seconds_passed, double a_max, 
				double b_max, double perturbation) const;
		double get_closest_approach(const Unit * a, const Unit * b,
				double seconds_passed, double a_max,
				double b_max) const;

		// Returns whether we're outside the wall (rectangle) given by
		// (min_x, min_y) - (max_x, max_y). True is yes, false is no,
		// edges count as no. (See new descript)
		double arena_wall_dist(const coordinate & pos, double minx, 
				double miny, double maxx, double maxy, 
				double thickness) 
			const;

	public:
		// Used in initial robot location checks, to be sure we don't
		// place the robot in the middle of a wall or something.
		bool inside_wall(const coordinate & pos, double 
				collision_radius, const coordinate &
				arena_max) const;
		// Determines the point in time at which a and b are separated
		// by collision_distance. If "squared", expects squared
		// Euclidean distance, otherwise ordinary. Note that the
		// function doubles collision_radius since two objects
		// are (potentially) colliding, not just one.
		// a_max_trip_time is the time until a either crashes or
		// exhausts its timecycle; same with b_max_trip_time.
		// The function uses a binary search, and returns -1 if no crash
		// occurs or 0 if the robots are stationary.
		// If closest_possible is true, the function will, instead of
		// returning a time when they just collide, return the time when
		// they are closest.
		double determine_crossing_point(const double accuracy, 
				Unit * a, Unit * b, 
				const double a_max_trip_time, 
				const double b_max_trip_time,
				const int a_collide_radius, 
				const int b_collide_radius,
				bool squared) const;

		// Does the same, only with edges (arena boundaries).
		double determine_edge_crash_point(const double accuracy, 
				Unit * a, const double a_max_trip_time, 
				const int collision_radius, 
				const coordinate & arena_maxima) const;

		// This updates the track limits (how far the robot can move)
		// for all robots. Grid-type queries and early pruning will 
		// have to be inserted here later, or maybe through a 2D bool.
		// Hm. We'll see.
		// Crash_range is the distance we can be from the wall before
		// it's counted as a crash. Robot_radius is the radius of the
		// spherical shell of the robot (for robot-robot collisions).
		// The function updates the arrays "crashes" (true if there was
		// a crash) and time_units. It does not actually move any of
		// the units (so we can do missile checks on the track?).
		void set_track_limits(list<robot *> & robots, 
				int robot_radius, int crash_range, 
				const coordinate arena_size) const;

		// This checks missiles against robots and edges. Once a robot 
		// is hit by a missile, we check that robot no further. Then, 
		// after all that is done, we calculate splash damage and remove
		// the offending missiles. Perhaps a bit too monolithic, but
		// I couldn't figure out where else to put it. (Mines are going
		// to be a pain)
		// The reason we can't just have get_missile_crashes is that
		// we have to reconcile crashes since missiles detonate
		// immediately upon hitting.
		void handle_missile_crashes(vector<robot> & all_robots,
				list<robot *> & live_robots, 
				double full_timespan, list<missile> & missiles,
				int max_hit_radius, const coordinate arena_size,
				double absolute_time_at_start, blasts &
				explosions) const;

		// This does the same, but with mines.
		void handle_mine_crashes(vector<robot> & all_robots,
				list<robot *> & live_robots, 
				double full_timespan, list<mine> & mines, 
				blasts & explosions) const;
};

double collider::get_closest_approach(const Unit * a, const Unit * b, 
		double seconds_passed, double a_max, double b_max, 
		double perturbation) const {

	// First find the other ends of the line segments - where the robots
	// will be after their respective maxima.
	coordinate a_other_end = a->predict_motion(a->get_pos(),
			min(seconds_passed, a_max));
	coordinate b_other_end = b->predict_motion(b->get_pos(),
			min(seconds_passed, b_max));

	if (a_other_end == b_other_end)
		return(CT_DLL_ONLY_POINTS); // Early opt

	// Add jumbling if so specified. By altering only one point, we can be
	// sure it turns parallel into non-parallel.

	if (a_other_end.y == a->get_pos().y)
		a_other_end.y += perturbation;
	else	a_other_end.x += perturbation;
	
	// Then ask about the distance between the lines, midpoints
	// notwithstanding.
	
	return(intersect_test.dist_line_line(a->get_pos(), a_other_end,
				b->get_pos(), b_other_end, false, true));

}

// For the common case of no perturbation.
double collider::get_closest_approach(const Unit * a, const Unit * b, 
		double seconds_passed, double a_max, double b_max) const {

	return(get_closest_approach(a, b, seconds_passed, a_max, b_max, 0));
}

// returns distance to closest arena wall, or negative some distance if outside
// of arena.
double collider::arena_wall_dist(const coordinate & pos, double minx, 
		double miny, double maxx, double maxy, double thickness) const {

	//cout << "Checking " << pos.x << ", " << pos.y << endl;

	double check, record;
	check = pos.x - (minx + thickness);
	if (check < 0) return(check);
	record = check;

	check = (maxx - thickness) - pos.x;
	if (check < 0) return(check);
	if (record > check) record = check;

	check = pos.y - (miny + thickness);
	if (check < 0) return(check);
	if (record > check) record = check;

	/*if (pos.x >= maxx - thickness) return(false);
	if (pos.y >= maxy - thickness) return(false);*/

	check = (maxy - thickness) - pos.y;
	if (check < 0) return(check);
	if (record > check) record = check;

	return(record);
}


// ---- //

bool collider::inside_wall(const coordinate & pos, double collision_radius, 
		const coordinate & arena_max) const {

	return (arena_wall_dist(pos, 0, 0, arena_max.x, arena_max.y, 
				collision_radius) <= 0);
}

// Caveats: this method still only works on straight lines. We need to move
// to the ultimate infrastructure before it'll work on curves.
// Note: When using missiles, the missile must be Unit * b.
double collider::determine_crossing_point(const double accuracy, Unit * a, 
		Unit * b, const double a_max_trip_time, 
		const double b_max_trip_time, const int a_collide_radius, 
		const int b_collide_radius, bool squared) const{

	// First translate the collision radii into a common standard, dealing
	// with the case of two bodies of differing radii as well (like a
	// missile with radius zero wrt a robot of large radius.
	
	// DONE: The missile problem is an optimization problem. How are we
	// going to deal with it? Gradient descent? We may be able to do
	// something special-case. DONE (special case).

	int collision_squared;
	if (squared)
		collision_squared = a_collide_radius + b_collide_radius;
	else	collision_squared = a_collide_radius * a_collide_radius +
		b_collide_radius * b_collide_radius;

	// Now, first of all we should check the most common cases. These are
	// that the robots aren't going to collide even after moving through
	// their entire path, and that they're stationary and thus never will
	// collide either.
	
	// Set max trip time, then figure out if any of these common cases are
	// true. Max trip time is the time required until both robots have
	// exhausted their time; if a_max_trip_time and b_max_trip_time are
	// not equal, then the shorter-timed one was judged to crash with 
	// another robot earlier, and therefore we model this by the robot 
	// simply stopping after that time.
	double max_trip_time = max(a_max_trip_time, b_max_trip_time);

	double full_dist = get_closest_approach(a, b, max_trip_time,
			a_max_trip_time, b_max_trip_time);

	// If we're dealing with parallel lines, perturb one of them ever so
	// slightly and run the test again. This handles some edge cases with
	// differing acceleration (basically, one robot overtaking the other)
	// and missiles that can't possibly reach.
	double perturb = 0;
	if (full_dist == CT_DLL_IS_PARALLEL) {
		perturb = 1e-9;
		full_dist = get_closest_approach(a, b, max_trip_time,
				a_max_trip_time, b_max_trip_time, perturb);
	}

	// The search doesn't work if the lines are parallel, and the 
	// perturbation above should have made it go a bit off course so it's
	// no longer parallel; if it didn't, something's wrong.
	assert (full_dist != CT_DLL_IS_PARALLEL);

	// If we're dealing with stationary points, then no amount of time will
	// make them go from crossing to not-crossing or vice versa, and since
	// stationary points have zero velocity, return the shortest possible
	// time in order that they don't move.
	// (NOTE: Changed so that if we're stationary and far apart, we still
	//  are allotted the maximum time to actually turn, reverse, whatnot.
	//  In other words, we return maximum if the points are further apart
	//  than the minimum radius, otherwise 0.)
	if (full_dist == CT_DLL_ONLY_POINTS) {
		if (a->get_pos().sq_distance(b->get_pos()) > collision_squared)
			return(-1); // No crash
		else	return(0);  // Crash at 0
	}

	/*if (missile_hack) {
		cout << "Last check: Full dist is " << full_dist << " and collision_squared is " << collision_squared << endl;
		cout << "Coordinate data: begin (" << b->get_pos().x << ", " << b->get_pos().y << ") end ";
		coordinate b_end = b->predict_motion(b->get_pos(), min(max_trip_time, b_max_trip_time));
		cout << "(" << b_end.x << ", " << b_end.y << ") target (" << 
			a->get_pos().x << ", " << a->get_pos().y << ")" <<
			endl;
		cout << "Line-point distance test: " << endl;
		coordinate p = intersect_test.closest_line_point(b->get_pos(), 
				b_end, a->get_pos(), true);
		cout << "\t" << p.sq_distance(a->get_pos()) << endl;

		cout << "Endpoint distance tests: " << b->get_pos().sq_distance(a->get_pos()) << ", m: " << b->get_pos_at(0.5).sq_distance(a->get_pos()) <<", e: " << b_end.sq_distance(a->get_pos()) << endl;

	}*/

	// If the lines don't intersect at full blast and don't come near
	// each other enough to let the robots collide, there's no reason to
	// check any further.
	// Note that this may fail if robots are already inside each other.
	
	// DEBUG:
	// Check instead of aborting. If we get a contradiction, abort right
	// out.. so that we can figure out what's going on.

	// Hm, seems to be a radius-diameter thing. I don't really like that,
	// because I can't check if that's the case.. but tests seem to show
	// that by multiplying by 2^2 (since it's squared), the problems go
	// away.

	bool fdist = (full_dist > (4 * collision_squared));
	// Comment this out if you want to test. It'll break on assert if
	// the distance check is wrong.
	if (fdist)
		return(-1);

	//cout << "Fulldist: " << full_dist - collision_squared << endl;
	//cout << "Fulldist details: F: " << full_dist << ", CS: " << collision_squared << endl;

	//cout << "Ready for rootfind." << endl;

	// Okay. If we get here, we know they'll either collide end-to-end,
	// crash in making a T-intersection, or make a T-intersection but
	// separate enough not to crash. Find out the point where they do crash,
	// or come closest.
	
	// We used to use binary search, but because of a lot of corner cases,
	// we now use the secant method instead. Note that if we ever move to
	// the event-queue case, we must remove the prior heuristics so that
	// they don't screw up our long-term prediction.
	
	// The return values are -1 for no crash, otherwise the time value of
	// the collision.
	
	double d;
	double past = 0, cur = min(a_max_trip_time, b_max_trip_time);
	double past_record, cur_record;
	double error = accuracy; // Steerable
	int maxiter = 35;    // Ditto

	// Start the secant evaluation
	coordinate begin_a = a->get_pos(), begin_b = b->get_pos();

	past_record = begin_a.sq_distance(begin_b) - collision_squared;

	// Check if we're already inside the missile check radius. This should
	// only happen if we fire at an enemy while collided with it (or very
	// very close) and collision radius is less than missile check radius.
	// We have to check that we're firing at the same enemy that we
	// collided with, to handle the case where we collided with one and
	// we're now shooting at someone else.
	if (past_record < 0) {
		// Get closest point on the projected line. If it's at t = 0,
		// that means we're either incredibly unlucky (collision_radius
		// is zero) or that we're not firing at the one it's detecting
		// (since the missile is traveling away from it).
		// We don't deal with the former case, but the latter. Otherwise
		// (if t != 0) return 0 to show there's a hit.
		coordinate closest = intersect_test.closest_line_point_to_line(
				b->get_pos(), b->get_pos_at(b_max_trip_time),
				a->get_pos(), a->get_pos_at(a_max_trip_time));

		if (closest == b->get_pos())
			return(-1);
		else {
			assert (!fdist);
			return(0);
		}
	}

	// Otherwise, if we're within the margin (missile enters just at the
	// collision point), return 0.
	if (fabs(past_record) < error) {
		assert (!fdist);
		return(0);
	}

	coordinate advanced_a, advanced_b;

	// We should replace this with the other secant method. We can't use
	// the regula falsi variation because we can't bracket the distance
	// function.

	int counter;
	for (counter = 0; counter < maxiter; ++counter) {

		cur_record = a->get_pos_at(cur).sq_distance(b->get_pos_at(cur)) 
			- collision_squared;

		// A "time until crash" of less than zero is invalid, so make
		// sure it'll never have a root there.
		if (cur < 0)
			cur_record = fabs(cur_record) + 1;

		d = (cur - past) / (cur_record - past_record) * cur_record;

		if (!finite(d))
			return(-1);

		//cout << "Rootfind: Iter " << counter << ", error " << d << " at value " << cur << " (check value " << cur_record << " ) " << endl;

		// Now check if we found something. If so, and it's not out of
		// bounds, okay. If we found the second root, make note of the
		// location of it and then trace backwards using a bracketing
		// method.

		// Used to be if fabs(d) ..., but this works better.
		if (fabs(cur_record) < error) break;

		past = cur;
		past_record = cur_record;

		cur = cur - d;
	}

	bool second_root = false;
	double at_minus_a_bit;

	if (/*fabs(d) < error ||*/ fabs(cur_record) < error) {
		//cout << "Rootfind: Beneath threshold at " << counter << endl;
		// Find out if it's the second root, where it leaves the robot.
		if (cur > 0) {
			at_minus_a_bit = (a->get_pos_at(cur - (2 * error))).
				sq_distance(b->get_pos_at(cur - (2 * error)))
				- collision_squared;
			//cout << "Check: " << at_minus_a_bit << " and at curpos " << cur_record << endl;
			if (at_minus_a_bit < 0) {
			//	cout << "Second root at " << cur << endl;
				second_root = true;
			}
		}

		// If it spies too far ahead, that's not good either.
		if ((cur > min(a_max_trip_time, b_max_trip_time) ||
				cur < 0) && !second_root) {
	/*		cout << "But t = " << cur << " which is beyond"
				<< " bounds of " << min(a_max_trip_time,
						b_max_trip_time)
				<< endl;*/
			return(-1); // Failed to converge
		}

		//cout << "f(" << cur << ") = " << cur_record << endl;

		if (!second_root) {
			assert (!fdist);
			return(cur);
		}
	}

	if (!second_root) {
	//	cout << "Nothing found by rootfind." << endl;
		return(-2); // No collision - failed to converge
	} else {
		// Okay, we found the second root. Now do a binary search
		// (too lazy to use anything like regula falsi) to find the 
		// first root. It's not a proper bisection search either, fix
		// later.

		// Ridder mark.

		double fx, fy, fz, x, y, z, fznew, znew;
		double retval = -1, f_retval;
		double minimum = 0, maximum = cur - (2 * error);

		x = minimum; y = maximum;
		fx = a->get_pos().sq_distance(b->get_pos()) - collision_squared;
		fy = at_minus_a_bit;

		//cout << "FX is " << fx << endl;

		int calculations = 0;
		for (counter = 0; counter < maxiter && retval == -1; ++counter) {
			if (fabs(fx) < error || fabs(x - y) < error) {
				retval = x;
				f_retval = fx;
				continue;
			}

			assert(sign(fx) != sign(fy));

			z = (x + y) / 2;
			fz = a->get_pos_at(z).sq_distance(b->get_pos_at(z))
				- collision_squared;
			++calculations;

			if (fabs(fz) < error) {
				retval = z;
				f_retval = fz;
				continue;
			}

			znew = z + (z-x) * (sign(fx - fy) * fz)/(sqrt(fz * fz -
						fx * fy));

			fznew = a->get_pos_at(znew).sq_distance(b->get_pos_at(
						znew)) - collision_squared;
			++calculations;

			if (fabs(fznew) < error) {
				retval = znew;
				f_retval = fznew;
				continue;
			}

	//		cout << "Iteration " << counter << ": " << fznew 
	//			<< endl;

			// Propagate for next round.

			if (sign(fznew) != sign(fz)) {
				x = z;
				fx = fz;
			}
			if (sign(fznew) != sign(fx)) {
				y = znew;
				fy = fznew;
			} else if (sign(fznew) != sign(fy)) {
				x = znew;
				fx = fznew;
			}

			if (fabs(x - y) < error) {
				retval = z;
				continue;
			}
		}

		assert (retval != -1); // This shouldn't happen; we're
			// bracketed, after all.

		if (retval != -1) {
			if (retval > min(a_max_trip_time, b_max_trip_time) ||
					retval < 0)
				return(-1); // Out of bounds
		//	cout << "Found collision after " << calculations << " evals. Details: f(" << retval << ") = " << f_retval << endl;
			assert (!fdist);
			return(retval);
		}

		return(-1);
	}
}

// Collision radius should not be squared. Note that it'll collide further
// away from the wall than it really ought to for the corners, but the
// alternative is much slower and it's not going to make much of a difference
// anyway.
double collider::determine_edge_crash_point(const double accuracy, Unit * a, 
		const double a_max_trip_time, const int collision_radius, 
		const coordinate & arena_max) const {

	// This is rather simple. First we check if we're going to collide with
	// the wall at all. If not, get outta here. If we're going to collide,
	// do a binary search to find out when. We handle the crash radius by
	// just shrinking the wall by that much.
	
	// First check if we're going to collide at all.
	
	/*cout << "At " << a->get_pos().x << ", " << a->get_pos().y << ", coll. radius " << collision_radius << " to go to " << a->get_pos_at(a_max_trip_time).x << ", " << a->get_pos_at(a_max_trip_time).y << " AMT: " << a_max_trip_time << endl;
	cout << "F_high is ";*/

	double f_high = arena_wall_dist(a->get_pos_at(a_max_trip_time), 0, 0,
			arena_max.x, arena_max.y, collision_radius);
	/*cout << f_high << endl;
	cout << "0.099 is " << a->get_pos_at(0.099).x << ", " << a->get_pos_at(0.099).y << endl;*/

	if (f_high > 0)
		return(-1);	// Not gonna collide.

	// DEBUG
	//double f_low = arena_wall_dist(a->get_pos(), 0, 0, arena_max.x, 
	//		arena_max.y, collision_radius);
	//assert (arena_wall_dist(a->get_pos(), 0, 0, arena_max.x, arena_max.y, 
	//			collision_radius) > 0);
	double f_low = 1;	// Intersection cares only about signs, and
				// we shouldn't be already embedded, so saying
				// 1 is as good as anything (and we are freed
				// of one calculation).
	assert (f_low > 0);

	// Do the bisection dance!
	double nmid, nhigh = a_max_trip_time, nlow = 0, last_inside = -1;

	while (nlow < nhigh) {
		nmid = (nlow + nhigh) / 2.0;

		double f_normmid = arena_wall_dist(a->get_pos_at(nmid), 0, 0, 
				arena_max.x, arena_max.y, collision_radius);

	//	cout << "Edge bracket: " << nlow << ", " << nmid << ", " << nhigh << endl;
	//	cout << "Edge: f(" << nmid << ") = " << f_normmid << endl;

		if (f_high <= 0 ^ f_normmid <= 0) {
			// Early break if we have the solution.
			if (f_normmid <= accuracy) {
				//cout << "Within limits." << endl << endl;
				return(nmid);
			}
			
			// We're in the positive bracket
			last_inside = nmid;
			nlow = nmid;
			f_low = f_normmid;
		} else {
			// We're in the negative bracket
			nhigh = nmid;
			f_high = f_normmid;
		}
	}

	// This rarely happens, if ever.
	//cout << "Last inside: " << last_inside << endl;
	//cout << endl;

	// Perhaps return the last confirmed outside (or just inside)
	// variable? Yes, this avoids a case where it gets stuck just outside
	// and can't get inside because determine_edge_crash_point triggers at
	// 0 no matter what, afterwards.

	// Safeguard the case where it starts off way outside and thus can
	// never get inside again. This should never happen.
	assert(last_inside != -1);
	//assert (1 != 1);

	return(last_inside);
}

// This function takes time unit and crash arrays, as well as the robots and
// some auxiliary information, and updates the arrays so that the robots, when
// advanced by the proper time amount, will just touch the crashing object (if
// there's a crash).
// Note: There may be skipping problems with particular very complex reactions
// (r0 crashes with r1 at time t, but r1 is found to crash with r2 at time 
//  q < t, then r0 will stop still), but I haven't found any quick algorithm
// (below n^3 log n) to handle this robustly. It shouldn't make a difference
// for the kind of time slices we're dealing with, but beware.
void collider::set_track_limits(list<robot *> & robots, int robot_radius, 
		int crash_range, const coordinate arena_size) const {

	// Accuracy of the root-finder.
	const double accuracy = 1e-3;

	// The point of this is to cancel out the error in the root-finding
	// algorithm, so that collisions don't get recorded multiple times.
	// When we do collide, we subtract this from the time of collision,
	// so that if the collision errs inside the radius, it gets moved
	// slightly outside. The "close enough" parameter of the root finder
	// should be set so that anything that passes but is too far inside
	// will be negated by subtracting this value from the time of impact.
	double safety_deg = accuracy * 0.9;

	// First alter the time units based on wall collisions.
	
	int counter;

	//cout << "--- Beginning " << endl;
	list<robot *>::iterator p_outer, p_inner;

	counter = 0;

	robot * outer;	// So that it'll be easy to change if we get the stats
			// fixed.

	// We wouldn't have to do this ugliness if stats were separate from the
	// robots.
	for (p_outer = robots.begin(); p_outer != robots.end(); ++p_outer) {

		outer = *p_outer; // Deref

		// Assume we haven't crashed - we'll update this later if
		// the opposite is true.
		outer->clear_crash();

		// This shouldn't trigger, since it's after all only a list of
		// live robots.
		assert (!outer->dead());

		double cand = -1;
		// There's a bug with expanse's factor > 1. But with it == 1,
		// there's no cache! Bluesky fix.
		double expanse = 1 * outer->time_units();
		double start = 0;

		// Check whether the cache tells us that we're going to collide.
		if (!outer->has_changed_route()) {
			cand = outer->get_edge_collision_time() -
				outer->get_time();

			if (outer->get_does_crash()) {
				if (cand < 0)
					cand = -1;
			} else {
				if (cand >= 0) {
					if (cand > outer->time_units())
						cand = -2; // No crash
					else
						start = cand;
						cand = -1;
				}
			}
			//cout << "DET: CACHE: cand is " << cand << endl;
		}

		// If not, determine the crash point.
		if (cand == -1) {
			cand = determine_edge_crash_point(accuracy, outer, 
					expanse, start,	arena_size);
			//cout << "DET: Real cand is " << cand << endl;

			// Update the cache.
			if (cand >= 0) {
				outer->set_rel_edge_collision_time(cand);
				outer->set_does_crash(true);
			}
			else {
				outer->set_rel_edge_collision_time(expanse);
				outer->set_does_crash(false);
			}

			// And let it know the cache is OK.
			outer->clear_altered();

		}

		if (cand == -2) cand = -1;

		// If it crashes (not -1) and it's earlier than our limit,
		// update the limit.
		if (cand != -1 && cand < outer->time_units()) {
			//cout << "CRASH TIME at " << cand << endl;
			outer->set_time_units(cand - safety_deg);
			outer->set_crash(CRASH_EDGE);
		}

	}

	// Radius not diameter since when they collide, there'll be half diam.
	// towards the enemy, and half of the enemy's diameter towards the
	// collider. DONE: Figure out if the collision radius is greater than
	// the true robot radius. Couldn't figure it out, but it shouldn't be,
	// since in that case there's nothing else that relies on robot radius
	// as opposed to crash range, thus making robot radius undefined.
	const int squared_range = robot_radius * robot_radius;

	// Then narrow down based on unit-unit collisions. 0.5n^2.
	// Since crashes are symmetric (if a crashes into b, b crashes into a),
	// we don't need to check b vs a if we've already tested a vs b.
	robot * outerptr;

	for (p_outer = robots.begin(); p_outer != robots.end(); ++p_outer) {

		outer = *p_outer;

		//if (outer->dead()) 
		//	continue;
		
		// Heuristics go here, such as don't care if both robots
		// are motionless, etc.

		double potential_crash_time = 1e100; // Suitably large number
		robot * crashed_with = NULL;

		// Early check to remove those that definitely can't crash.
		// Bluesky here would be to adjust the cache based on expected
		// time until it does get within the "can crash" range, turning
		// this from N calls to (worst case) log N calls.
		// Perhaps even better would be a grid-based structure for
		// automatic pruning, but there will have to be a lot of 
		// missiles flying before it's worth it.
		double outer_reach = outer->get_worst_case_abs_speed() *
			outer->time_units();

		// Ugly stub.
		p_inner = p_outer;
		++p_inner;

		for (; p_inner != robots.end(); ++p_inner) {
			robot * inner = *p_inner; // ditto

			//if (inner->dead()) 
			//	continue;
			
			double inner_reach = inner->get_worst_case_abs_speed() 
				* inner->time_units();

			if (inner->get_pos().distance(outer->get_pos()) -
					(inner_reach + outer_reach) > 
					squared_range) continue;

			//cout << "Now to check in detail." << endl;
			// Determine time unit of crash, if any.
			double crash_at = determine_crossing_point(accuracy,
					outer, inner, outer->time_units(),
					inner->time_units(), squared_range, 
					squared_range, true);
			//cout << "Crash_at says " << crash_at << endl;

			// Was there any? If there was, and it's earlier than
			// our past candidate for crash, update the record.
			// (This mitigates the complex problem slightly.)

			if (crash_at >= 0 && crash_at < potential_crash_time) {
				// Yes, update the time unit limitations
				crashed_with = inner;
				potential_crash_time = crash_at - safety_deg;
			}

		}

		// Check if we did crash. If so, update the limitation on both
		// robots and set the crashed flag.
		if (crashed_with != NULL) {
			// Check if we're outside the edge at this time. This
			// may happen in nonmonotonic cases, particularly
			// where the robot is accelerating away from the edge
			// but hasn't overcome forwards momentum yet.
			double a_minimum = min(outer->time_units(), 
					potential_crash_time);
			double b_minimum = min(crashed_with->time_units(),
					potential_crash_time);

			double a_edge_cand, b_edge_cand;
			int edge_crash = 0;

			a_edge_cand = determine_edge_crash_point(accuracy, 
					outer, a_minimum, //crash_range
					0, arena_size);
			b_edge_cand = determine_edge_crash_point(accuracy, 
					crashed_with, b_minimum, 0, 
					arena_size);

			// Find out which crashes first.
			if (a_edge_cand != -1 && a_edge_cand < a_minimum) {
				if (b_edge_cand != -1 && b_edge_cand < 
						b_minimum) {
					if (a_edge_cand < b_edge_cand)
						edge_crash = -1;
					else	edge_crash = 1;
				}
				edge_crash = -1;
			} else
				if (b_edge_cand != -1 && b_edge_cand < 
						b_minimum)
					edge_crash = 1;

			if (edge_crash == -1) { // A crashes first
				outer->set_time_units(a_edge_cand);
				outer->set_crash(CRASH_EDGE);
			}
			if (edge_crash == 1) { // B crashes first
				crashed_with->set_time_units(b_edge_cand);
				crashed_with->set_crash(CRASH_EDGE);
			}

			if (edge_crash == 0) { // no crash
				//cout << "Setting TU to " << potential_crash_time << " after crash with " << crashed_with << endl;
				outer->set_time_units(min(outer->
							time_units(),
							potential_crash_time));
				crashed_with->set_time_units(min(crashed_with->
							time_units(),
							potential_crash_time));
				outer->set_crash(crashed_with->get_UID());
				crashed_with->set_crash(outer->get_UID());
				// Vector/momentum stuff go here if you want to
				// make an in-space version.
				crashed_with->set_crash_target_vel(outer->
						get_throttle());
				outer->set_crash_target_vel(crashed_with->
						get_throttle());
			}
		}
	}
	
	// All done!
}

// Should this be turned into vector<Unit *> & instead, later, to handle
// dead robots? Hm.
// Another possible solution is this: missiles collide with robots like any
// other unit, but on crash, check all robots for splash damage, then detonate
// (and the robots don't stop). However, that requires register_crash to pass
// a list of all robots (for the missile to check which pass within the range),
// which would be messy.
void collider::handle_missile_crashes(vector<robot> & all_robots,
		list<robot *> & live_robots, double full_timespan, 
		list<missile> & missiles, int max_hit_radius, 
		const coordinate arena_size, double absolute_time_at_start, 
		blasts & explosions) const {

	const double accuracy = 1e-3; // Root-finder accuracy

	// As by ATR2 rules, missiles don't affect a robot's motion (unless
	// the latter explodes).
	
	// So, to do this we first check all the missiles against edges. Those
	// that will exit the edge are marked crashed (crashed missiles are
	// removed from the missile list). Crashed or not, we then check these
	// against robots. 
	
	// We might speed this up by preallocating the vectors used to house
	// "did this missile crash", etc.
	
	// (Note: missile hits are rare. Thus, being elaborate upon one hurts
	// little, but taking a long time to determine whether there'll be a
	// collision is bad.)
	
	// A better idea: Just check if it's outside of the arena. If so, remove
	// it. This simulates the condition that the missiles are just flying
	// off the arena edge, not crashing into anything.
	// BLUESKY: Make it crash if there's a wall around the arena high enough
	// for missiles to not fly off (parameter). Since robots crash, logic
	// would say there's already a wall, but that's not how ATR2 works.

	// (Maybe we could do a regula falsi type search by first finding the
	//  closest point on the line, and then the time at which we're at that
	//  point? (Draw a vertical line through; any position to the left is -
	//  and any to the right is +) Nah, because we want to have 
	//  generalization in case the two robots are turning.)

	// Check against robots. 
	// Note: in ATR2, we extend the line to infinity to get the best hit.
	// (I think.)
	
	// Note that max_hit_radius is a half-open interval, so you can't
	// actually hit at max_hit_radius. Thus the -1.
	int squared_dist = (max_hit_radius - 1) * (max_hit_radius - 1);

	//cout << "MIS: --- missiles ---" << endl;
	//cout << "MIS: --- missiles ---" << endl;

	list<coordinate> explosion_points;
	list<missile>::iterator cur;
	list<robot *>::iterator xrobocur;
	vector<robot>::iterator all_robot_cur;

	for (cur = missiles.begin(); cur != missiles.end(); ++cur) {
		//cout << "MIS/A: Dealing with missile at " << cur->get_pos().x << ", " << cur->get_pos().y << endl;

		robot * originator = NULL;

		for (xrobocur = live_robots.begin(); xrobocur != 
				live_robots.end(); ++xrobocur) {
			robot * robocur = *xrobocur;

			// If this is the shooter, don't check (or it'll
			// explode before it can get out of the tube).
			// When we go to lists, just look up UID against the
			// master vector<robot> part after this, since missiles
			// have negative UID of whoever launched them.
			if (robocur->get_UID() == cur->get_shooter()) {
				originator = &*robocur;
				continue;
			}
			// If it's dead, don't check either (as we can't collide
			// with the dead). (We can't use Units, since Units
			// can't register damage, crash, etc.)
			assert (!robocur->dead());

			double missile_hit_time = cur->get_hit_time();
			if (missile_hit_time < 0)
				missile_hit_time = full_timespan;

	//		cout << "MIS/A: Squared dist is " << squared_dist << endl;
	//		missile_hack = true;

			// Determine impact time
			double crash_at = determine_crossing_point(accuracy,
					&*robocur, &*cur, robocur->time_units(),
					missile_hit_time, squared_dist, 0, 
					true);

			// If the root-finder failed to converge, assume the
			// explosion happened at the very end. This shouldn't
			// happen, but we can't guarantee that the root-finder
			// does get the solution.
			if (crash_at == -2 && cur->get_hit_time() == -1) {
				//assert (1 != 1);
				if (cur->get_pos_at(full_timespan).
						sq_distance(robocur->get_pos())
						< squared_dist)
					crash_at = full_timespan;
			}

	//		missile_hack = false;

	//		cout << "MIS/A: That missile crashes at " << crash_at << endl;

			// FIXED: Was crash_at < missile_hit_time which got
			// caught in the edge case that it hit at 1.
			if (crash_at >= 0 && crash_at <= missile_hit_time) {
				cur->update_time_on_target(crash_at);
				cur->set_target(&*robocur);
				//cout << "MIS/A: Updating crash time from " << missile_hit_time << " to " << crash_at << endl;
			}
		}

		// If we hit something, let the robot that did the shooting
		// know. Originator is NULL if we didn't find the originating
		// bot.
		if (cur->get_hit_time() != -1 && originator != NULL)
			originator->record_bot_shot(cur->get_target());
	}

	// Okay, now we have set the crash parameters. For all those that
	// *did* hit something, for all bots in the proximity, inflict damage
	// and remove the missile.
	
	cur = missiles.begin();

	while (cur != missiles.end()) {
		//cout << "MIS/B: -- Missile -- " << endl;
		// Since we dealt with missiles that flew off the edge, the
		// only "crashed" missiles are those that hit something. Thus,
		// don't bother about missiles that didn't.
		if (cur->get_hit_time() == -1) {
			++cur;
			continue;
		}

		// Find our position at the time, and the time itself
		// If we used the hack to set the explosion point directly,
		// hack t_deton to 0. Perhaps we should refactor this into
		// two different routines, and one extra that goes explode(
		// time, coordinate, bool overrule_coordinate).
		double t_deton;
		coordinate impact_point, exp_point;

		t_deton = cur->get_hit_time();
		impact_point = cur->get_pos_at(t_deton);
		// Now extrapolate to figure out where the missile explodes 
		// within the body of the robot.
		exp_point = cur->get_explosion_point(impact_point, t_deton, 
				cur->get_target()->get_pos_at(t_deton));

		//cout << "MIS/B: Impact point " << impact_point.x << ", " << impact_point.y << " at t = " << t_deton << endl;

		// Add the explosion to our list of explosions.
		explosions.add_blast(exp_point, B_MISSILE, max_hit_radius);

		//cout << "MIS/B: Hit time: " << t_deton << endl;

		double multiplier = cur->get_damage_multiplier();

		// Determine originator so we can credit the damage caused.
		robot * originator = NULL;

		// DONE: Master vector here - credit damage even if the
		// inflicting robot is dead.
		for (vector<robot>::iterator mrobocur = all_robots.begin(); 
				mrobocur != all_robots.end(); ++mrobocur)
			if (mrobocur->get_UID() == cur->get_shooter())
				originator = &*mrobocur;

		// The splash damage can affect all robots (except the dead 
		// ones), so iterate through them.
		for (xrobocur = live_robots.begin(); xrobocur != 
				live_robots.end(); ++xrobocur) {

			// No need to check for alive, since dead robots
			// would have tripped the assert above.
			//if ((*xrobocur)->dead()) continue;

			// Because this happens so rarely, we'll use
			// real distances.
			coordinate robot_at_time = (*xrobocur)->get_pos_at(
					t_deton);

			// Register a blast done by the shooter at the
			// position of the missile impact.
			int origin = cur->get_shooter();

			double damage = (*xrobocur)->register_blast(
					exp_point.distance(robot_at_time), 
					multiplier, max_hit_radius, origin);

			// Credit the damage if we know who did it.
			if (originator != NULL)
				originator->record_damage_to_others(
						**xrobocur, damage);
		}

		cur = missiles.erase(cur); // And the missile's gone.
	}

	// Finally, remove those that'll fly off the edge (and haven't hit
	// anything yet). Does this cut too much into speed? If so optimize:
	// it should be easy, since missiles have a fixed speed, to have a
	// if distance to closest wall < reach then check further.
	//
	// The hit doesn't seem to be too bad, so we don't.
	
	cur = missiles.begin();
	while (cur != missiles.end()) {
		coordinate thenpos = cur->get_pos_at(full_timespan);

		if (thenpos.x < 0 || thenpos.y < 0 || thenpos.x > arena_size.x
				|| thenpos.y > arena_size.y)

			cur = missiles.erase(cur);
		else    ++cur;
	}
}

// This handles mine crashes by detecting collisions between robots and mines. 
// I may transition to a separated "only notify the mines that they should 
// explode" approach later.
// (Could need some refactoring wrt missiles; missiles have set_target,
// here we do it manually, and we look up the originator twice..)
// Again we use vector<robots> although this will slow things down; the Unit
// can't register damage and doesn't have an UID. Maybe refactor this somehow,
// in the future.
void collider::handle_mine_crashes(vector<robot> & all_robots, 
		list<robot *> & live_robots, double full_timespan,
		list<mine> & mines, blasts & explosions) const {

	// Root-finder accuracy
	const double accuracy = 1e-3;

	const int mine_blast_radius = 35; // ATR2 constant

	// For all the mines, do this
	// 	For all the robots except the one identified as friend,
	// 		If it gets within the detection radius (and does so
	// 		sooner than any other), set that mine as ready to 
	// 		explode on the time unit in question
	// 	Next
	// Next
	
	list<mine>::iterator cur;
	list<robot *>::iterator robot_xcur;

	// Assumes all robots are equally large.
	int robot_squared_dist = square((*live_robots.begin())->get_radius());

	bool exploded = false; // Did a mine trigger?

	for (cur = mines.begin(); cur != mines.end(); ++cur) {

		// BLUESKY: Find out the fastest a robot can get within range.
		// If a robot is outside of this area, there's no point in
		// even considering it. Making robot_cur the outer loop may
		// also be good.
		// 	It's not going to do us any good unless we move the
		// 	robots to the outer loop.

		// Hack for requested explosion.
		// (Or should this be 0?)
		if (cur->should_explode())
			cur->update_hit_time(full_timespan);

		// Get mine sensitivity. It's squared because one square op
		// is much faster than a bunch of square roots.
		int squared_distance = cur->get_radius() * cur->get_radius();

		robot * originator = NULL, * target = NULL;;

		for (robot_xcur = live_robots.begin(); robot_xcur != 
				live_robots.end(); ++robot_xcur) {

			robot * robot_cur = *robot_xcur;

			// No friendly fire please! But do record so we can
			// let this bot know someone stepped on the mine.
			// (For stats purposes)
			if (cur->layer_UID() == robot_cur->get_UID()) {
				originator = &*robot_cur; 
				continue;
			}

			// No care about dead robots either. Note that this
			// happens after the originator check so that even if
			// the robot's dead, if a mine hurts another, he's
			// credited.
			assert (!robot_cur->dead());

			double mine_hit_record = cur->get_hit_time();
			if (mine_hit_record < 0)
				mine_hit_record = full_timespan;

			// Find out when it crashes, if ever.
			// If robots teleport (in some later version), it'll
			// be important to adjust r_s_d so that it gets the
			// full impact damage.
			double crash_at = determine_crossing_point(accuracy,
					&*robot_cur, &*cur,
					robot_cur->time_units(), 
					mine_hit_record, robot_squared_dist,
					squared_distance, true);

//			cout << "The return value for that was " << crash_at << endl;

			if (crash_at >= 0 && crash_at < mine_hit_record) {
				cur->update_hit_time(crash_at);
				target = &*robot_cur;
			}
		}

		// If we didn't find the originator, that means one of the
		// robots that are now dead did it. Search for him and credit
		// him.
		// No point in doing this check if we didn't explode.
		if (target != NULL) {
			if (originator == NULL)
				for (vector<robot>::iterator mrobot_cur = 
						all_robots.begin(); 
						mrobot_cur != all_robots.end();
						++mrobot_cur)
					if (cur->layer_UID() == mrobot_cur->
							get_UID())
						originator = &*mrobot_cur;

			originator->record_bot_mine_hurt(target);
			// This hasn't happened yet, but will happen in the
			// loop below, so do it now while we know who the
			// originator is.
			originator->register_destroyed_mine();
			exploded = true;
		}
	}

	// Then just go through those that are ready to explode, and explode
	// them with the right time setting.
	
	// If no mines were triggered, there's no point in going through the
	// mines yet again.
	if (!exploded) return;

	cur = mines.begin();

	while (cur != mines.end()) {
		// If it doesn't explode, pay it no attention.
		if (cur->get_hit_time() == -1) {
			++cur;
			continue;
		}

		//cout << "Detonating mine laid-by " << cur->layer_UID()
		//	<< endl;

		// Otherwise, get time and space, and register hits.

		double t_deton = cur->get_hit_time();
		coordinate ground_zero = cur->get_pos_at(t_deton);
		// Should this be const radius?
		explosions.add_blast(ground_zero, B_MINE, mine_blast_radius);

		for (robot_xcur = live_robots.begin(); robot_xcur != 
				live_robots.end(); ++robot_xcur) {
			// Dead bots tell of no damage.
			//if ((*robot_xcur)->dead()) continue;

			coordinate robot_at_time = (*robot_xcur)->get_pos_at(
					t_deton);

			// ATR2 constant ahead! The 35 means 35 points of damage
			// at ground zero, 1 point of damage 35 m out.
			// Yet again, the ordnance's ID is negative of that
			// of the one who dispensed it.
			(*robot_xcur)->register_blast(ground_zero.distance(
						robot_at_time), 1,
					mine_blast_radius, cur->layer_UID());
		}

		// The mine, having exploded, is removed from play.
		cur = mines.erase(cur);
	}
}
