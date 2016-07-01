// Scanner sensor. This class includes both a scan function and information
// about the last scan.
// The scanner itself can only detect a single target. It detects targets in
// the order the robots were loaded, meaning that if two robots are inside the
// same scanarc, it'll always return the position of the same of these, unless
// that one is dead.

#ifndef _KROB_SCANNER
#define _KROB_SCANNER

#include "coordinate.cc"
#include "coord_tools.cc"
#include "object.cc"
#include "tools.cc"
#include <list>
#include <iostream>

using namespace std;

class Scanner {

	private:
		// Scanner settings
		int center_hexangle;	// Where are we aimed?
		int span;		// How large is our scanarc?
					// (This is half, as in ATR2)
		int scanner_radius;	// How far can we see?
		int detection_radius;	// How large are robots to our scanner?
					// (How far can we see from the edge?)

		// Results
		const Unit * detected_target;	// or NULL
		unsigned short range_to_target; // 32767 if not found.
		double accuracy;		// -2 at extreme left, 
						//  2 at extreme right.
					//  BLUESKY: Make continuous.
		bool found;			// True if we found something.

		coord_tool close_check;

		void set_not_found();

	public:
		Scanner();
		Scanner(int center_ha, int span_in, int scanrange_in,
				int detection_radius_in);

		bool scan(const list<Unit * > & check_against, 
				const coordinate scanner_pos);

		void set_center_hexangle(int chi) { center_hexangle = chi; }
		void set_span(int si) { span = si; }
		void set_radius(int sr) { scanner_radius = sr; }
		void set_detection_radius(int dr) { detection_radius = dr; }

		bool detected() { return(found); }
		const Unit * target() { return(detected_target); }

		int get_accuracy();
		int get_range() { return(range_to_target); }
		int get_span() { return(span); }
		int get_center_hexangle() { return(center_hexangle); }

		int get_radius() const { return(scanner_radius); }
};

// -- //

void Scanner::set_not_found() {
	found = false;
	detected_target = NULL;
	range_to_target = 32767;
}

Scanner::Scanner() {
	set_not_found();
	center_hexangle = 0;
	span = 0;
	scanner_radius = 0;
	detection_radius = 0;
}

Scanner::Scanner(int center_ha, int span_in, int scanrange_in,
		int detection_radius_in) {
	set_not_found();
	center_hexangle = center_ha;
	span = span_in;
	scanner_radius = scanrange_in;
	detection_radius = detection_radius_in;
}


// But note special case when we're looking North; it seems to lose its grip.
// DONE: Investigate and fix.
bool Scanner::scan(const list<Unit *> & robots, const coordinate scanner_pos) {

	// The scanner algorithm works like this:
	// 	For each robot, we find its angle in the polar coordinate system
	// 	that has ourselves as center. If that angle is inside
	// 	center_hexangle +/- span (and not too far away for us to spot
	// 	it), then the robot must be inside our scanarc, and so we 
	// 	calculate accuracy and return.
	//
	// 	Otherwise, we check which line (left or right) the robot is
	// 	closest to, get the closest point on the relevant span line,
	// 	and check if the robot's close enough that we detect it (since
	// 	robots are round and not point sources).
	//
	// The worst case is when we check against a robot that isn't there,
	// because it'll fail the first check and also the second.
	
	bool found_one = false;
	coordinate found_pos;
	double found_angle = -1;

	detection_radius = 12; //See ATR2.pas with detection_radius = 14 and
	// if (r < detection_radius - 2) in the scanner
	
	int squared_radius = detection_radius * detection_radius;
	double record_dist = -1;

	// DONE: Check if it's in fact "return closest object to ourselves
	// that's in the scanner's range". It is.
	// (Game equipment idea: chaff: returns true with p = 0.5, withstands
	//  two or three successful scans.)
	for (list<Unit *>::const_iterator cur_ref = robots.begin(); cur_ref !=
			robots.end(); ++cur_ref) {

		// Aliasing to fit our earlier code and avoid ugliness like
		// (*cur_ref)->.
		Unit * cur = *cur_ref;

		double cand_distance = cur->get_pos().sq_distance(scanner_pos);
		if (cand_distance > record_dist && record_dist != -1)
			continue;

		//cout << "INFORMATSIYA: " << cur->get_pos().x << ", " << cur->get_pos().y << "\t\tus: " << scanner_pos.x << ", " << scanner_pos.y << endl;

		// If it's me, forget it.
		if (cur->get_pos() == scanner_pos) continue;

		//cout << "Passed #1" << endl;

		// If it's cloaked, ditto.
		if (cur->is_cloaked()) continue;

		//cout << "Passed #2" << endl;
		//cout << "Scanner radius is " << scanner_radius << endl;
	
		// Check that the robot isn't too far away for us to detect.
		if (cur->get_pos().distance(scanner_pos) > scanner_radius) 
			continue;
		
		//cout << "Passed #3" << endl;

		// Get the relative angle.

		coordinate normalized = cur->get_pos() - scanner_pos;

		// Perhaps some monotonic function of angle could be used 
		// instead. Unlikely, but perhaps a precalc table accurate to
		// within 1 m at 1500 m.
		double angle = radian_to_hex(atan2(normalized.y, normalized.x));

		/*cout << "In a shallow field, we wait." << endl;
		cout << "Detection radius is " << detection_radius << endl;*/

		// Is it within the span?
		if (!hexangle_within(center_hexangle - span,
					center_hexangle + span, angle)) {
	//		cout << "--SCAN: Is-not-within " << center_hexangle - span << ", " << center_hexangle + span << " with angle " << angle << endl;
			// No, check if we can still detect it.

			// Find out which span it is closest to (questionable
			// code?) and then check if it's close enough.

			int approximant;

			// Find out which edge of the scanning arc is closest.
			// Consider the case where the center of the arc is
			// north. Then anything to the west belongs to the left
			// side of the arc, and anything to the east belongs
			// to the right side of the arc. The tiebreak is of
			// no importance, since no arc can be large enough.

			if (angle_within(center_hexangle - 128, 
						center_hexangle, angle, 256))
				approximant = center_hexangle - span;
			else	approximant = center_hexangle + span;

			angle = approximant;

			// Calculate the end points for our scanner at the
			// span in question.
			// (Optimization idea: offload these so we look them
			//  up, since there are only two possibilities.)
			coordinate end_of_line = scanner_pos;
			end_of_line.x += scanner_radius * cos(hex_to_radian(
						approximant));
			end_of_line.y += scanner_radius * sin(hex_to_radian(
						approximant));

			// Get the closest point on that line, and figure out
			// the distance to the robot.
			// (Then break if it's too far away)
			double real_dist = close_check.dist_closest_point(
					scanner_pos, end_of_line, cur->get_pos(),
					true);

			//cout << "Real distance: " << real_dist << endl;

			if (real_dist > squared_radius)
/*					close_check.dist_closest_point(scanner_pos,
						end_of_line, cur->get_pos(),
						true) >= squared_radius)*/
				continue;
		} else {
	//		cout << "--SCAN: Is-within" << endl;
		}

		// Okay, if we got here, it's either within the span, or it's
		// near enough that we can detect it anyhow. Thus we know it
		// should be registered, so set the relevant data to skip out
		// of the loop.

		if (cand_distance < record_dist || record_dist == -1) {
		//	cout << "Found one set" << endl;
		//	cout << "Angle is " << angle << endl;
			found_one = true;
			detected_target = *cur_ref;
			found_pos = cur->get_pos();
			found_angle = angle;
			record_dist = cand_distance;
		}
	}

	/*if (found_one) {
		cout << "SCAN: something found." << endl;
	} else
		cout << "SCAN: nothing found." << endl;*/


	// If we get here, either we exhausted all the bots without finding any,
	// or we've found any (depending on what found_one says).

	found = found_one;

	if (!found_one) {
		set_not_found();
		accuracy = 0; // ??? is this even touched? Yes, and it's 0
		return(false);
	}

	// So we found something. Set range and accuracy.
	range_to_target = found_pos.distance(scanner_pos);

	// Determine accuracy. To determine accuracy, we need to center so that
	// the center of span is zero. Then we divide by span and get a
	// normalized number -1..1.
	// (Well, we do it differently now; we push the start angle (-span) to
	//  0, find the relative position of the found angle wrt the interval
	//  between that and span*2, and then normalize to -1..1. If this is
	//  TDS, we'll try to optimize later.)
	
	double start_angle = center_hexangle - span, 
	       end_angle = center_hexangle + span;
	double to_check = hexangle(found_angle - start_angle);
	end_angle = hexangle(end_angle - start_angle);
	double centered = 2 * to_check / end_angle - 1;

	// This is now symmetric about 0, so we need only consider the half:
	// 	accuracy	fraction
	// 	0:		1/4 (only 1/8 because it's split in the middle)
	// 	1:		1/4
	// 	2:		1/8

	// IOW, we need a function for which
	// 	round(f(x)*2)/2 = 0   for -0.25 <= x < 0.25
	// 	round(f(x)*2)/2 = 0.5 for  0.25 <= x < 0.75
	// 	round(f(x)*2)/2 = 1   for  0.75 <= x < 1
	// so that we can generalize.

	if (fabs(centered) < 0.250) {
		accuracy = copysign(0, centered);
		//cout << "Accuracy 0" << endl;
		return(true);
	}

	if(fabs(centered) < 0.750) {
		accuracy = copysign(1, centered);
		//cout << "Accuracy " << accuracy << endl;
		return(true);
	}

	accuracy = copysign(2, centered);
	//cout << "Accuracy " << accuracy << endl;*/
	return(true);
}

int Scanner::get_accuracy() { 
	return(round(accuracy)); 
}

#endif
