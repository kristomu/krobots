// Coordinate tools for finding closest points and distances between lines
// and points.

#ifndef _KROB_COORDTOOL
#define _KROB_COORDTOOL

#include "coordinate.cc"
#include <math.h>
#include <iostream>

using namespace std;

class coord_tool {

	private:
		void check_line_intersection(coordinate l1_begin, coordinate
				l1_end, coordinate l2_begin, coordinate l2_end,
				double & u, double & up) const;

	public:
		// Closest point between a line and a point off the line. If
		// segment is true, returns the closest point on the segment.
		// The squared_distance parameter is used for caching results
		// inside scan() - since line_end is reconstructed using trig
		// operators, we already know the squared distance from the
		// start of the line to its end.
		coordinate closest_line_point(coordinate line_begin, coordinate
				line_end, coordinate point, bool segment,
				double squared_distance) const;
		coordinate closest_line_point(coordinate line_begin, coordinate
				line_end, coordinate point, bool segment) const;

		// Check if two line segments intersect.
		bool segments_intersect(coordinate l1_begin, coordinate l1_end,
				coordinate l2_begin, coordinate l2_end) const;

		// Distance between two lines at the closest points. If 
		// midpoint_intersection is false, returns NaN unless both of
		// the closest points are at the end of the line segments.
		// (Used for collision detection.)
		// If squared is true, the function returns squared Euclidean
		// distance, otherwise, ordinary Euclidean distance.
		double dist_line_line(const coordinate & l1_begin, const 
				coordinate & l1_end, const coordinate & 
				l2_begin, const coordinate & l2_end,
				bool midpoint_intersection, bool squared) const;

	/*	bool lines_cross(const coordinate & l1_begin, const
				coordinate & l1_end, const coordinate & 
				l2_begin,const coordinate & l2_end) const;*/

		// These are stolen from VGL.

		// Distance between a line and closest point - to pare the
		// number of calculations required. If squared is true,
		// then squared distance, otherwise Euclidean.
		bool sq_dist_closest_point(const coordinate & l1_begin, 
				const coordinate & l1_end, 
				const coordinate & p, double & upper,
				double & lower) const;
		double dist_closest_point(const coordinate & l1_begin,
				const coordinate & l1_end,
				const coordinate & p, bool squared) const;


		double dist2_line_line(const coordinate & l1_begin,
				const coordinate & l1_end,
				const coordinate & l2_begin,
				const coordinate & l2_end, bool squared) const;

		coordinate closest_line_point_to_line(
				const coordinate & l1_begin,
				const coordinate & l1_end, 
				const coordinate & l2_begin,
				const coordinate & l2_end) const;
};

// Calculate the normalized intersection values for a line-line intersection,
// or NaN if the lines are singular or parallel.
void coord_tool::check_line_intersection(coordinate l1_begin, 
		coordinate l1_end, coordinate l2_begin, coordinate l2_end, 
		double & u, double & up) const {

	// Point degeneracy
	if (l1_begin == l1_end && l2_begin == l2_end) {
		u = INFINITY;
		up = INFINITY;
		return;
	}

	double denom = ((l2_end.y - l2_begin.y) * (l1_end.x - l1_begin.x)) -
		(l2_end.x - l2_begin.x) * (l1_end.y - l1_begin.y);

	// Parallel or line-point.
	if (denom == 0) {
		u = NAN;
		up = NAN;
		return;
	}

	double num_a = ((l2_end.x - l2_begin.x) * (l1_begin.y - l2_begin.y)) -
		((l2_end.y - l2_begin.y) * (l1_begin.x - l2_begin.x));

	double num_b = ((l1_end.x - l1_begin.x) * (l1_begin.y - l2_begin.y)) -
		((l1_end.y - l1_begin.y) * (l1_begin.x - l2_begin.x));

	u = num_a / denom;
	up = num_b / denom;

}


coordinate coord_tool::closest_line_point(coordinate line_begin, coordinate
		line_end, coordinate point, bool segment, 
		double squared_distance) const {

	double u_upper = (point.x - line_begin.x) * (line_end.x - line_begin.x)
		+ (point.y - line_begin.y) * (line_end.y - line_begin.y);
	double u_lower = squared_distance;

	if (u_lower == 0) // Point-point check?
		return(line_begin); // Then there's only one closest point.

	double u = u_upper / u_lower;

	if (segment && (u < 0 || u > 1)) {
		// Closest point is not on the segment.
		if (u < 0) 
			return(line_begin);
		else	return(line_end);
	}

	coordinate closest(line_begin.x + u * (line_end.x - line_begin.x),
			line_begin.y + u * (line_end.y - line_begin.y));

	return(closest);
}

coordinate coord_tool::closest_line_point(coordinate line_begin, coordinate
		line_end, coordinate point, bool segment) const {

	return(closest_line_point(line_begin, line_end, point, segment,
				line_begin.sq_distance(line_end)));
}

bool coord_tool::segments_intersect(coordinate l1_begin, coordinate l1_end,
		coordinate l2_begin, coordinate l2_end) const {

	double u, up;

	check_line_intersection(l1_begin, l1_end, l2_begin, l2_end, u, up);

	// Parallel or singular? Beat it.
	if (!finite(u) || !finite(up)) return(false);

	// If the normalized intersection points are between 0 and 1 for both
	// lines, then there's an intersection within the interval in question.
	return (u >= 0 && u <= 1 && up >= 0 && up <= 1);
}


// This returns NaN if the closest point isn't on the endpoint (and midpoint_
// intersection is false), infinity for single points, and -infinity for 
// parallel lines.

// BLUESKY: Differ between parallel lines we never meet and parallel lines
// that are all inside the range.

// Also, dist_line_line may not be the right tool. Consider the case of two
// intersecting lines, so that at one point (when rolled far enough back),
// we have
//
//  O(0) O(t_0)		O(t)
//  -----x--------------->
//
//       | T(0)
//       |
//       v T(t)

// Then, for all time points after t_0, the closest line-line distance will be
// the same, yet the distance at positions O(t) and T(t) will definitely differ
// as t differs!

// What we really want to find is time t so that distance between T(t) and O(t)
// is some constant (if this is possible), and also find the first such t.

#define CT_DLL_IS_PARALLEL -INFINITY
#define CT_DLL_ONLY_POINTS INFINITY
#define CT_DLL_MIDSECTION  NAN


// Fix this routine, it's buggy (making shots constantly miss at 60 cps).
double coord_tool::dist_line_line(const coordinate & l1_begin, 
		const coordinate & l1_end, const coordinate & l2_begin, 
		const coordinate & l2_end, bool 
		midpoint_intersection, bool squared) const {

	// If there's an intersection between the two lines, then the distance
	// of the closest points is zero (obviously).
	// Thus, calculate the intersection points as in the line-point case,
	// and if both's u equivalents are 0 < u < 1, then there's an 
	// intersection.
	
	double u, up;
	
	// Check for line-point degeneracy. If we have a line versus a point,
	// change the point into a very short line segment. This is ugly; we
	// should really just have a point-line distance check. Refactor
	// later.

	if (l1_begin == l1_end || l2_begin == l2_end) {
		coordinate closest;
		double mini;
		if (l2_begin == l2_end) {
			if (l1_begin == l1_end)
				closest = l1_begin;
			else	return(dist_closest_point(l1_begin, l1_end,
						l2_begin, squared));
			/*else	closest = closest_line_point(l1_begin, l1_end,
					l2_begin, true);*/
			mini = closest.sq_distance(l2_begin);
		} else {
			return(dist_closest_point(l2_begin, l2_end, l1_begin,
						squared));
			/*closest = closest_line_point(l2_begin, l2_end,
					l1_begin, true);
			mini = closest.sq_distance(l1_begin);*/
		}
		if (!squared) return(sqrt(mini));
		return(mini);
	}

	check_line_intersection(l1_begin, l1_end, l2_begin, l2_end, u, up);

	if (isnan(u) || isnan(up)) return(CT_DLL_IS_PARALLEL);
	// Shouldn't this be &&? Or handle line-point intersection somehow.
	//if (!finite(u) || !finite(up)) return(CT_DLL_ONLY_POINTS);

	// This should work now.
	// Intersection is within (0,1) on both, thus the lines intersect,
	// thus the closest points are on the line.
	if (u >= 0 && u <= 1 && up >= 0 && up <= 1) {
		/*cout << "L1: (" << l1_begin.x << ", " << l1_begin.y << ") - ("
			<< l1_end.x << ", " << l1_end.y << ")" << endl;
		cout << "L2: (" << l2_begin.x << ", " << l2_begin.y << ") - ("
			<< l2_end.x << ", " << l2_end.y << ")" << endl;
		cout << "Return early break, u = " << u << ", up = " << up << endl;*/
		return(0);
	}

	// Hacky brute-force solution. We now know that the lines don't cross.
	// Assuming they aren't parallel, then the closest point is one of the
	// extrema points of one line wrt the entirety of the other line.
	// This makes for four different options; check them all.

	double record = closest_line_point(l1_begin, l1_end, l2_begin, true). 
			sq_distance(l2_begin);
	double cand;
	cand = closest_line_point(l1_begin, l1_end, l2_end, true).
		sq_distance(l2_end);
	if (cand < record) record = cand;
	cand = closest_line_point(l2_begin, l2_end, l1_begin, true).
		sq_distance(l1_begin);
	if (cand < record) record = cand;
	cand = closest_line_point(l2_begin, l2_end, l1_end, true).
		sq_distance(l1_end);
	if (cand < record) record = cand;

	if (squared)	return(record);
	else 		return(sqrt(record));
}

// Returns the closest point on the l1 line segment with regards to the l2 
// line segment. This is used for our missile hack, for covering up that secant
// some times fail to find the root (or finds the exit root instead of the entry
// root). We assume that a previous method has already established that the
// lines get close enough and aren't parallel.
coordinate coord_tool::closest_line_point_to_line(const coordinate & l1_begin,
		const coordinate & l1_end, const coordinate & l2_begin,
		const coordinate & l2_end) const {

	// First check the point-point and line-line cases. Since we're
	// returning from l1, this is somewhat simpler.
	if (l1_begin == l1_end) return(l1_begin);
	if (l2_begin == l2_end) 
		return(closest_line_point(l1_begin, l1_end, l2_begin, true));

	// Okay, brute force. First check if the lines cross. If so, return
	// the crossing point.
	double u, up;
	check_line_intersection(l1_begin, l1_end, l2_begin, l2_end, u, up);

	// But hey, we only need to check that we're not encountering a point-
	// point or point-line condition, since if the lines are parallel,
	// it still works. Consider the case of
	// 	A---------B
	// 	    C-----------D
	// Then the closest point on (A,B) wrt (C,D) is the same as one of A,B,
	// C,D with regard to the opposite line, which is what we'll check
	// anyway!
	assert(!isinf(u) && !isinf(up));

	if (finite(u) && finite(up) && u >= 0 && u <= 1 && up >= 0 && up <= 1)
		// Crosses at l1_begin + u * (l1_end - l1_begin).
		// Is this right?
		return (coordinate(l1_begin.x + u * (l1_end.x - l1_begin.x),
					l1_begin.y + u * (l1_end.y - l1_begin.y)
				  ));

	// No? Then try all possible combinations and return whatever is
	// closest. The combinations are l1 begin and end versus the l2
	// line, and vice versa.
	
	coordinate cand, cand_rec;
	double record, cand_dist;

	cand = closest_line_point(l1_begin, l1_end, l2_begin, true);
	cand_rec = cand;
	record = cand.sq_distance(l2_begin);

	cand = closest_line_point(l1_begin, l1_end, l2_end, true);
	cand_dist = cand.sq_distance(l2_end);
	if (cand_dist < record) {
		cand_rec = cand;
		record = cand_dist;
	}

	cand = closest_line_point(l2_begin, l2_end, l1_begin, true);
	cand_dist = cand.sq_distance(l1_begin);
	if (cand_dist < record) {
		cand_rec = cand;
		record = cand_dist;
	}

	cand = closest_line_point(l2_begin, l2_end, l1_end, true);
	cand_dist = cand.sq_distance(l1_end);
	if (cand_dist < record) {
		cand_rec = cand;
		record = cand_dist;
	}

	return(cand_rec);
}


// Start stealing here.

bool coord_tool::sq_dist_closest_point(const coordinate & l1_begin,
		const coordinate & l1_end, const coordinate & p, double & upper,
		double & lower) const {

	// Squared distance between endpoints.
	double ddh = l1_begin.sq_distance(l1_end);

	// Squared distance to endpoints
	double dd1 = l1_begin.sq_distance(p), dd2 = l1_end.sq_distance(p);

	// If closest to the start point or end point, give that result.
	// (Triangle inequality?)
	if (dd2 >= ddh + dd1) {
		lower = dd1;
		return(false);
	}
	if (dd1 >= ddh + dd2) {
		lower = dd2;
		return(false);
	}

	// Otherwise, squared perpendiclar distance to line.
	// (Ax + By + C = 0)
	double A = l1_begin.y - l1_end.y;
	double B = l1_end.x - l1_begin.x;
	double C = l1_begin.x * l1_end.y - l1_end.x * l1_begin.y;

	upper = A * p.x + B * p.y + C;
	lower = A * A + B * B;

	return(true);
}

double coord_tool::dist_closest_point(const coordinate & l1_begin, 
		const coordinate & l1_end, const coordinate & p, 
		bool squared) const {

	double upper, lower;

	if (!sq_dist_closest_point(l1_begin, l1_end, p, upper, lower)) {
		// One of the endpoints, and the squared distance is in lower.
		if (squared) return(lower);
		return(sqrt(lower));
	}

	// Not an endpoint.

	if (squared)
		return((upper*upper) / lower);
	else	return(fabs(upper) / sqrt(lower));

	 // squared distance between endpoints :
	 // 00044   T ddh = square(x2-x1) + square(y2-y1);
	 // 00045 
	 // 00046   // squared distance to endpoints :
	 // 00047   T dd1 = square(x-x1) + square(y-y1);
	 // 00048   T dd2 = square(x-x2) + square(y-y2);
	 // 00049 
	 // 00050   // if closest to the start point :
	 // 00051   if (dd2 >= ddh + dd1)
	 // 00052     return dd1;
	 // 00053 
	 // 00054   // if closest to the end point :
	 // 00055   if (dd1 >= ddh + dd2)
	 // 00056     return dd2;
	 // 00057 
	 // 00058   // squared perpendicular distance to line :
	 // 00059   T a = y1-y2;
	 // 00060   T b = x2-x1;
	 // 00061   T c = x1*y2-x2*y1;
	 // 00062   return square(a*x + b*y + c)/double(a*a + b*b);
}

/*                double dist2_line_line(const coordinate & l1_begin,
				                                const coordinate & l1_end,
								                                const coordinate & l2_begin,
												                                const coordinate & l2_end, bool squared) const;
*/
#endif
