
#include "tools.cc"

#ifndef _KROB_COORD
#define _KROB_COORD

// Your average 2D Cartesian coordinate.

class coordinate {
	public:
		coordinate() {x = 0; y = 0; }
		coordinate(double xin, double yin) {x = xin; y = yin; }
		double x;
		double y;

		// Euclidean distance.
		double distance(const coordinate & other) const;
		double sq_distance(const coordinate & other) const;

		// clamp_min and clamp_max clamps values less than the
		// coordinate values on other, or greater than, depending
		// on _min or _max.
		void clamp_min(const coordinate & other);
		void clamp_max(const coordinate & other);

		bool cisnan() { return (isnan(x) || isnan(y)); }

		// Operator!
		coordinate operator- (const coordinate p) const;
		coordinate operator+ (const coordinate p) const;
		coordinate operator/ (const coordinate p) const;
		coordinate operator* (const coordinate p) const;

		bool operator== (const coordinate p) const;
		bool operator!= (const coordinate p) const;
};

double coordinate::distance(const coordinate & other) const {

	return(euc_distance(x, y, other.x, other.y));
}

double coordinate::sq_distance(const coordinate & other) const {
	return(euc_sq_distance(x, y, other.x, other.y));
}

void coordinate::clamp_min(const coordinate & other) {
	// These are equivalent to if (x < other.x) x = other.x (and similar
	// for .y).
	x = max(x, other.x);
	y = max(y, other.y);
}

void coordinate::clamp_max(const coordinate & other) {
	x = min(x, other.x);
	y = min(y, other.y);
}

coordinate coordinate::operator- (const coordinate p) const {
	return(coordinate(x - p.x, y - p.y));
}
coordinate coordinate::operator+ (const coordinate p) const {
	return(coordinate(x + p.x, y + p.y));
}
coordinate coordinate::operator/ (const coordinate p) const {
	return (coordinate(x/p.x, y/p.y));
}
coordinate coordinate::operator* (const coordinate p) const {
	return(coordinate(x * p.x, y * p.y));
}

bool coordinate::operator== (const coordinate p) const {
	return(x == p.x && y == p.y);
}

bool coordinate::operator!= (const coordinate p) const {
	return(x != p.x || y != p.y);
}

#endif
