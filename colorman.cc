// This class returns robot colors based on a binary scheme. This binary scheme
// uses subdivision, so that cases with 1, 2, 4, 8, robots etc. have as great
// a hue difference as possible, while retaining the colors of earlier robots
// (so that when running a tournament of 3 robots, the colors of #1 and #2 will
// be the same as when running one of 2 robots). Other ideas here, not yet
// implemented, include making a color based on a hash, on a #color 
// statement in the robot program, or on UID or transponder ID.

// DONE: Add shield color derivation. (Missiles, turrets, and mines have fixed
// colors)

#ifndef _KROB_COLORMAN
#define _KROB_COLORMAN

#include "color.cc"
#include <iostream>

using namespace std;

class color_manager {

	//private:
	public:
		double get_raw_hue(int number);

	//public:
		Color robot_color(int number, double saturation, double value);
		Color shield_color(int number);
};

double color_manager::get_raw_hue(int number) {

	// The first robot is green (hue p). The second is red.
	// The relative hues go like this:
	// 	(1) (0.5) (0.25) (0.75) (0.125) (0.375) (0.625) (0.875) etc
	
	// This is, first 1, then 1/2, then 1/4 and 3/4, and in general
	// all odd numbers between 1 and 2^p, divided by 2^p.
	// To keep the value from repeating, the GCD of the values and any
	// of the previous iterations must be 1. Since two does not divide
	// odd numbers, and 2^p consist only of the factor 2, subsequent
	// pass values can't be reduced to previous passes.
	
	// in	adj in	out
	// N/A	0	1
	// 	1	1/2
	// 	2	1/4
	// 	3	3/4
	// 	4	1/8
	
	// Then we have, for input p, the output is x/y, where
	// 	y is the next higher power of two to p
	// 	x is 2(p - (y/2)) + 1.
	
	int y = 1 << (int)ceil(log(number+1)/log(2));
	int x = 2 * (number - (y >> 1)) + 1;

	return(x/(double)y);
}

Color color_manager::robot_color(int number, double saturation, double value) {
	// Red is HSV 0 (belongs to robot #2)
	// Green is HSV 0.33^.
	// This doesn't bode well unless we go to a 3-base fraction system,
	// which we may do later.

	Color x(0, 0, 0);
	double adj_hue = 1/3.0 + (1 - get_raw_hue(number));
	if (adj_hue > 1) --adj_hue;
	if (adj_hue < 0) ++adj_hue;

	x.set_hsv(adj_hue, saturation, value);
	return(x);
}

Color color_manager::shield_color(int number) {
	return(robot_color(number, 1, 0.60));
}

#endif
