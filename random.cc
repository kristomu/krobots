#ifndef _KROB_RAND
#define _KROB_RAND

#include <stdint.h>

// Portable RNG, used for replaying matches with given Match IDs on different
// compiler architectures. This may be overengineering, but it's a good thing
// to have, given that we're going to implement Match IDs (reproducible initial
// seeds) for debugging purposes anyway.

// Implements a 2^160 xor shift generator (Marsaglia, comp.lang.c, May 13 2003)

// If we were to be really paranoid, this would be replaced with a cryptographic
// secure RNG so that one robot can't sniff the state of another robot, but I
// find this attack pretty unlikely.

// Type is here because the class isn't static, and so we don't want to have
// identical random numbers for different robots.

typedef enum rnd_minors {RND_INIT = 0, RND_ROBOT = 1, RND_SONAR = 2 };
typedef uint32_t matchid_t;

class single_rand {
	private:
		uint32_t x, y, z, w, v;

	public:
		single_rand(matchid_t seed, uint32_t type_major,
				rnd_minors type_minor);
		uint32_t irand();
		double drand();
};

single_rand::single_rand(matchid_t seed, uint32_t type_major,
		rnd_minors type_minor) {
	x = seed; y = type_major; z = (uint32_t)type_minor;
	w = 88675123; v = 886756453;

	// Mix it good.
	for (int counter = 0; counter < 10; ++counter)
		irand();
}

uint32_t single_rand::irand() {
	unsigned long t;
	
	t=(x^(x>>7)); 
	x=y; 
	y=z; 
	z=w; 
	w=v;
	v=(v^(v<<6))^(t^(t<<13)); 
	return (y+y+1)*v;
}

double single_rand::drand() {
	double maximum = (double)((uint64_t)-1);
	// Clock two then group since double is more finegrained than 2^-32.
	uint64_t dx = ((uint64_t)irand()) << 32 + irand();
	return(dx/maximum);
}

/*
#include <iostream>

main() {

	single_rand test(1234, 1);

	for (int counter = 0; counter < 100000; ++counter) {
		std::cout << test.drand() << endl;
	}
}
*/

#endif
