#ifndef _KROB_TICKT
#define _KROB_TICKT

#include "tools.cc"

// Simple timer class for global timing, used in this case to make the keyboard
// polling system run at a fixed interval (even sans multithreading) so that it
// doesn't use up too many resources.
// The timer works in the way that there's a certain event that should happen
// at a given interval, and a (possibly fluctuating) "clock" that designates
// that time has passed. The timer predicts the number of ticks required until
// the time required has passed, updating the prediction each time the event
// occurs.
// The point is that checking whether something is ready is just a simple count
// and doesn't require checks against the true clock.

#include <iostream>

class ticktimer {
	private:
		bool initial;
		double interval;
		double counts_per_sec, last_abs_time;
		int count;
		int counts_required;

		void refresh();

	public:
		void set_timer(double interval_in, int initial_count_guess);
		ticktimer(double interval_in, int initial_count_guess);

		void increment_count() { ++count; }
		bool ready() const { return (count >= counts_required); }
		void reset() { if (ready()) refresh(); }
		double get_cps() const { return(counts_per_sec); }
};

void ticktimer::refresh() {
	double cur = get_abs_time();
	double time_elapsed = cur - last_abs_time;
	last_abs_time = cur;
	// Exponential average.
	if (initial) {
		counts_per_sec = count / time_elapsed;
		initial = false;
	} else	
		counts_per_sec = 0.25 * counts_per_sec + (count / time_elapsed)
			* 0.75;
	count = 0;
	counts_required = round(counts_per_sec * interval);
}

void ticktimer::set_timer(double interval_in, int initial_count_guess) {
	interval = interval_in;
	counts_required = initial_count_guess;
	last_abs_time = get_abs_time();
	count = 0;
}

ticktimer::ticktimer(double interval_in, int initial_count_guess) {
	initial = true;
	set_timer(interval_in, initial_count_guess);
}

#endif //_KROB_TICKT
