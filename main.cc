// DONE: Add stats printing as in nosdl. DONE. Refactor along. Fix the SDL bug.
// DONE: Fix cosmetic error when drawing borders for statlets (don't copy the
// top line). Add #TIME. DONE!
// DONE: Make it count errors. DONE.
// DONE: Make it pass the upper CPU limit into run_round so that we can do
// 	fastcpu easily. DONE. Also fix the keyboard polling for console. DONE.
// DONE: Add matchID parameter (Z?) and additional parameter to display r4-style
// 	stats for each round, used to find troublesome rounds. DONE
// DONE: Fix the error reporting problem and add the maxlines constraint. We'll
// 	probably need a constraint class, then... that error_container takes
// 	as a parameter when constructing its human-readable error message.
// 	Picked another way of doing this; first one DONE, now maxlines.. DONE!
//
// DONE: Implement + and -. Then test against the tst* robots.., check todos
// 	elsewhere, and afterwards, we're done! Kinda done, but I think + and
// 	- are reversed: look them up in the ATR2 docs to see if I'm right.
// 			+ is slower, - is faster. DONE (except checking todos
// 			elsewhere)

// DONE: Test against unit tests, ATR2 standard bots, and robots/* for
// 	discrepancies. Find out why it's so slow at compiling (probably that
// 	lookup unit getting reinited again and again..) DONE and N/A
// DONE: Give it a title. Do test against unit tests. Run a lot to find that
// 	finite(u) && finite(up) assertion break, and fix that.
// 	Test shields and heat evolution (doesn't work very well right now,
// 	 though damage transmission seems to), and fix the segfault on incorrect
// 	 filenames. Also, fix the statlet problem (offset > robots.size() -
// 	 statlets.size() without having to know statlets.size(). Maybe 
// 	 	renderer.get_last_visible_robot?). Bluesky, add small statlets
// 	 and have it handle that seamlessly.
// 	 		Scrolling no longer works at all, fix that. Otherwise,
// 	 		and except the BLUESKY, we're all done here. DONE.

// 	Oh, and is there a problem with the colors? They don't seem to be
// 	 subdividing correctly. Also, it might be better to have a radix 3
// 	 subdivision - fits more readily into (red, green, blue).
// 	 Bluesky that, but should be possible to find a closed form. For the
// 	 real bonus points, use LCbCr instead of HSV.

// DONE: Test the get_num_statlets thing we fixed now (that there aren't any
//	 duplicates when we scroll down). DONE
// DONE: Fix the display thing where it doesn't show robots completely on the
// 		edge. There are two solutions: fix edge_collide so that the
// 		collision radius is != 0, or lower the size of the robot
// 		widget so that they won't get clipped.
// 			Is that a real error? Make some robots that always 
// 			crash with the edge, then check.
// 			Done, I can't reproduce the error.
// 		Also make a parameter to show progress (cout with \r) and
// 		one to enable line-buffering (don't init consolekbd).
// 			Not sure if I should make a progress one, but I
// 			really should make a line-buffering one.
// 			DONE line-buffering. Not sure if I should do progress;
// 				in that case, it should be on -v, with -vv
// 				for showing per-cycle stats.
//
// 		Also name the robot when he's showing errors and the error
// 		is dumped to console (e.g instead of saying "Error at line 27"
// 		say "Error at blah.at2:27" or somesuch. DONE, although it
// 		doesn't add the extension.
// 			Check various robots for errors and whether these
// 			errors also appear in ATR2 native.
//
// 		And also fix whatever it is that hides one of the robots at the
// 		end of a porttst regression test. The mine is visible yet the
// 		robot is not, somehow.
// 			FIXED. Break breaks the loop, not just the if.
//
// BLUESKY : If the user wants a progress report, include ETA in it. This
// 		overloads kbd_timer since it's no longer a kbd_timer, it's a
// 		"anything periodic requiring user input or output" timer.
// 			Could use the window title hack for this, but it's
// 			getting pretty kludgy..
// DONE: Allow things like ktr sduck sduck to resolve to sduck.at2 sduck.at2.
// 		This probably would take the form of does_file_exist, where
// 		each filename is run through it, and then with at2 appended.
// 		A version supporting locked robots would also check against
// 		atl, unless that'd make redundancy (where both q.at2 and q.atl
// 		exists and he types ktr q; it would then say "don't know q" or
// 		"don't know what file you mean").
// DONE: Fix segmentation fault with extreme number of robots (preotr/*)
// 	Actually, there's a c* robot that does it: comms crash on insert with
// 	channel < 0.

// This is the last resort truetype path. If we don't find it in any other
// location, try it here, and if that too fails, abort with an error.

#ifndef LRTTF_LOC
#define LRTTF_LOC "/usr/share/fonts/freetype/"
#endif

// There's a memory leak here somewhere. Fix later. (FIXED)
// The big culprit, as far as graphics sloth is concerned, is blit. Our
// multiple-display strategy didn't really work; ultimately, we may need to
// rework widgets.

#include "handler.cc"
#include "consolekbd.cc"
#include "color.cc"
#include "colorman.cc"
#include "display.cc"
#include "font.cc"
#include "widgets.cc"
#include "coordinate.cc"
#include "coord_tools.cc"
#include "stored_cores.cc"
#include "global_stats.cc"
#include "presenter.cc"
#include "arena_disp.cc"
#include "mover.cc"
#include "object.cc"
#include "robot.cc"
#include "collision.cc"
#include "ticktimer.cc"
#include "scanner.cc"
#include "missile.cc"
#include "blast.cc"
#include "mine.cc"
#include "configorder.h"
#include "game_balance.cc"
#include "cpu/corelogic.cc"
#include "cpu/compiler.cc"

#include <iostream>
#include <fstream>
#include <vector>

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <getopt.h>

using namespace std;

// We should have a Display class that takes an Arena class and some aux info.
// Not yet, though, so we emulate the vector<stats> and all that stuff
// internally.

// Can we use AABBs for scanning? Probably not, but we can approximate it as
// a triangle. That's still n^2 though. We need a sophisticated space 
// partitioning structure to beat n^2; it's dubious how much we'll gain since
// most fights are one-on-one anyway, but it'd also be very hard to restructure
// once the partitioning structure is in place.

// ------- These functions advance the game ------

// Advance the movement of robots, missiles, and mines, handling explosions and
// crashes. This basically invokes the correct collision detection routines.
void advance_movement(vector<robot> & robots, list<robot *> & live_robots,
		list<missile> & missiles,
		list<mine> & mines, blasts & explosions, double cycles_elapsed,
		double absolute_time_at_start, int robot_radius, 
		int crash_range, int missile_hit_range, const coordinate &
		arena_size) {

	/*vector<robot>::iterator rp;

	for (rp = robots.begin(); rp != robots.end(); ++rp) {
		rp->set_time_units(cycles_elapsed);
		rp->set_crash(false);
	}*/

	list<robot *>::iterator rp;
	for (rp = live_robots.begin(); rp != live_robots.end(); ++rp) {
		(*rp)->set_time_units(cycles_elapsed);
		(*rp)->set_crash(false);
	}

	collider test_collision;
	// First find out if any robots will crash into each other.
	test_collision.set_track_limits(live_robots, robot_radius, 
			crash_range, arena_size);

	// Then deal damage from missiles that hit.
	test_collision.handle_missile_crashes(robots, live_robots, 
			cycles_elapsed, missiles, missile_hit_range, 
			arena_size, absolute_time_at_start, explosions);

	// Also deal damage from mines.
	test_collision.handle_mine_crashes(robots, live_robots, cycles_elapsed,
			mines, explosions);

	for (rp = live_robots.begin(); rp != live_robots.end(); ++rp) {
		robot * cur_live = *rp; // Dereference so it isn't so ugly.

		// First advance
		cur_live->move(cur_live->time_units());

		// Then register the crash if there was one.
		// (Here we don't need to specify the time, but we do so.)
		if (cur_live->crashed())
			cur_live->register_crash(cur_live->time_units());
	}

	// Finally, advance missiles. (There's no need to "advance" mines).
	for (list<missile>::iterator pos = missiles.begin();
			pos != missiles.end(); ++pos)
		pos->move(cycles_elapsed);
}

void advance_CPUs(core_storage & robot_cores, vector<robot> & robots,
		list<Unit *> & live_robots, list<missile> & missiles,
		list<mine> & mines, vector<set<robot *> > & comms_lookup,
		const cmd_parse & aux_disassembler, matchid_t match_id,
		const coordinate arena_size, const bool verbose, 
		const bool report_errors) {

	vector<corelogic>::iterator core_iter, core_begin = robot_cores.
		cores.begin();

	core_iter = core_begin;

	for (vector<robot>::iterator rp = robots.begin(); rp != robots.end();
			++rp) {
		run_error cpu_error(ERR_NOERR);

		int cycles_permitted = rp->withdraw_CPU_cycles();

		// Execute until we're done
		while (cycles_permitted > 0 && !rp->dead()) {
			if (!core_iter->execute_multiple(cycles_permitted, *rp,
					live_robots, missiles, mines,
					comms_lookup, 1, 1, arena_size,
					false, cpu_error, cycles_permitted)) {
				// We go here if there's an error.
				// "Proper" ATR2 does a pause here, probably
				// part of the same logic that breaks on debug.

				// Let it be known what's acting up.
				// Doesn't work if the error was at the last
				// line, in which case get_IP()-1 will be -1
				// (wraparound).
				rp->update_error(cpu_error);

				if (verbose || report_errors) {
					int idx = rp - robots.begin();
					cout << "Error " << (int)cpu_error 
						<< " in robot " << idx 
						<< "(" << rp->get_local_stats().
						robot_name << ") (matchID " << match_id << "), at ";

					int eff_line = core_iter->get_old_IP();
					int source_line = robot_cores.
						lookup_line_number(idx, 
								eff_line);

					// If we can look up the real line,
					// print it. Otherwise print the
					// "effective" line (IP).

					if (source_line != -1)
						cout << "line " << source_line;
					else	cout << "effective line " <<
						eff_line;

					cout << ", disasm: " 
						<< aux_disassembler.disassemble
						(core_iter->
						 get_instr_at_oldIP()) << endl;
				}
			}
		}

		++core_iter;
	}
}

// --- These generate robot bodies, and give them a location in the arena ---

// Note that device_weighting must not be & and not be const; otherwise,
// completion will cause side effects. It really shouldn't be necessary to
// do completion (since we did it in core_store), but we still do it to be
// safe.
// ?? Do we really need "kills" and "deaths", now that we have local and global
// score stores? Now removed.

// Should this be just remove_extension, both, or neither?
string snip_extraneous(string filename) {
	return(remove_path(remove_extension(filename)));
}

// Instantiate a robot.
// Insanity is -2 for no "insane missiles" setting, otherwise the insanity
// amount. (Insanity level -2 makes missiles stand almost still. Lower 
// levels would make them go backwards, except that mover doesn't support it.
// Implement if amused.)

#define INSANITY_SANE -10

robot generate_robot(matchid_t matchid, const coordinate arena_size,
		single_rand & initstate_randomizer, string fn, 
		int robot_radius, int & count, string message, 
		vector<int> device_weighting, int maxweight,
		bool old_shields, int CPU_cycles_per_cycle,
		global_round_info & this_robot_global_stats,
		int insanity) {

	double start_heading = initstate_randomizer.irand() % 256;
	double start_throttle = 0;

	game_balance limits;

	// Default scan range.
	int scanrange = limits.get_scanner_range(5);
	int default_mines = limits.get_num_mines(0);

	color_manager colorman;

	double missile_speed;
	if (insanity != INSANITY_SANE)
		missile_speed = 100.1 + 50 * insanity;
	else	missile_speed = limits.get_missile_speed();

	robot to_ret(matchid, count++, snip_extraneous(fn), 
			limits.get_min_throttle(), limits.get_max_throttle(), 
			limits.get_speed_multiplier(), limits.get_turn_rate(),
			limits.get_accel_value(),
			start_heading, start_throttle, robot_radius, scanrange,
			16, default_mines, CPU_cycles_per_cycle, 
			limits.get_heat_shutdown(), 
			limits.get_heat_hysteresis(), limits.get_sonar_range(),
			missile_speed, old_shields, &this_robot_global_stats);

	// -1 because count starts at 1 and robot_color starts at 0. The robot
	// itself is fully saturated and bright.
	to_ret.set_colors(colorman.robot_color(count - 1, 1, 1),
			colorman.shield_color(count - 1));

	to_ret.set_message(message);
	to_ret.set_desired_throttle(0);
	to_ret.turret_heading = initstate_randomizer.irand() % 256;

	// Add message and reweight strengths according to #config.
	limits.complete_config_values(device_weighting);

	// Check that we're not cheating. While we did this once already, in
	// core_store, this is here just to be certain.
	int sum = 0;
	for (size_t counter = 0; counter < device_weighting.size(); ++counter) 
		sum += device_weighting[counter];
	assert (sum <= maxweight);

	to_ret.adjust_balance(device_weighting, limits);

	return(to_ret);
}

robot generate_robot(matchid_t matchid, const coordinate arena_size,
		single_rand & initstate_randomizer, string fn,
		int robot_radius, int & count, const core_storage & cores,
		int core_idx, int maxweight, bool old_shields, 
		int max_CPU_cycles_per_cycle, 
		global_round_info & this_robot_global_stats, int insanity) {

	return(generate_robot(matchid, arena_size, initstate_randomizer,
				fn, robot_radius, count, 
				cores.get_message(core_idx),
				cores.get_weighting(core_idx), maxweight, 
				old_shields, cores.get_CPU_speed(core_idx, 
					max_CPU_cycles_per_cycle, 
					max_CPU_cycles_per_cycle),
				this_robot_global_stats, insanity));
}

void place_robot_randomly(single_rand & initstate_randomizer, 
		const coordinate arena_size, vector<robot> & robots_so_far,
		int index, int crash_range) {

	if (index < 0 || index >= robots_so_far.size()) return;

	coordinate rndpos;
	bool colliding;
	collider edge_tester;

	do {
		// Note, this actually returns a value on [0..arena_size],
		// which is not what we want. The edge test removes the, err,
		// edge cases though, so we can do it with impunity.
		rndpos.x = initstate_randomizer.drand() * arena_size.x;
		rndpos.y = initstate_randomizer.drand() * arena_size.y;

		// Check if this new proposed location collides.
		colliding = edge_tester.inside_wall(rndpos, crash_range,
				arena_size);

		// If it collided with the edge, this never gets executed.
		// Nifty!
		for (size_t counter = 0; counter < robots_so_far.size() 
				&& !colliding; ++counter) {

			if (counter == index) continue;

			// Margin of safety.
			if (rndpos.distance(robots_so_far[counter].get_pos()) 
					< crash_range * 1.2)
				colliding = true;
		}
	} while (colliding);

	robots_so_far[index].set_pos(rndpos);
}


// Others
// Don't include dead people here.

template<typename T> void alias(vector<robot> & input,
		T & output) {

	T toRet;

	for (vector<robot>::iterator rp = input.begin(); rp != input.end();
			++rp)
		toRet.push_back(&*rp);

	output = toRet;
}

template<typename T> void realias(T & to_check) { 

	typename T::iterator curpos = to_check.begin();

	while (curpos != to_check.end())
		if ((*curpos)->dead())
			curpos = to_check.erase(curpos);
		else	++curpos;
}

// Guess filenames to find out if they're incompletely specified, so as to
// permit things like ./ktr2 sduck sduck to resolve to sduck.at2 sduck.at2.
// The function returns those filenames that were accessible.
vector<string> guess_filename(string base_filename) {
	ifstream inf(base_filename.c_str());
	if (inf) return(vector<string>(1, base_filename));

	vector<string> to_attempt, success;

	to_attempt.push_back(base_filename + ".at2");
	to_attempt.push_back(base_filename + ".AT2");
	to_attempt.push_back(base_filename + ".atl");
	to_attempt.push_back(base_filename + ".ATL");

	for (int counter = 0; counter < to_attempt.size(); ++counter) {
		ifstream test(to_attempt[counter].c_str());
		compile_error err = CER_NOERR;
		if (!test)
			err = check_filename(to_attempt[counter], CER_NOERR);

		// Add to the success list if it's anything but "file not found"
		// to avoid bizarre cases where ./ktr2 sduck says "sduck not
		// found" when sduck.at2 exists but can't be accessed.
		if (err != CER_NOFOUND)
			success.push_back(to_attempt[counter]);
	}

	return(success);
}

bool compile_robots(const vector<string> filenames, int maxdevices, 
		int maxlines, bool verbose, bool strict_compile,
		core_storage & out) {

	error_container retval(CER_NOERR);

	vector<string> prospective;

	for (size_t counter = 0; counter < filenames.size(); ++counter) {
		ifstream inf(filenames[counter].c_str());

		retval = out.insert_core(inf, filenames[counter], maxdevices,
				maxlines, verbose, strict_compile);

		// If it didn't open, try some of the completion rules. If more
		// than one file is openable there, return "ambiguous name".
		if (retval.error == CER_NOFOUND)
			prospective = guess_filename(filenames[counter]);

		// If the filename guesser didn't find anything at all, return
		// "not found", skipping this loop. Otherwise, check if
		// ambiguous. (This will cause odd behavior when someone does
		// ./ktr2 sduck and sduck.at2 exists but has incorrect 
		// permissions, while sduck.atl exists and has correct 
		// permissions, but returning "ambiguous" might be the right
		// thing to do after all.)
		if (retval.error == CER_NOFOUND && prospective.size() >= 1) {

			if (prospective.size() > 1)
				retval = error_container(filenames[counter],
						prospective.size(),
						CER_F_AMBIGUOUS);
			else {
				ifstream infx(prospective[0].c_str());
				retval = out.insert_core(infx, prospective[0],
						maxdevices, maxlines, verbose,
						strict_compile);
			}
		}

		// DONE: Do something with the constraint problem (-1 here),
		// so that it gives maxdevices if the error is that it cheats,
		// max # lines if that's the problem, etc.
		if (retval.error != CER_NOERR) {
			cerr << filenames[counter] << "\tError! " << retval.
				construct_error_message() << endl;
			return(false);
		}
	}

	return(true);
}

// Returns -1 if it's not being used by the game, otherwise char code.
int translate_SDL_keypress(int SDL_keypress) {
	switch(SDL_keypress) {
		case SDLK_ESCAPE:
		case SDLK_q:	return('Q');

		case SDLK_SPACE:
		case SDLK_BACKSPACE: return(' ');

		case SDLK_a: return('A');

		case SDLK_PLUS:
		case SDLK_KP_PLUS: return('+');

		case SDLK_MINUS:
		case SDLK_KP_MINUS: return('-');

		case SDLK_w: return('W');

		case SDLK_d: 
		case SDLK_DOWN: return('D');

		case SDLK_u:
		case SDLK_UP: return('U');

		default: return(-1);
	}
}

// DONE: Rewrite this comment block.
// Runs a single round. The parameters are:
// 	print_outcomes: If true, prints the round outcome.
// 	report_errors: If true, reports errors when they happen.
// 	verbose: If true, show per-cycle debugging info.
//	graphics: If true, access graphics structure and try to render each
//		frame.
//	show_true: If true and graphics is true, show scanning arcs when
//		rendering.
//	cycles_per_step: How many cycles to execute per CPU and collision
//		detection step. Higher is faster, but may become inaccurate
//		with weird collision errors. Only 1 has been checked.
//	framerate: Display frame rate. Lock the display to at most this many
//		frames per second.
//	per_round_tinfo: Value of per-round tournament info to show to console
//		after round is done, or -1 if we shouldn't show it.
//	core_storage: Storage structure for robot cores (programs/CPUs).
//	explosions: Structure to keep count of explosions and explosion
//		locations, for rendering.
//	filenames: Filenames of the robots.
//	comms_lookup: Communications structure, for making comms transmission
//		and reception take constant time instead of logarithmic or
//		linear time. (Maybe YAGNI?)
//	old_shields: If true, enable old shields (where robots are invulnerable
//		with shields on)
//	max_CPU_speed: Maximum CPU speed, in CPU cycles per game cycle.
//	robot_radius: Radius of robots.
//	crash_range: How close the robots can approach before they crash.
//	missile_hit_radius: Maximum distance at which a missile can explode
//		against a robot instead of passing by.
//	missile_insanity: Insanity level for "insane missiles" (really quick
//		missiles), or INSANITY_SANE for normal play.
//	arena_size: Size (in game units) of the arena.
//	maxweight: Maximum possible weight (points) allowed for the improvement
//		of robot parts.
//	maxcycles: How many cycles to run before declaring a tie.
//	min_victory_margin: ???
//	curmatch: Current match number (not ID).
//	maxmatch: Maximum match number
//	matchid: Match ID (random seed for replaying matches/examining odd
//		results more closely).
//	renderer: Pointer to the class that draws (renders) the graphics mode
//		content.
//	palette: Palette map between color names and actual colors.
//	robot_disp_radius: Display robots (in graphics mode) as if they had a
//		radius of this size. The variable is usually greater than the
//		true robot radius so that it'll be easier to see the robots in
//		graphics mode.
//	buffer_thickness: How much space to give to the buffer -- area that
//		can't be driven to, but is displayed so that large objects
//		don't get smushed up against the edge of the screen or clipped.
//	scan_lag: Duration, in msecs, to show the scanning arc after the actual
//		scan was performed by the robot. Use this so users can see when
//		scans happen without getting a horrible flickering effect.
//	disassembler: Class used for disassembling, for instance when referring
//		to errors.
//	balancer: Balance class containing information about how much
//		various hardware weights grant of benefits, maximum and minimum
//		speeds, etc.
//	stdfont: GUI rendering font.
//	SDLc: SDL interface class.
//	ckbd: Class for gathering console keypresses.
//	global_robot_stats: Statistics class to be updated. It contains the
//		score and stats of the robots thus far.
//	time_passed: Will be set to the number of cycles that did pass
//		before either a tie or victory was declared.
//	kbd_trigger: Timer to make keyboard checking trigger at regular
//		intervals and thus not slow down graphics-less execution too
//		much.

bool run_round(bool print_outcomes, bool report_errors, bool verbose, 
		bool graphics, bool show_scanarcs, double cycles_per_step, 
		int framerate, int per_round_tinfo, 
		core_storage & core_store, blasts explosions,
		const vector<string> filenames, vector<set<robot *> > & 
		comms_lookup, bool old_shields, int max_CPU_speed, 
		int robot_radius, int crash_range, int missile_hit_radius, 
		int missile_insanity, const coordinate arena_size,
		int maxweight, int maxcycles, int min_victory_margin,
		int curmatch, int maxmatch, matchid_t matchid, arena_disp *
		renderer, const map<string, Color> & palette, 
		int robot_disp_radius, int buffer_thickness, double scan_lag,
		const cmd_parse & disassembler, const game_balance & balancer,
		Font & stdfont, SDLHandler & SDLc, ConsoleKeyboard * ckbd,
		vector<global_round_info> &
		global_robot_stats, int & time_passed, 
		ticktimer & kbd_trigger) {

	size_t counter;
	single_rand robot_state_rng(matchid, 0, RND_INIT);

	// Reset the robot CPUs as we don't want anything to carry over.
	for (vector<corelogic>::iterator pos = core_store.cores.begin(); pos !=
			core_store.cores.end(); ++pos)
		pos->reset_CPU();

	// Set up the structures we're going to use.

	list<missile> missiles;
	list<mine> mines;

	vector<robot> robots;

	//vector<set<robot *> > comms_lookup(32767); // used for quick comms;
	// the set is of robots at the channel given by the vector's index.
	
	int UID_count = 1;	// Not 0, since ATR2 counts from 1

	// DONE: Is the weighting/maxweight part really needed? After all,
	// compile checks it now.. But it might be useful in the case that
	// we're going to optimize stuff, in which case we'll be altering
	// the weight profile after compilation. 
	// Refactor in any case, so that it refers to core_store instead of
	// through get_message/get_weighting.
	
	for (counter = 0; counter < filenames.size(); ++counter) {
		// Side effect: increments UID_count.
		robots.push_back(generate_robot(matchid, arena_size,
					robot_state_rng, filenames[counter], 
					robot_radius, UID_count, core_store, 
					counter, maxweight, old_shields,
					max_CPU_speed, global_robot_stats
						[counter],missile_insanity));
		// Now set a random position for this bot.
		place_robot_randomly(robot_state_rng, arena_size, robots,
				counter, crash_range);
	}

	int statlet_offset = 0;

	vector<robot>::iterator robot_pos;
	list<robot *>::iterator lrobot_pos;

	// Note that this must happen *AFTER* filling up the robots array,
	// since push_back can alter pointers, and communications_channel
	// uses pointers as reference. We should probably encapsulate this
	// properly.
	for (robot_pos = robots.begin(); robot_pos != robots.end(); 
			++robot_pos)
		// Register for comms
		robot_pos->set_communications_channel(robot_pos->
				get_communications_channel(), comms_lookup);


	// Done initing robots.
	
	// Set all as alive.
	list<robot *> live_robots;
	list<Unit *> live_units;

	alias(robots, live_robots);
	alias(robots, live_units);
	
	double current_cycle = 0;

	double timeslice = cycles_per_step;
	double cycles_per_sec = timeslice * framerate;

	// Set some cosmetic defaults.
	double sonar_lag = scan_lag * 3;   // Because sonar takes so long time
	double radar_lag = scan_lag * 2.5; // Because the effect would be missed
					   // otherwise.
	double x_separation = 0.01;

	int time_last_death = -1;

	bool finished = false, abort = false;

	Color black = palette.find("black")->second, 
	      midgrey = palette.find("midgrey")->second,
	      border_grey = palette.find("border_grey")->second,
	      lightgrey = palette.find("lightgrey")->second,
	      dkblue = palette.find("dkblue")->second,
	      cyan = palette.find("cyan")->second,
	      white(1, 1, 1),
	      yellow(0, 1, 1);

	// BLUESKY: Replace this with some nifty double precision "virtual
	// framerate", where if real fps < virtual framerate, the rest is
	// handled through frameskip.
	// Say that maxfps = 10, we want 11. Then the residue is (11 - 10)/10,
	// and that's the frameskip, i.e skip one frame every ten frames.
	double frameskip = 0;	// how much to skip when he keeps asking to 
				// speed up after we're at max framerate.
	int max_framerate = 60;

	double frameskip_counter = 0;
	double ccstep = 0;
		// Independent of granularity, although
		// it shouldn't be. The point here is that incrementing it
		// is much quicker than fussing with current_cycle (which is
		// double).

	bool only_one_at_start = (++live_robots.begin() == live_robots.end());

	string roundstats = "Match " + itos(curmatch) + "/" + itos(maxmatch) +
		" (Match ID " + lltos(matchid) + ")";

	if (&SDLc)
		SDLc.set_title("K-Robots - " + roundstats, "K-Robots");

	bool blocking = false;

	// DEBUG
	cout << roundstats << endl;

	double cps = 0;

	while (!finished) {

		// Check whether this round is over. Should perhaps be
		// done after we've run the turn, to avoid off-by-ones.
		if (current_cycle >= maxcycles) finished = true;
		// Check if there's only one, and if so, if he's kept himself
		// alive longer than the "simultaneous destruction" threshold.
		// (If there're nobody left, we're done.)
		if (live_robots.empty())
			finished = true;
		// Don't do this test if there was only one robot when we
		// called it; presumably, this is a test of that the robot
		// doesn't blow itself up, and thus it should be checked until
		// the time runs out.
		if (time_last_death == -1) {
			if (++live_robots.begin() == live_robots.end() && 
					!only_one_at_start) 
				time_last_death = current_cycle;
		} else
			if (current_cycle - time_last_death >= 
					min_victory_margin)
				finished = true;

		if (finished) continue;

		// Report debugging info if desired. Note that this is O(n)
		// for mines and missiles.

		if (verbose) {
			cout << "[Main] Live robots left: " << live_robots.
				size() << endl;
			cout << "[Main] Missiles left: " << missiles.size() <<
				endl;
			cout << "[Main] Mines left: " << mines.size() << endl;
			cout << "[Main] Cycle: " << current_cycle << endl;
		}

		// Advance explosions (display effect) and robots before we
		// render anything.

		if (graphics)
			explosions.update_all(current_cycle);

		// Advance ordnance state
		advance_movement(robots, live_robots, missiles, mines, 
				explosions, timeslice, current_cycle, 
				robot_radius, crash_range, missile_hit_radius, 
				arena_size);

		// Advance robot state and also see if someone died. If someone
		// died, then we'll have to prune the live robot list later on,
		// but there's no point in pruning it if not.
		bool someone_died = false;
		for (lrobot_pos = live_robots.begin(); lrobot_pos != 
				live_robots.end(); ++lrobot_pos)
			if (!(*lrobot_pos)->advance_internally(timeslice, 
						balancer, explosions, robots))
				someone_died = true;

		// Advance CPU
		advance_CPUs(core_store, robots, live_units, missiles, 
				mines, comms_lookup, disassembler, matchid,
				arena_size, verbose, report_errors);

		// Now that everything has been advanced by a step, set the
		// clocks to match.
		for (lrobot_pos = live_robots.begin(); lrobot_pos != 
				live_robots.end(); ++lrobot_pos) {
			// DEBUG: Check that time is correct.
			assert((*lrobot_pos)->get_time() == current_cycle);
			// Then advance the clock.
			(*lrobot_pos)->tick(timeslice);
		}

		// Remove dead robots, and advance the global clock.
		// DONE: some sort of bool that tells us someone died. Maybe
		// advance_internally returning false upon encountering
		// someone who's dead.
		if (someone_died) {
			realias(live_robots);
			realias(live_units);
		}
		current_cycle += timeslice;

		// Register the dead.
		// Not really needed, since bot stats do the stuff now.
		// Refactor!
		//register_dead(robots, time_of_death, current_cycle);

		// Finally, render.
		if (graphics && frameskip_counter > frameskip) {
			if (frameskip_counter > framerate)
				frameskip_counter = 0;

			if (!renderer->render_all(x_separation, border_grey, 
						white, black, dkblue, midgrey, 
						lightgrey, lightgrey, white, 
						yellow, cyan, matchid, 
						current_cycle, maxcycles, 
						curmatch, maxmatch, robots, 
						missiles, mines, explosions,
						robot_disp_radius, arena_size,
						buffer_thickness, 
						cycles_per_sec, scan_lag, 
						sonar_lag, radar_lag, 
						show_scanarcs, statlet_offset)){
				cerr << "Error: Can't draw display!" << endl;

				// Return empty result?
			} else 
				renderer->display();

			SDLc.wait_for_frame_refresh(); // delay to fill fps
		} 

		// Check the timer as to whether we should check for a keypress.
		// Utopia would be to use a separate thread, but I don't know 
		// multithreading and I don't know if SDL is thread-safe anyway.

		kbd_trigger.increment_count();
		if (kbd_trigger.ready()) {
			kbd_trigger.reset();

			if (graphics && &SDLc) {
				cps = cps * 0.8 + kbd_trigger.get_cps() * 0.2;

				SDLc.set_title("K-Robots - " + roundstats +
						" [" + dtos(cps, 1) + 
						" cycles/sec]",
						"K-Robots");
			}
			
			int keypress = -1;

			// This gets rid of MOUSEOVER and other irrelevant
			// events. It slows down graphics a bit, but this isn't
			// where the bottleneck lies.
			while (graphics && (SDLc.get_next_event() != -1) && 
					!blocking) {
				int px;
				switch(SDLc.get_last_event()) {
					case SDL_VIDEORESIZE:
						px = SDLc.get_event_resize().w;

						renderer->resize_displays(px,
								arena_size);
						break;
					case SDL_QUIT:
						keypress = 'Q';
						blocking = true;
						break;
					case SDL_KEYDOWN:
						keypress = 
							translate_SDL_keypress(
							 SDLc.
							 get_event_keypress());
						break;
				}
			} 
			
			// In case of console, the while above passes through
			// and we go here. Doing an if(graphics) and then
			// nesting everything another step to the right would
			// squish things too far to the right. We pay by that
			// if the user presses, say, "Q", and then A really
			// quickly, only the A is registered.
			if (!graphics && ckbd != NULL)
				keypress = ckbd->getchar();

			if (keypress != -1) keypress = toupper(keypress);

			switch (keypress) {
				case ' ':
				case '\b':
					finished = true;
					break;
				case 'Q':
				case '\e':
					finished = true;
					abort = true;
					break;
				case 'W':
					print_outcomes = !print_outcomes;
					break;
				case 'G':
					// Toggle display of graphics.
					// Impossible if we're in text mode,
					// since SDL, etc, haven't been
					// allocated.
					// Also impossible in SDL because we
					// can't hide the main window. So it
					// does nothing for now.
					break;
				case 'A':
					show_scanarcs = !show_scanarcs;
					break;
				case '+':
					// Game runs slower. But only in GFX.
					if (frameskip > 0) frameskip --;
					else if (framerate > 1)
						SDLc.set_framerate(--framerate);
					break;
				case '-':
					// Game runs faster. But only in GFX.
					if (framerate < max_framerate)
						SDLc.set_framerate(++framerate);
					else	frameskip ++; 
					break;
				case 'U':
					if (statlet_offset > 0) 
						--statlet_offset;
					break;
				case 'D':
					if (renderer->can_scroll_statlets(
								statlet_offset
								+1, robots.
								size()))
						++statlet_offset;
					break;
			}
		}

		if (graphics)
			frameskip_counter += timeslice;
		
	}

	time_passed = current_cycle;

	// Okay, the round has been played. If there's only one left standing, 
	// he's the victor, so increment his victory count. Also, increment the
	// rounds counter for all, and merge the local stats into the global 
	// stats.
	list<robot *>::const_iterator vpos = live_robots.begin();
	vpos++;
	bool only_one_survivor = (vpos == live_robots.end() &&
			!live_robots.empty());

	for (counter = 0; counter < robots.size(); ++counter) {
		robots[counter].record_end_stats(true, maxcycles);
		if (only_one_survivor && !robots[counter].dead())
			robots[counter].record_victory();

		global_robot_stats[counter].add_information(robots[counter].
				get_local_stats(), false);
	}

	// All done, output local stats.
	if (print_outcomes) {
		presenter present;
		cout << "Match " << curmatch << "/" << maxmatch << " (Match ID "
			<< matchid << ") results:";
		cout << endl << endl;
		string header = present.local_header();
		cout << header << endl;
		cout << string(header.size(), '~') << endl; 
		for (counter = 0; counter < robots.size(); ++counter) 
			cout << present.single_summary(robots[counter].
					get_local_stats(),
					*global_robot_stats[counter].get_sum(),
					counter+1) << endl;
		
		// If user requested it, also print a machine-readable
		// tournament info type output.
		if (per_round_tinfo > 0) {
			for (counter = 0; counter < robots.size(); ++counter) {
				cout << "(" << matchid << "/ext)\t";
				cout << present.get_tournament_line(
						robots[counter].
						get_local_stats(),
						per_round_tinfo, 1) << endl;
			}
		}

		cout << endl;
		// Status ("No clear victor", etc) goes here.

	}

	// If this isn't the last match, clean up the comms array and CPU
	// memory so no information leaks from one round to the next. There's
	// no reason to do this if we're in the last match, so shave off the
	// cycles there.
	if (curmatch != maxmatch && !abort) {
		// Deregister comms so the slate will be blank for the next 
		// round. Not required if we're aborting, since we're not going
		// to use the comms later anyhow.
		for (counter = 0; counter < robots.size(); ++counter)
			robots[counter].remove_communications_link(
					comms_lookup);
		// Remove memory and register contents.
		core_store.reset_cores();
	}

	// Finally, return.
	return(!abort);
}

map<string, Color> make_palette() {
	map<string, Color> toRet;

	toRet["black"] = Color(0, 0, 0);
	// Used for statlets with no information
	toRet["blanked_grey"] = Color(0.15, 0.15, 0.15);
	// For borders between fields in the GUI
	toRet["border_grey"] = Color(0.65, 0.65, 0.65);
	// The rest are mostly for robots themselves.
	toRet["midgrey"] = Color(0.5, 0.5, 0.5);
	toRet["lightgrey"] = Color(0.8, 0.8, 0.8);
	toRet["green"] = Color(0, 0.8, 0);
	toRet["white"] = Color(1, 1, 1);
	toRet["cyan"] = Color(0, 1, 1);
	toRet["dkblue"] = Color(0, 0, 0.2);

	return(toRet);
}

bool setup_graphics(Display *& display_pointer, arena_disp *& 
		arena_disp_pointer, int xsize, int ysize, Font & stdfont, 
		const coordinate arena_size, int num_robots) {

	// First set up the display. If it doesn't work, remove it and say so.
	display_pointer = new Display(true, xsize, ysize, 32);
	if (!display_pointer->ready()) {
		cerr << "Error: Cannot initialize primary display!" << endl;
		delete display_pointer;
		return(false);
	}

	// num_robots is required to decide whether to use large or small
	// statlets. We can't update it inside run_round because that'd take
	// too much time, with an alloc and dealloc for each round.
	arena_disp_pointer = new arena_disp(*display_pointer, arena_size, 
			stdfont, num_robots);

	return(true);
}

void print_usage(string program_name) {

	cout << "K-Robots, AT-Robots 2 reimplementation." << endl;
	cout << "Usage: " << program_name << " [OPTION]... [FILE]..." << endl;
	cout << "Runs an AT-Robots 2 competition among the robots whose programs reside in the " << endl << "FILEs." << endl;
	cout << endl;
	cout << "Graphical output options:" << endl;
	cout << "\t-a\t\t Show robot scanning arcs from the start (default is " 
		<< "to\n\t\t\tnot show them, which can also be toggled by "
		<< "pressing\n\t\t\t'A' when the match is running)." << endl;
	cout << "\t-d <num>\t Set framerate to <num>." << endl;
	cout << "\t-g\t\t Disable graphics altogether in favor of running in " 
		<< "\n\t\t\tconsole mode. Use this to run at maximum speed" <<
		"\n\t\t\tsuch as for tournaments." << endl;
	cout << endl;
	cout << "Console/file I/O options:" << endl;
	cout << "\t-b\t\t Disable console-mode keyboard polling, so that line-"
		<< "\n\t\t\tbuffering works. Has no effect in graphics mode." << endl;
	cout << "\t-e\t\t Report each robot error, as it happens, on the " <<
		"console.\n\t\t\tOff by default; use when debugging." <<
		endl;
	cout << "\t-r1\t\t Write simple bout info to ktr2.rep. (See the original"
		<< "\n\t\t\tATR2 document for what this means)" <<
		endl;
	cout << "\t-r2\t\t Write basic bout info to ktr2.rep. " << endl;
	cout << "\t-r3\t\t Write detailed bout info to ktr2.rep. " << endl;
	cout << "\t-r4\t\t Write very detailed bout info to ktr2.rep. " << endl;
	cout << "\t-i1\t\t Write per-round stats to console after each round"
		<< "\n\t\t\t(simple)" << endl;
	cout << "\t-i2\t\t Write per-round stats to console after each round"
		<< "\n\t\t\t(basic)" << endl;
	cout << "\t-i3\t\t Write per-round stats to console after each round"
		<< "\n\t\t\t(detailed)" << endl;
	cout << "\t-i4\t\t Write per-round stats to console after each round"
		<< "\n\t\t\t(very detailed)" << endl;
	cout << "\t-s\t\t Do not write any round- or bout outcome information "
		<< endl << "\t\t\tto the console." << endl;
	cout << "\t-v\t\t Be verbose - write information about compilation, as "
		<< endl << "\t\t\twell as round information each cycle of the"
		<< "\n\t\t\tcompetition." << endl;
	cout << "\t-w\t\t Do not write any round outcome information to " 
		<< "\n\t\t\tthe console, only the outcome of the entire bout."
		<< endl;
	cout << endl;
	cout << "Compile-time options:" << endl;
	cout << "\t-# <num>\t Limit the robots to a maximum of <num> " <<
		"effective\n\t\t\t(instruction) lines." << endl;
	cout << "\t-q\t\t Enable quirks mode. This makes the compiler ignore some"
		<< "\n\t\t\tnonfatal errors in order to behave more like"<<
		"\n\t\t\tthe original ATR2, and thus accept robots that" <<
		"\n\t\t\tATR2 accepts." << endl;
	cout << endl;
	cout << "Game execution options: " << endl;
	cout << "\t-c\t\t Don't run, just compile and exit. Use to check whether "
		<< "\n\t\t\ta robot is valid, for instance for qualifying"
		<< "\n\t\t\tto a tournament." << endl;
	cout << "\t-l <num>\t Time out a round after <num> times thousand " <<
		"\n\t\t\tcycles." << endl;
	cout << "\t-m <num>\t Run a match of <num> rounds in all. " << endl;
	cout << "\t-t <num>\t Limit CPU execution time to <num> CPU cycles " <<
		"\n\t\t\tper game cycle, maximum. #TIME limits apply if " <<
		"\n\t\t\tlower than <num>." << endl;
	cout << "\t-z <num>\t Force the first round to use the Match ID " <<
		"\n\t\t\tspecified in <num>." << endl;
	cout << "\t-@\t\t Enable old-style shields. Robots shielded with these" <<
		"\n\t\t\ttake no damage from missiles when the shield"<<
		"\n\t\t\tis up, but it is nonstandard." << endl;
	cout << "\t-% <num>\t Enable insane missiles. These missiles go at a "<<
		"base\n\t\t\tspeed of (100.1 + 50 * num) m/s, subject to "<<
		"overburn\n\t\t\tand #config boosts as usual." << endl;
}

// Half-
// DONE: Make "print user help information" procedure. Use pmars as pattern.
bool setup_user_parameters(int argc, char * * argv,
		int & framerate, int & CPU_cycles_per_cycle, int & matches,
		int & tournament_level, int & per_round_tourn_level, 
		int & maxlines, int & missile_insanity, int & maxcycle, 
		matchid_t & matchid, bool & verbose, bool & print_outcomes, 
		bool & print_final_outcome, 
		bool & graphics, bool & text_input, bool & run_battles, 
		bool & show_scanarcs, bool & report_errors, bool & old_shield,
		bool & strict_compile, bool & display_speed_info,
		vector<string> & filenames) {

	int c, index;

	// /S: Do not show source code during compile (N/A)
	// /Dn: Specify game delay (timing control) (repurp. to framerate)
	// /Tn: CPU cycles per game cycle (default 5)
	// /Ln: Battle cycle limit in thousands (default 0, no limit? Or 100k?)
	// /Q: Quiet mode, no sound fx (N/A, use for "no text" instead)
	// /Mn: Number of matches to play, default 1
	// /G: No graphics
	// /Rn: Generate a report file after battle (tournament)
	// /C: Compile-only, do not run battle
	// /A: Show scanarcs during battle (default off)
	// /W: Do not pause windowed information; turn between-round info off
	// /E: Turn on error logging for robots (output line when there's an 
	// 		error)
	// /#n: Specify maximum robot program length, e.g /#32 is 32 c. lines
	// /!n: Insane missiles (n is 0 to 15)
	// /@:  Use old shield style (no damage or heat from hits)
	// /Xn: Start in debug mode and set the step count to n (N/A)
	
	// Also:
	// 	-v: Be verbose.
	// 	-q: Compatible compilation (quirks mode). (instead of Quiet)
	// 	-s: Silent mode, report nothing.
	// 	-z <num>: Force first battle to have MatchID <num>

	// Arguments are case insensitive. Maybe make that lowercase only here.

	string ext;

	bool success = true;

	while ((c = getopt(argc, argv, "d:t:l:z:i:qsm:gr:cwvabe#:%:@")) != -1) {
		if (optarg) 
			ext = optarg;

		switch(c) {
			case 'd': // Framerate
				if (!is_integer(ext, false) || stoi(ext) <= 0) {
					cerr << "Error: Invalid framerate." 
						<< endl;
					success = false;
				} else
					framerate = stoi(ext);
				break;
			case 't': // CPU cycles per cycle
				if (!is_integer(ext, false) || stoi(ext) <= 0) {
					cerr << "Error: Invalid base CPU"
						<< " speed." << endl;
					success = false;
				} else
					CPU_cycles_per_cycle = stoi(ext);
				break;
			case 'l': // Duration of single round, in thousands
				if (!is_integer(ext, false) || stoi(ext) < 0) {
					cerr << "Error: Invalid round duration"
						<< " specified." << endl;
					success = false;
				} else
					maxcycle = stoi(ext) * 1000;
				break;
			case 'z': // Force initial Match ID
				// Because of signed and unsigned differences,
				// we now check against 0 since is_integer only
				// works wrt 31-bit ints and MatchID is 32 bits
				if (stoui(ext) == 0) {
					cerr << "Error: Invalid Match ID " <<
						"specified." << endl;
					success = false;
				} else
					matchid = stoui(ext);
				break;
			case 's': // Silent mode
				display_speed_info = false;
				print_final_outcome = false;
			case 'w':
				print_outcomes = false;
				break;
			case 'm': // number of matches
				ext = optarg;
				if (!is_integer(ext, false) || stoi(ext) <= 0) {
					cerr << "Error: Invalid number of " <<
						"matches specified." << endl;
					success = false;
				} else
					matches = stoi(ext);
				break;
			case 'g': // no graphics
				graphics = false;
				break;
			case 'r': // tournament report detail level
				if (!is_integer(ext, false) || stoi(ext) <= 0) {
					cerr << "Error: Invalid tournament data"
						<< " request." << endl;
					success = false;
				} else 
					tournament_level = stoi(ext);
				break;
			case 'i': // per-round report detail level
				if (!is_integer(ext, false) || stoi(ext) <= 0) {
					cerr << "Error: Invalid per-round data"
						<< " request." << endl;
					success = false;
				} else
					per_round_tourn_level = stoi(ext);
				break;
			case 'c': // compile-only
				run_battles = false;
				break;
			case 'a': // show scanarcs in battle (def. off, A 
				// toggles
				show_scanarcs = true;
				break;
			case 'b': // Enable line-buffering/disable ckbd
				text_input = false;
				break;
			case 'e': // report errors
				report_errors = true; // Should give IP and such
				break;
			case '#': // maximum number of lines (jmps don't count)
				if (!is_integer(ext, false) || stoi(ext) <= 0) {
					cerr << "Error: Invalid maximum number"
						<< " of lines specified." <<
						endl;
					success = false;
				} else
					maxlines = stoi(ext);
				break;
			case '%': // insane missiles
				// Insane missiles makes missiles go 100 +
				// (50 * insanity) * mult, where mult is the
				// robot's shot strength and insanity is an
				// integer specified as parameter.
				if (is_integer(ext, false) && (stoi(ext) == 
						INSANITY_SANE || stoi(ext) < 
						-2)) {
					 cerr << "Error: Invalid missile "<<
						 "insanity level." << endl;
					 success = false;
				} else {
					if (is_integer(ext, false))
						missile_insanity = stoi(ext);
					else	missile_insanity = 0;
				}
				break; 
			case 'v': // Verbose
				verbose = true;
				break;
			case 'q': // Lenient compilation
				strict_compile = false;
				break;
			case '@': // old shield style
				old_shield = true;
				break;
			case '?': // Unknown
				success = false;
				if (isprint(optopt))
					cerr << "Unknown option '-" <<
						(char)optopt << "'" << endl;
				else
					cerr << "Unknown option character '" <<
						(int)optopt << "'" << endl;
				break;
		}
	}

	filenames.resize(0);

	for (int counter = optind; counter < argc; ++counter)
		filenames.push_back(argv[counter]);

	return(success);
}

// Try to load a truetype font. Returns true if we managed to do so, otherwise
// false.
bool load_font(Font & destination, string filename) {

	string suffix = "ttf-dejavu/";

	vector<string> paths;
	paths.push_back("/usr/share/fonts/truetype/" + suffix);
	paths.push_back(LRTTF_LOC + suffix);
	paths.push_back("./" + suffix);
	paths.push_back("/usr/share/fonts/truetype/");
	paths.push_back(LRTTF_LOC);
	paths.push_back("./");

	for (int counter = 0; counter < paths.size(); ++counter) {
		cout << paths[counter] + filename << endl;
		if (destination.load_new_font(paths[counter] + filename))
			return(true);
	}

	return(false);
}

int main(int argc, char * * argv) {

	coordinate arena_size(1000, 1000);

	// Set parameter defaults.

	bool print_final_outcome = true;
	bool show_speed_info = true;
	bool show_scanarcs = false;
	bool print_outcomes = true;
	bool strict_compile = true;
	bool run_battles = true;	// as opposed to just compiling.
	bool graphics = true;
	bool text_input = true;
	bool verbose = false;

	int framerate = 60;

	int maxlines = -1;		// Robots may be as long as is desired,
					// unless the user overrides this.
	
	int missile_insanity = INSANITY_SANE;	// Normal missiles

	int tournament_level = 0;	// no tournament info
	string tournament_file = "ktr2.rep";

	int per_round_tourn_level = 0;	// Ditto between rounds.

	int matches = 1;		// max # matches
	matchid_t predet_matchid = -1;	// Match ID of first round, if any
	bool use_predet_matchid = false;

	int max_CPU_speed = 5;		// In CPU cycles per game cycle.
	bool old_shields = false;	// Old shields take no damage upon hit.
	bool report_errors = false;

	int maxcycle = 100000;		// Duration of a single round, in cycles

	int missile_hit_radius = 14;

	int min_victory_margin = 32;	// Anything below this and it's
					// "simultaneous destruction".

	// Some other robot stats that we may make tunable later.
	int maxweight = 12;             // Number of points avail. to #configs
	int robot_radius = 4;
	int crash_range = 20;           // So says ATR2. Should be robot_radius
					// if realistic.
	int robot_disp_radius = 13;     // So that it's easier to actually see
					// the robot. ATR2 standard was 6 pix
					// on about 470x470 arena rendering.

	//////////////////////////////////////////////////////////////////
	
	// Get user parameters - filenames and overrides.
	vector<string> filenames;
	bool correct_params = setup_user_parameters(argc, argv, framerate, 
			max_CPU_speed, matches, tournament_level, 
			per_round_tourn_level, maxlines, missile_insanity,
			maxcycle, predet_matchid, verbose, print_outcomes, 
			print_final_outcome, graphics, text_input, run_battles, 
			show_scanarcs, report_errors, old_shields, 
			strict_compile, show_speed_info, filenames);

	if (filenames.empty())
		cerr << "Error: no robots specified." << endl;

	if (filenames.empty() || !correct_params) {
		print_usage(argv[0]);
		return(-1);
	}
	
	if (filenames.size() == 1) {
		cout << "Warning: Only one robot has been entered." << endl;
	}

	// Implication: If we're only compiling, it won't be graphical.
	if (!run_battles)
		graphics = false;

	/////////////////////////////////////////////////////////////////////

	ConsoleKeyboard * ckbd = NULL;
	if (!graphics && text_input)
		ckbd = &ConsoleKeyboard::instantiate();

	SDLHandler & SDLc = SDLHandler::instantiate(graphics, true);
	if (!SDLc.ready()) {
		cerr << "Error: Cannot init SDL!" << endl;
		return(-1);
	}
	SDLc.set_framerate(framerate);

	////////// Cycle data ATR2
	int show_explosion_length = 10; // Compromise - ATR2 has two different
					// for shots and mines.

	double granularity = 1;		// How many cycles we run before doing
					// collision detection/CPU execution.
					// Higher is faster but less accurate.

	// Display data native
	// Running at 1024x768 really stresses the display. Some of the
	// load is due to constantly redrawing fonts, but that's not where
	// the main hog lies.
	// Now it works! Huh.. Seems to be revenge of the 200k/400k problem 
	// with Linux's CPU scheduler.
	// Yup.
	int basis_x = 640, basis_y = 480;
	//int basis_x = 1024, basis_y = 768;
	int buffer_thickness = 14;

	// DEBUG. Alter this to 100
	double scan_lag = 100; // msecs

	map<string, Color> palette = make_palette();

	// Init explosions structure
	blasts explosions(0, show_explosion_length);
	explosions.set_color(B_MINE, palette.find("cyan")->second);
	explosions.set_color(B_MISSILE, palette.find("white")->second);
	explosions.set_color(B_ROBOT, palette.find("white")->second);

	///////////// Init weapons and comms structures /////////////

	vector<set<robot *> > comms_lookup(65536); // Quite cumbersome.
						   // Fixed bug that crashed
						   // when negative channels
						   // were specified.

	///////////////////////////// Init robots ///////////////////

	core_storage core_store(256);

	if (!compile_robots(filenames, maxweight, maxlines, verbose,
				strict_compile,	core_store))
		return(-1);

	if (!run_battles)
		return(0);

	vector<global_round_info> bot_stats;
	size_t counter;

	for (counter = 0; counter < filenames.size(); ++counter)
		bot_stats.push_back(global_round_info(snip_extraneous(
						filenames[counter])));

	// Set up graphics if so required.
	arena_disp * gui = NULL;
	Display * vmem = NULL;

	Font stdfont;

	if (graphics) {

		// Try to load the font we need for printing text in graphics 
		// mode. If we can't find it, let the user know.
		string fontfile = "DejaVuSansMono-Bold.ttf";
		if (!load_font(stdfont, fontfile)) {
			cerr << "Error: Cannot find standard font " << fontfile
				<< endl;

			cerr << "\tInstall the DejaVu fonts or run without " <<
				"graphics." << endl;

			return(-1);
		}

		if (!setup_graphics(vmem, gui, basis_x, basis_y, stdfont,
					arena_size, core_store.get_num_cores()))
			return(-1);

	}
	
	// ------------------------ DONE -------------------

	int curmatch;
	matchid_t matchid;

	bool global_quit = false;

	double start = get_abs_time();
	int tot_cycles = 0, this_cycle = 0;

	double kbd_check_rate;
	if (graphics)
		kbd_check_rate = 0.1;	// 100ms for graphics, suited for
					// instant response
	else	kbd_check_rate = 1.0;	// 1 sec for non-graphics, suited for
					// batch runs

	ticktimer kbd_trigger(kbd_check_rate, 1);
	game_balance balancer;
	cmd_parse disassembler;

	use_predet_matchid = (predet_matchid != -1);

	unsigned int seed = round(get_abs_time() * 1e3);
	srandom(seed); // Bluesky: Use /dev/urandom

	single_rand round_determine(random(), 0, RND_INIT);

	for (curmatch = 1; curmatch <= matches && !global_quit; ++curmatch) {
		// DEBUG: Just to make sure; might slow us down, later..
		/*srandom(1);
		srand(1);*/

		if (use_predet_matchid) {
			matchid = predet_matchid;
			use_predet_matchid = false;
		} else
			matchid = round_determine.irand();

		global_quit = !run_round(print_outcomes, report_errors, verbose,
			graphics, show_scanarcs, granularity, framerate,
			per_round_tourn_level, core_store, explosions,
			filenames, comms_lookup, old_shields, max_CPU_speed, 
			robot_radius, crash_range, missile_hit_radius,
			missile_insanity, arena_size, maxweight, maxcycle, 
			min_victory_margin, curmatch, matches, matchid, gui, 
			palette, robot_disp_radius, buffer_thickness, scan_lag,
			disassembler, balancer, stdfont, SDLc, ckbd,
			bot_stats, this_cycle, kbd_trigger);

		tot_cycles += this_cycle;
	}

	presenter present;

	// If the user wanted outcomes, print the global outcome.
	if (print_final_outcome) {
		string header = present.global_header();
		cout << endl << endl;
		cout << header << endl << string(header.size(), '~') << endl;

		for (counter = 0; counter < bot_stats.size(); ++counter)
			cout << present.global_summary(*bot_stats[counter].
					get_sum(), counter+1) << endl;
	}

	if (tournament_level > 0) {
		ofstream tournament_out(tournament_file.c_str());

		if (!tournament_out) {
			cerr << "Error: Could not open tournament file " <<
				tournament_file << " for writing!" << endl;
			return(-1);
		}

		// This should go in some .rep file.
		tournament_out << bot_stats.size() << endl;

		for (counter = 0; counter < bot_stats.size(); ++counter)
			tournament_out << present.get_tournament_line(
					*bot_stats[counter].get_sum(), 
					tournament_level, matches) << endl;

		tournament_out.close();
	}

	// Deallocate

	if (gui != NULL) delete gui;
	if (vmem != NULL) delete vmem;
	// ckbd gets removed by itself, since it's static.

	// Finally, show how much time was used.
	if (show_speed_info) {
		double span = get_abs_time() - start;
		cout << tot_cycles << " in " << span << " seconds, for an " <<
			"average of " << tot_cycles/(double)span << " cps. " 
			<< endl;
	}
}
