#ifndef _KROB_GUI
#define _KROB_GUI
// Arena and GUI display class.

// The GUI is made of these components:
// 	Statlets - information about robots' health/armor/etc.
// 	Arena - the arena itself.
// 	Round information - Match ID, cycles elapsed, etc.
// All of this is wrapped in the required border frames.

// We do this by making a bunch of Display classes. Then we update each and blit
// them all into the main display. This is very slow and should be replaced, but
// it's delaying release. Hopefully, abstracting this in an OOP manner should
// make it easier to replace when it does need to be.

// In an utopia, this would be in another thread.

#include "display.cc"
#include "widgets.cc"
#include "missile.cc"
#include "random.cc"
#include "tools.cc"
#include <vector>

using namespace std;

typedef enum { ST_LARGE, ST_SMALL } statlet_type;

class arena_disp {

	private:
		int max_small_onscreen;		// Statlets
		int max_large_onscreen;

		Display * display_out;		// Screen output.
		Display arena_field, round_info;
		vector<Display> statlets;
		statlet_type cur_statlet_type;

		Font typeface;

		Widgets widget;

		vector<Display> construct_statlets(const Display & real_display,
				const Color & inactive_background, statlet_type
				kind);
		void refresh_statlets(const vector<robot> & robots,
				const Color & active_background, 
				const Color & dark_meter, 
				const Color & no_error, int offset);

		Display construct_round_info(const Display & real_display,
				const Color & background_black);
		void refresh_round_info(const Color & background, const Color &
				text, matchid_t match_id, int cur_cycle, 
				int max_cycle, int cur_match, int max_match);

		Display construct_arena(const Display & real_display,
				const coordinate arena_size);

		double get_normalized_opacity(double latency, double
				current_time, double time_at_effect,
				double maxval);

		void refresh_arena(const Color & turret,
				const Color & missile_col, 
				const Color & overburn_missile_col,
				const Color & mine_col,
				const vector<robot> & robots, 
				const list<missile> & missiles,
				const list<mine> & mines,
				const blasts & explosions, 
				int robot_display_radius, coordinate arena_size,
				int buffer_thickness, double cycles_per_sec,
				double scan_lag, double sonar_lag,
				double radar_lag, bool show_scanarcs);

		bool blit_refresh(const Display & from, Display & to,
				double xmin, double ymin, double xmax, 
				double ymax, int num_robots);

	public:
		arena_disp(Display & real_display, const coordinate arena_size,
				Font & typeface_in, int num_robots);

		bool render_all(double x_separation, const Color border_color,
				const Color round_info_fg, const Color
				round_info_bg, const Color statlet_active_bg,
				const Color statlet_dark_meter, const Color
				statlet_no_error, const Color turret,
				const Color missile_col,
				const Color overburn_missile_col, const Color
				mine_col,
				matchid_t match_id, int cur_cycle,
				int max_cycle, int cur_match, int max_match,
				const vector<robot> & robots, 
				const list<missile> & missiles,
				const list<mine> & mines, 
				const blasts & explosions,
				int robot_display_radius, coordinate arena_size,
				int buffer_thickness, double cycles_per_sec,
				double scan_lag, double sonar_lag,
				double radar_lag, bool show_scanarcs, 
				int offset);
		
		// Create the virtual displays anew; used when the window is
		// resized.
		void derive_displays(const coordinate arena_size, 
				int num_robots);
		void resize_displays(int new_xsize, const coordinate 
				arena_size);
		void resize_displays(int new_xsize, const coordinate arena_size,
				int num_robots);

		// Used for moving from large statlets to small statlets and
		// vice versa. Bluesky later (with keypress to switch) may need
		// a force_large or force_small boolean so that we don't make
		// the functionality obscure.
		void reset_num_statlets(int num_robots);

		void display() { display_out->render(); }

		statlet_type get_cur_statlet_type() { return(cur_statlet_type);}

		// To check whether we can scroll down or not, for statlets
		bool can_scroll_statlets(int scroll_number, int num_robots);
};

// DONE: Mini-statlets for > 4 contestants.
vector<Display> arena_disp::construct_statlets(const Display & real_display,
		const Color & inactive_background, statlet_type kind) {
	// First get the x and y size of where we're going to dump this display,
	// then use ATR2 measures to find out how large the statlet window is
	// going to be (so that we retain nominal resolution independence).
	
	coordinate our_size;
	cur_statlet_type = kind;
	if (kind == ST_SMALL)
		our_size = coordinate(real_display.get_xsize() * 19 / 80,
				real_display.get_ysize() * 1 / 15);
	else
		our_size = coordinate(real_display.get_xsize() * 19 / 80,
				real_display.get_ysize() * 2 / 15);

	// There are four windows, and if there's no robot in action, it's grey
	// (dithered in ATR2), hence the need for default_grey.
	
	int num_windows;

	if (kind == ST_SMALL)
		num_windows = max_small_onscreen;
	else
		num_windows = max_large_onscreen;

	vector<Display> statlets_out;

	int counter;

	for (counter = 0; counter < num_windows; ++counter) 
		statlets_out.push_back(Display(false, our_size.x, our_size.y,
					32));

	for (counter = 0; counter < num_windows; ++counter) {
		assert(statlets_out[counter].ready()); // It better be working!
		statlets_out[counter].clear(inactive_background);
	}

	return(statlets_out);
}

void arena_disp::refresh_statlets(const vector<robot> & robots,
		const Color & active_background, const Color & dark_meter,
		const Color & no_error, int offset) {

	// Update the information in the statlets. A clever, utopian, method
	// would only do this if the robot has actually changed since the last
	// turn (vented heat or whatever), but we're not there yet. We use
	// a vector to robots so that it never has to bother about whether
	// the robot is alive or dead.
	
	// Active_background is the background color that statlets with robots
	// have - in ATR2, this is blue. dark_meter is the dark (base) color
	// for the meters, and no_error is the white that gets used for the
	// "robot hasn't committed any errors yet" message.

	for (size_t counter = offset; counter < min(robots.size(), 
				offset + statlets.size()); ++counter) {
		statlets[counter - offset].clear(active_background);
		if (cur_statlet_type == ST_SMALL)
			widget.small_botinfo(statlets[counter - offset],
					robots[counter], dark_meter, typeface);
		else
			widget.large_botinfo(statlets[counter - offset], 
					robots[counter], dark_meter, no_error, 
					typeface);
	}

}

Display arena_disp::construct_round_info(const Display & real_display, 
		const Color & background_black) {

	// Pretty much the same strategy as in construct_statlets, except that
	// we need only one, and the default background is now black.

	coordinate our_size(real_display.get_xsize() * 19/80,
			real_display.get_ysize() * 11 / 120);

	Display round_info(false, our_size.x, our_size.y, 32);

	assert (round_info.ready());

	return(round_info);
}

void arena_disp::refresh_round_info(const Color & background_black, 
		const Color & text, matchid_t match_id, int cur_cycle,
		int max_cycle, int cur_match, int max_match) {

	// text is the color of the text, match_id is the match ID (random 
	// seed), cur_cycle is the current cycle, with max_cycle being the
	// maximum; the same explanation goes for cur_match and max_match.

	// In this case, the box is always active, and so the background is
	// always black.

	round_info.clear(background_black);

	widget.round_stats(round_info, text, typeface, match_id, cur_cycle,
			max_cycle, cur_match, max_match);
}

Display arena_disp::construct_arena(const Display & real_display, coordinate
		arena_size) {

	// This is quite simple. Since the background is black, we don't have
	// to clear or anything, just dump a Display with the right relative
	// dimensions.
	
	// Arena size is in meters (that is, ingame units). All we want is
	// the proportion so we know what aspect ratio to make, thus avoiding
	// non-round circle problems.
	
	int scaled_arena_xsize = round(0.73 * real_display.get_xsize());
	int scaled_arena_ysize = scaled_arena_xsize * (arena_size.y / (double)
			arena_size.x);

	Display to_return(false, scaled_arena_xsize, scaled_arena_ysize, 32);

	assert(to_return.ready());
	to_return.clear();

	return(to_return);
}

// Determine normalized opacity for fading effects (scanner, sonar, and radar
// information). The latency is the duration the effect should be visible,
// current_time is the current time since some fixed point, time_at_effect is
// the time when the event happened, and maxval is the maximum value of opacity.
// Then we return 1 at current_time = time_at_effect sloping down to 0 at
// current_time = time_at_effect + latency. Any excess or unexpected values
// return -1.
double arena_disp::get_normalized_opacity(double latency, double current_time, 
		double time_at_effect, double maxval) {

	if (current_time > time_at_effect + latency || current_time < 
			time_at_effect) return(-1);

	return(renorm(time_at_effect, time_at_effect + latency, current_time, 
				maxval, 0.0));
}

// Perhaps a "robot_color" generator that makes distinguishably different shield
// and turret colors? Nah, turret is grey no matter whose turret it is; but
// shield should be a darker version of the robot's hue. DONE.
void arena_disp::refresh_arena(const Color & turret,
		const Color & missile_col, const Color & overburn_missile_col,
		const Color & mine_col,
		const vector<robot> & robots, const list<missile> & missiles, 
		const list<mine> & mines, const blasts & explosions, 
		int robot_display_radius, coordinate arena_size, 
		int buffer_thickness, double cycles_per_sec, double scan_lag, 
		double sonar_lag, double radar_lag, bool show_scanarcs) {

	// Utopian versions would know what has changed and only remove the
	// necessary parts.

	// First clear what's already drawn here.
	arena_field.clear();

	// Set up buffer_thickness adjustments. These indent the actual arena
	// a bit so that it's possible to see what's going on at, say, (0,0).
	coordinate base(buffer_thickness, buffer_thickness),
		   adjusted_arena(arena_size.x + buffer_thickness * 2,
				   arena_size.y + buffer_thickness * 2),
		   less_arena(arena_size.x - buffer_thickness,
				   arena_size.y - buffer_thickness);

	double rel_robot_disp_radius = robot_display_radius / (double)
		adjusted_arena.y;

	// Get sonar cycle persistence in terms of cycles instead of msecs.
	// This is how long the sonar afterimage is displayed (so we can see
	// the result).
	// Then do the same for scanning!
	double sonar_cycle_persistence = cycles_per_sec * (sonar_lag / 1000.0);
	double scan_cycle_persistence = cycles_per_sec * (scan_lag / 1000.0);
	double radar_cycle_persistence = cycles_per_sec * (radar_lag / 1000.0);

	// Draw faint edge border (not in ATR2) so that we can see when a
	// missile passes through the edge and thus doesn't impact anything.
	Color wallcolor(0.3, 0, 0);

	arena_field.box(0, 0, 1, base.y/adjusted_arena.y, wallcolor, 1.0);
	arena_field.box(0, 1, 1, 1 - base.y/adjusted_arena.y, wallcolor, 1.0);
	arena_field.box(0, 0, base.x/adjusted_arena.x, 1, wallcolor, 1.0);
	arena_field.box(1, 0, 1 - base.x/adjusted_arena.x, 1, wallcolor, 1.0);


	// Draw robots.
	for (vector<robot>::const_iterator pos = robots.begin(); pos !=
			robots.end(); ++pos) {
		if (pos->dead()) continue; // Don't draw the dead.

		// Turn 0..arena_size into base..less_arena, then divide by
		// arena_size to get the "adjusted" 0..1 position.
		coordinate renorm_pos = renorm(coordinate(0, 0), 
				arena_size, pos->get_pos(), base, 
				less_arena) / arena_size;

		double n_turr_head_lastscan = hex_to_deg(pos->
				get_scan_center(false));

		// Determine if the radar has fired recently enough that we
		// should let the user know.
		double radar_opacity = get_normalized_opacity(
				radar_cycle_persistence, pos->get_time(), 
				pos->get_last_radar_time(), 1.0);

		if (radar_opacity < 0) radar_opacity = 0;

		// Draw the robot.
		widget.draw_robot(arena_field, renorm_pos.x, renorm_pos.y,
				 rel_robot_disp_radius, hex_to_deg(pos->
					 get_heading()), hex_to_deg(pos->
					 turret_heading), pos->assigned_color,
				 turret, pos->shield_color, pos->is_shielded(),
				 radar_opacity);

		// Draw the sonar afterimage if we did any sonar checks lately.
		double norm_sonar_opaq = 0.7, sonar_opacity = -1;
		if (pos->get_last_sonar_time() != -1)
			sonar_opacity = get_normalized_opacity(
					sonar_cycle_persistence, pos->
					get_time(), pos->get_last_sonar_time(),
					norm_sonar_opaq);
					
		// Too early or no sonar? If so, don't draw.
		if (sonar_opacity != -1 && sonar_opacity > 0) {

			coordinate renorm_sonar_pos = renorm(coordinate(0, 0),
					arena_size, pos->get_last_sonar_pos(),
					base, less_arena) / arena_size;

			widget.draw_sonar(arena_field, renorm_sonar_pos.x,
					renorm_sonar_pos.y, pos->
					get_sonar_maxrange()/adjusted_arena.y, 
					pos->get_last_sonar_angle(),
					turret, true, sonar_opacity);
		}

		// Draw scanner afterimage if we did any scans lately. [hSA]

		double norm_scan_opaq = 0.7, scan_opacity = -1;
		if (pos->get_scan_time(false) != -1)
			scan_opacity = get_normalized_opacity(
					scan_cycle_persistence, pos->get_time(),
					pos->get_scan_time(false), 
					norm_scan_opaq);

		Color scangrey(0.7, 0.7, 0.7);

		// Only draw if we haven't faded out.
		if (scan_opacity > 0 && show_scanarcs) {

			coordinate scanpos_raw = pos->get_scan_pos(false);
			double maximal_edge = max(arena_size.y - scanpos_raw.y,
					arena_size.x - scanpos_raw.x);
			maximal_edge = max(max(maximal_edge, scanpos_raw.y),
					scanpos_raw.x);

			coordinate scanpos_norm = renorm(coordinate(0, 0),
					arena_size, scanpos_raw,
					base, less_arena) / arena_size;

			// To not make it too slow, we should take the minimum
			// of the range here and that to the edge of the screen.
			// We also take the minimum of the scanner's range and
			// the distance to the target (if any), so it's more 
			// clear which robot was detected, if there are more
			// than one.
			double real_dist = min(pos->get_scan_dist(false),
					pos->get_scanrange());
			// This could be misleading, since the end of the arcs
			// don't go past the edge if we do so, but would if
			// we don't.. Hm.
			//real_dist = min(real_dist, maximal_edge);
			real_dist /= arena_size.y;

			// Since the scanning span is a scalar (shows
			// magnitude of something, not direction), it must
			// be converted to degrees manually.
			double scan_arc = 2 * pos->get_scan_span(false) / 256.0
				* 360.0;
			
			widget.scanarc(arena_field, scanpos_norm.x, 
					scanpos_norm.y, real_dist, 
					n_turr_head_lastscan, scan_arc,
					scangrey, true, turret, scan_opacity,
					0.5);
		}
	}

	// Draw missiles.
	
	for (list<missile>::const_iterator shell = missiles.begin(); shell !=
			missiles.end(); ++shell)
		widget.draw_missile(base, arena_field, *shell, 1.0, false,
				missile_col, overburn_missile_col, 
				adjusted_arena.x, adjusted_arena.y);

	// .. mines,
	
	for (list<mine>::const_iterator mshell = mines.begin(); mshell !=
			mines.end(); ++mshell) {
		coordinate norm_mine_pos = renorm(coordinate(0, 0), arena_size,
				mshell->get_pos(), base, less_arena) / 
			arena_size;

		widget.draw_mine(arena_field, norm_mine_pos.x, norm_mine_pos.y,
				0.002, mine_col);
	}

	// And finally, the explosions.
	explosions.draw_blasts(base, arena_field, adjusted_arena);
}

// -- PUBLIC //

void arena_disp::derive_displays(const coordinate arena_size, int num_robots) {
	Color background_black(0, 0, 0), inactive_background(0.16, 0.16, 0.16);

	if (num_robots > max_large_onscreen)
		statlets = construct_statlets(*display_out, inactive_background,
				ST_SMALL);
	else	statlets = construct_statlets(*display_out, inactive_background,
			ST_LARGE);

	round_info = construct_round_info(*display_out, background_black);
	arena_field = construct_arena(*display_out, arena_size);
	arena_field.clear();
}

// The display constructors here (1x1 pixel at 32 bpp) are just temporary
// fillers until we initialize them properly with construct_*.
arena_disp::arena_disp(Display & real_display, const coordinate arena_size, 
		Font & typeface_in, int num_robots) : arena_field(false, 1, 1, 
			32), round_info(false, 1, 1, 32) {
	max_small_onscreen = 11;
	max_large_onscreen = 6;

	display_out = &real_display;

	// Check that the typeface is in fact available.
	assert (typeface_in.get_font(-1) != NULL);
	// Would operator= be better?
	typeface.load_new_font(typeface_in.get_filename());

	derive_displays(arena_size, num_robots);

}

void arena_disp::resize_displays(int new_xsize, const coordinate arena_size,
		int num_robots) {
	// First, adjust the size of the video-out.
	display_out->resize(new_xsize);
	// Then rederive the rest from it.
	derive_displays(arena_size, num_robots);
}

void arena_disp::resize_displays(int new_xsize, const coordinate arena_size) {
	resize_displays(new_xsize, arena_size, statlets.size());
}

void arena_disp::reset_num_statlets(int num_robots) {
	derive_displays(display_out->get_size(), num_robots);
}

bool arena_disp::blit_refresh(const Display & from, Display & to,
		double xmin, double ymin, double xmax, double ymax,
		int num_robots) {
	// First try to blit. If we managed to blit, return true.
	// Otherwise, if it's -2, that means everything must be redrawn, so
	// rederive, and in either case, return false since the blit was
	// unsuccessful.
	
	int blit_attempt = to.blit(from, xmin, ymin, xmax, ymax);
	if (blit_attempt == 0) return(true);

	if (blit_attempt == -2)
		derive_displays(display_out->get_size(), num_robots);

	return(false);
}

// X_separation is the thickness of the borders. The purpose of offset is to
// let the user "scroll" robot stats while in graphics mode. DONE: Proof
// this for the case where he scrolls towards the edge (e.g so that the last
// bot is at the top). Ideally, it shouldn't let him.
bool arena_disp::render_all(double x_separation, const Color border_color,
		const Color round_info_fg, const Color
		round_info_bg, const Color statlet_active_bg,
		const Color statlet_dark_meter, const Color statlet_no_error, 
		const Color turret, const Color missile_col,
		const Color overburn_missile_col, const Color mine_col,
		matchid_t match_id, int cur_cycle, int max_cycle, 
		int cur_match, int max_match, const vector<robot> & robots,
		const list<missile> & missiles, const list<mine> & mines,
		const blasts & explosions, int robot_display_radius, 
		coordinate arena_size, int buffer_thickness, 
		double cycles_per_sec, double scan_lag, double sonar_lag,
		double radar_lag, bool show_scanarcs, int offset) {

	// Robot display radius is how large (with regards to the arena, which
	// is to say it's in simulated units) the robot should appear. This
	// differs from the robot's real radius, which is much smaller.
	// Scan lag is in msecs and determines how long an earlier scan stays
	// on the display; this simulates a "radar" effect, makes the display
	// less flickery, and makes it possible to see scans when a single frame
	// lasts more than a single cycle.

	// Basic error checking.
	if (display_out == NULL)	return(false);
	if (!display_out->ready())	return(false);

	// Clamp offset so he can't scroll past the edge, unless that
	// would go below zero.
	offset = max((size_t)0, min((size_t)offset, 
				robots.size()-statlets.size()));

	// First refresh the virtual displays we're going to show.

	refresh_statlets(robots, statlet_active_bg, statlet_dark_meter,
			statlet_no_error, offset);

	refresh_arena(turret, missile_col, overburn_missile_col,
			mine_col, robots, missiles, mines, explosions,
			robot_display_radius, arena_size, buffer_thickness,
			cycles_per_sec, scan_lag, sonar_lag, radar_lag, 
			show_scanarcs);

	refresh_round_info(round_info_bg, round_info_fg, match_id, cur_cycle,
			max_cycle, cur_match, max_match);

	// Clear what we're going to draw to.
	// That's not necessary, since we're going to be overwriting every
	// changed area anyhow. Remove comments for debugging to see if any 
	// areas are undrawn.
	//display_out->clear(0.1, 0.5, 0.9);

	// Handle the arena and its borders.
	coordinate rel_size = arena_field.get_size() / display_out->get_size();

	double y_separation = 0.5 - rel_size.y * 0.5;

	// Border around the arena.
	widget.border(*display_out, 0, 0, rel_size.x + x_separation * 2,
			1.0, x_separation, y_separation, border_color, true,
			false);

	// .. and around everything to fix some glitches. (But WTH?)
	widget.border(*display_out, 0, 0, 1.0, 1.0, x_separation, y_separation,
			border_color, true, false);

	// Blit the arena
	if (!blit_refresh(arena_field, *display_out, x_separation, 
				y_separation, 1, 1, robots.size()))
		return(false);

	// Now copy the statlets. We just draw borders and copy statlet blocks
	// down until we're done. In the case of >4, we should use smaller
	// statlets, but for now we just quit once we reach the bottom.
	
	// Get the round information block size so we know when to stop. We'll
	// also use this for displaying the block itself;
	
	// Sans borders
	coordinate rel_size_roundinfo = round_info.get_size() / 
		display_out->get_size();

	// With borders
	coordinate far_end_roundinfo(1 - (x_separation * 2 + 
				rel_size_roundinfo.x), 1 - (y_separation * 2 +
					rel_size_roundinfo.y));

	double yloc = 0;
	for (size_t counter = 0; counter < statlets.size(); ++counter) {
		coordinate rel_size_statlet = statlets[counter].get_size() /
			display_out->get_size();
		// Count down from the last unused y position to find out the
		// coordinates (1-normalized) of the other edge of the box.
		coordinate far_end_statlet(1 - (x_separation * 2 +
					rel_size_statlet.x), (y_separation * 2
						+ rel_size_statlet.y));

		if (far_end_statlet.y > far_end_roundinfo.y) break;

		widget.border(*display_out, far_end_statlet.x, yloc, 1,
				yloc + far_end_statlet.y, x_separation,
				y_separation, border_color, false, false);

		// Get the start and end coordinates inside the box.
		double near_x = 1 - (x_separation + rel_size_statlet.x);
		// And blit!
		if (!blit_refresh(statlets[counter], *display_out, near_x,
					yloc + y_separation, 1, 1,
					robots.size()))
			return(false);

		yloc += rel_size_statlet.y + y_separation;
	}

	// Draw the round info.
	widget.border(*display_out, far_end_roundinfo.x, far_end_roundinfo.y,
			1, 1, x_separation, y_separation, border_color,
			false, false);

	coordinate inside_end(1 - (x_separation + rel_size_roundinfo.x),
			1 - (y_separation + rel_size_roundinfo.y));

	// Blit!
	// Perhaps change 1,1 to the other end if that makes it any faster.
	if (!blit_refresh(round_info, *display_out, inside_end.x, inside_end.y,
				1, 1, robots.size()))
		return(false);

	return(true);
}

bool arena_disp::can_scroll_statlets(int scroll_number, int num_robots) {
	// If scrolling to this margin would cause one of the areas to be
	// unoccupied, then return false, otherwise return true.
	
	return (num_robots - scroll_number >= statlets.size() );
}

#endif
