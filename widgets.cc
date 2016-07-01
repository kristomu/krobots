// Widgets for user interface and display:
// 	Drop shadow border, bot statistics box, robot (with or without shield,
// 	with turret at certain level - perhaps differing shield shade according
// 	to strength), scanning arc (with or without accuracy-bracketing),
// 	missiles (really just lines, but depend on whether they're overburnt),
// 	mines (really just dots or filled circles, but abstraction is good),
// 	and.. I guess that's it!
// 	Perhaps have the scanning arc where there's an enemy light up?

// Bluesky: Similar scanarc drawing except that it takes a robot (or scanner or
// something) as input; and handle line error where if crash_range < 
// disp_radius, it doesn't draw any of the lines that go beyond the bounds.

#ifndef _KROB_WIDGET
#define _KROB_WIDGET

#include "robot.cc"
#include "tools.cc"
#include "color.cc"
#include "display.cc"
#include "missile.cc"
#include "random.cc"
#include "blast.cc"

// Opaquity later?

class Widgets {

	public:
		void border(Display & target, double outer_xmin, double
				outer_ymin, double outer_xmax, 
				double outer_ymax, double width, double height,
				const Color & color, bool draw_outer, bool
				clear_inside);
		void border(Display & target, double outer_xmin, double
				outer_ymin, double outer_xmax,
				double outer_ymax, double width, double height,
				const Color & color);
		void linear_meter(Display & target, double xmin, double ymin,
				double xmax, double ymax, bool horizontal,
				double normalized_value, const Color & fg_color,
				const Color & bg_color);
		// constant size
		void default_text(Display & target, double xpos, double ypos,
				string text, const Color & color, Font &
				font);
		// rotation is heading wrt straight rightwards (positive on
		// x axis). Turret_rotation is wrt straight rightwards as well,
		// not relative. If inner_opacity is 1, the inside of the robot
		// shape will be filled with the color, if it's 0, nothing of
		// the sort happens (used to show when the robot uses radar).
		void draw_robot(Display & target, double centerx, 
				double centery, double radius, double rotation, 
				double turret_rotation,	const Color & color, 
				const Color & turret_color, const Color &
				shield_color, bool shields_on, 
				double inner_opacity);
		// Draw a missile of the given length starting from center
		// and heading angle.
		void draw_missile(Display & taget, double centerx, 
				double centery, double length, double angle, 
				bool overburn, const Color & normal, 
				const Color & overburn_c);
		// Draw a missile as ATR2 does it - from start pt to end pt
		// with a fixed time interval.
		void draw_missile(const coordinate base, Display & target, 
				const missile & source, double time_interval, 
				bool overburn, const Color & normal, 
				const Color & overburn_c, int arena_xsize, 
				int arena_ysize);
		void draw_missile(Display & target, const missile & source,
				double time_interval);
		void draw_mine(Display & target, 
				double centerx, double centery, double radius, 
				const Color & color);
		// Moved to blast.cc instead. 
	/*	void draw_blast(Display & target, double seconds_since_start, 
				const blast & source, int arena_xsize, 
				int arena_ysize, const Color & blast_color);*/
		void scanarc(Display & target, double centerx, double centery,
				double radius, double turret_angle, 
				double breadth, const Color & color, 
				bool color_quadrants, 
				const Color & quadrant_color, 
				double quadrant_opaquity,
				double quadrant_opaquity_falloff);
		void draw_sonar(Display & target, double centerx, 
				double centery, double radius, 
				double given_angle, const Color & color,
				bool mark_angle, double opaquity);

		// Would be neat: Mark off unacceptable heat.
		void large_botinfo(Display & target, const Color & bot_color,
				const Color & dark_meter, 
				const Color & no_error, Font & typeface, 
				string name, string message,
				double armor_fraction, double heat_fraction, 
				int wins, int kills, int deaths, bool has_error,
				signed short error);
		void large_botinfo(Display & target, const robot & source,
				const Color & dark_meter, 
				const Color & no_error, Font & typeface);
		// Small
		void small_botinfo(Display & target, const Color & bot_color,
				const Color & dark_meter, Font & typeface,
				string name, double armor_fraction, 
				double heat_fraction, int wins);
		void small_botinfo(Display & target, const robot & source,
				const Color & no_error, Font & typeface);

		void round_stats(Display & target, const Color & text_color,
				Font & typeface, const matchid_t matchid,
				const int cur_cycle, const int max_cycle,
				const int cur_match, const int max_match);
};

void Widgets::border(Display & target, double outer_xmin, double outer_ymin,
		double outer_xmax, double outer_ymax, double width,
		double height, const Color & color, bool draw_outer,
		bool clear_inside) {
	// Rather simple: 1/4 of the width on both outer and inner is reserved
	// for the drop shadow. The drop shadow on outer is white at UL and
	// dark grey at LR. The color given is assumed to be the midpoint; the
	// highlight is 0.32 brighter and the shadow is 0.32 darker.
	
	// NOTE: Non-square pixels may be a bother here.

	// First draw the box itself.
	Color highlight(color.get_by_idx(0) + 0.32, color.get_by_idx(1) + 0.32,
			color.get_by_idx(2) + 0.32);
	Color shadow(color.get_by_idx(0) - 0.32, color.get_by_idx(1) - 0.32,
			color.get_by_idx(2) - 0.32);

	// The empty space inside
	Color black (0, 0, 0);
	double inner_xmin = outer_xmin + width, inner_ymin = outer_ymin+ height,
	       inner_xmax = outer_xmax - width, inner_ymax = outer_ymax- height;

	// Draw the box itself. Now it draws only the "blocks" making up the
	// rectangle, and doesn't fill in the inside, since either we aren't
	// going to do so (if clear_inside is false), or we're going to replace
	// it with black anyhow (if clear_inside is true).
	// Note that we have to draw the blocks if draw_outer is on, since
	// they provide the corners (non drop-shadowed).
	double adj_outer_xmin, adj_outer_ymin, adj_outer_xmax, adj_outer_ymax;

	if (draw_outer) {
		adj_outer_xmin = outer_xmin; adj_outer_xmax = outer_xmax;
		adj_outer_ymin = outer_ymin; adj_outer_ymax = outer_ymax;
	} else {
		adj_outer_xmin = outer_xmin + width / 4.0;
		adj_outer_xmax = outer_xmax - width / 4.0;
		adj_outer_ymin = outer_ymin + width / 4.0;
		adj_outer_ymax = outer_ymax - width / 4.0;
	}

	// Draw the solid blocks.
	// Horizontals
	target.box(adj_outer_xmin, adj_outer_ymin, adj_outer_xmax, inner_ymin,
			color, 1.0);
	target.box(adj_outer_xmin, inner_ymax, adj_outer_xmax, adj_outer_ymax,
			color, 1.0);
	// Verticals
	target.box(adj_outer_xmin, adj_outer_ymin, inner_xmin, adj_outer_ymax,
			color, 1.0);
	target.box(inner_xmax, adj_outer_ymin, adj_outer_xmax, adj_outer_ymax,
			color, 1.0);

	// Off by one in drop shadow! Fix. (I don't remember if I fixed it
	// after writing that comment.)
	if (clear_inside)
		target.box(inner_xmin, inner_ymin, inner_xmax, inner_ymax, 
				black, 1.0);

	if (draw_outer) {
		// Then draw the highlight - upper left of outer
		target.box(outer_xmin, outer_ymin, outer_xmax - width/4.0, 
				outer_ymin + width/4.0, highlight, 1.0);// Upper
		target.box(outer_xmin, outer_ymin, outer_xmin + width/4.0,
				outer_ymax - width/4.0, highlight, 1.0);// Left
		// Shadow - lower right of outer
		target.box(outer_xmin + width/4.0, outer_ymax - width/4.0,
				outer_xmax,outer_ymax, shadow, 1.0);	// Lower
		target.box(outer_xmax - width/4.0, outer_ymin, outer_xmax,
				outer_ymax - width/4.0, shadow, 1.0);	// Right
	}

	// Highlight - lower right of inner
	// Readjust inner_* so that they point to the last painted part of the
	// borders (to avoid off-by-one)
	double fct = width/4.0;
	inner_xmax += fct; inner_ymax += fct;
	inner_xmin -= fct; inner_ymin -= fct;

	target.box(inner_xmin + width/4.0, inner_ymax - width/4.0, 
			inner_xmax,inner_ymax, highlight, 1.0);         // Lower
	target.box(inner_xmax - width/4.0, inner_ymin, inner_xmax,
			inner_ymax - width/4.0, highlight, 1.0);        // Right
	// Shadow - upper left of inner
	
	target.box(inner_xmin, inner_ymin, inner_xmax - width/4.0, 
			inner_ymin + width/4.0, shadow, 1.0);		// Upper
	target.box(inner_xmin, inner_ymin, inner_xmin + width/4.0,
			 inner_ymax - width/4.0, shadow, 1.0);		// Left

}

void Widgets::border(Display & target, double outer_xmin, double outer_ymin,
		double outer_xmax, double outer_ymax, double width,
		double height, const Color & color) {
	border(target, outer_xmin, outer_ymin, outer_xmax, outer_ymax, width,
			height, color, true, true);
}


void Widgets::linear_meter(Display & target, double xmin, double ymin,
		double xmax, double ymax, bool horizontal,
		double normalized_value, const Color & fg_color,
		const Color & bg_color) {

	// This one is even simpler. First we draw a bar of full size, then
	// we draw one of the right fraction to cover it.
	// Bluesky: Add limit gauges (say, for overheat shutdown) and regular 
	// lines for metering.
	
	target.box(xmin, ymin, xmax, ymax, bg_color, 1.0);
	if (horizontal)		// Points to the right
		target.box(xmin, ymin, xmin + normalized_value * (xmax-xmin),
				ymax, fg_color, 1.0);
	else			// Points upwards
		target.box(xmin, ymax - normalized_value * (ymax - ymin),
				xmax, ymax, fg_color, 1.0);
}

void Widgets::default_text(Display & target, double xpos, double ypos,
		string text, const Color & color, Font & font) {

	// Standard size is 8 px on 640x480, thus 0.017, but since the font
	// is somewhat different we need at least 9, 0.0187.
	
	target.print(xpos, ypos, 0.019, font, color, 1, text);
}

void Widgets::draw_robot(Display & target, double centerx, double centery,
		double radius, double rotation, double turret_rotation,
		const Color & color, const Color & turret_color,
		const Color & shield_color, bool shields_on, 
		double inner_opacity) {
	// First generate three points, one to 0 degrees, one to 70, and
	// one to 110. (Fix angles later)
	// Then if the shields are up, draw the circle required to encompass,
	// at the color of the robot.
	// Finally, draw the turret at 90 (angle to be supplied by "user".
	
	// Radius is really just (radius of robot in realspace)/(size of arena)
	
	double xpoints[3], ypoints[3], angles[3];
	angles[0] = deg_to_radian(rotation + 180 - 30); // Hind right
	angles[1] = deg_to_radian(rotation + 180 + 30); // Hind left
	angles[2] = deg_to_radian(rotation + 0);        // Front

	//cout << "Draw robot: " << centerx << ", " << centery << " angle " << rotation << " deg " << endl;

	for (int counter = 0; counter < 3; ++counter) {
		xpoints[counter] = radius * cos(angles[counter]);
		ypoints[counter] = radius * sin(angles[counter]);
		// Correct so that the coordinates are by the same standard
		// as the circle.
		target.stretch_to_aspect(xpoints[counter], ypoints[counter]);
		xpoints[counter] += centerx;
		ypoints[counter] += centery;
	}

	/*cout << "Points: (" << xpoints[0] << ", " << ypoints[0] << "), (" << 
		xpoints[1] << ", " << ypoints[1] << "), (" << xpoints[2] <<
		", " << ypoints[2] << ")" << endl;*/

	// Draw the robot itself
	if (inner_opacity > 0)
		target.triangle(xpoints[0], ypoints[0], xpoints[1], ypoints[1],
				xpoints[2], ypoints[2], color, color, true,
				1.0, inner_opacity);
	else	target.triangle(xpoints[0], ypoints[0], xpoints[1], ypoints[1],
			xpoints[2], ypoints[2], color, 1.0);
	/*target.line(xpoints[0], ypoints[0], xpoints[1], ypoints[1],
			color, 1.0);
	target.line(xpoints[1], ypoints[1], xpoints[2], ypoints[2],
			color, 1.0);
	target.line(xpoints[0], ypoints[0], xpoints[2], ypoints[2],
			color, 1.0);*/

	// Another color?
	if (shields_on)
		target.circle(centerx, centery, radius, shield_color, 1.0);

	// Draw the turret
	double endx = radius * cos(deg_to_radian(turret_rotation)),
	       endy = radius * sin(deg_to_radian(turret_rotation));
	target.stretch_to_aspect(endx, endy);
	endx += centerx;
	endy += centery;
	target.line(centerx, centery, endx, endy, turret_color, 1.0);

	// Debug: Centerpoint
	target.line(centerx, centery, centerx, centery, color, 1.0);
}

void Widgets::draw_missile(Display & target, double centerx, double centery,
		double length, double angle, bool overburn,
		const Color & normal, const Color & overburn_c) {

	// Convert polar to cartesian, then draw. Origin is centerx, centery,
	// length is r and angle is theta. The angles are all hexdegrees,
	// because that's what the robot uses.
	
	double destx = length * cos(hex_to_radian(angle));
	double desty = length * sin(hex_to_radian(angle));
	target.stretch_to_aspect(destx, desty);

	if (overburn)
		target.line(centerx, centery, centerx + destx, centery + desty,
				overburn_c, 1.0);
	else	target.line(centerx, centery, centerx+ destx, centery + desty,
			normal, 1.0);
}

void Widgets::draw_missile(const coordinate base, Display & target, 
		const missile & source, double time_interval, bool overburn, 
		const Color & normal, const Color & overburn_c, 
		int arena_xsize, int arena_ysize) {

	coordinate start_pos = source.get_pos(), end_pos = source.
		predict_motion(start_pos, time_interval);

	start_pos = start_pos + base; end_pos = end_pos + base;

	//cout << "Draw_missile consistency check: " << 
	//	start_pos.distance(end_pos) << endl;

	// Normalize
	start_pos.x /= (double)arena_xsize;
	start_pos.y /= (double)arena_ysize;
	end_pos.x /= (double)arena_xsize;
	end_pos.y /= (double)arena_ysize;

	if (overburn)
		target.line(start_pos.x, start_pos.y, end_pos.x, end_pos.y,
				overburn_c, 1.0);
	else	target.line(start_pos.x, start_pos.y, end_pos.x, end_pos.y,
			normal, 1.0);
}

void Widgets::draw_mine(Display & target, double centerx, double centery, 
		double radius, const Color & color) {

	// Either a circle or a diamond-shape. Diamond shape would be nicer,
	// (take four 90 degrees apart and draw four lines), but for simplicity,
	// we'll go with a circle. The radius must be one pixel, 0.002 in
	// normalized coordinates.

	target.circle(centerx, centery, 0.002, color, 1.0);
}

// Draws an explosion effect. The effect starts with a circle at radius 0
// (a point, basically) at time 0, then the circle spreads outwards until
// it reaches maxradius at time "maxtime". Circles will be distorted if the
// aspect ratio is incorrect.
/*void Widgets::draw_blast(Display & target, double seconds_since_start, 
		const blast & source, int arena_xsize, int arena_ysize, 
		const Color & blast_color) {

	// Find out our state of progress in the animation
	double animprogress = (seconds_since_start - source.start_time)
		/(source.maxtime - source.start_time);

	cout << "animprogress " << animprogress << endl;
	cout << "ATAS II: " << seconds_since_start << endl;

	coordinate normalized;
	normalized.x = source.impact_point.x / (double)arena_xsize;
	normalized.y = source.impact_point.y / (double)arena_ysize;
	cout << "location: " << normalized.x << ", " << normalized.y << endl;

	target.circle(normalized.x, normalized.y, source.maxradius * 
			animprogress / (double)arena_ysize, blast_color, 1.0);
}*/


void Widgets::scanarc(Display & target, double centerx, double centery,
		double radius, double turret_angle, double breadth,
		const Color & color, bool color_quadrants, 
		const Color & quadrant_color, double quadrant_opaquity,
		double quadrant_opaquity_falloff) {

	// First of all, if we're going to draw the quadrants, do so first
	// since they're solid.
	// "Quadrant" sizes are:
	// 	0:        1/4
	// 	1 and -1: 1/4
	// 	2 and -2: 1/8
	// Note that rounding effects may come into play. They're not
	// modeled here. DONE: Handle that once we've reproduced the actual
	// targeting code. (Now uses own pie routine)

	if (color_quadrants) {
		double q_one = quadrant_opaquity_falloff * quadrant_opaquity;
		double q_two = quadrant_opaquity_falloff * q_one;

		// 0 is 1/4, so 4 * 2 = 8.
		target.pie(true, centerx, centery, radius, turret_angle - 
				breadth/8.0, turret_angle + breadth/8.0,
				quadrant_color, quadrant_opaquity);
		// Although these /add/ to the angle, they're - because
		// - coordinates in ATR go to the left whereas they go to the
		// right in a proper polar coordinate system.

		// -1
		target.pie(true, centerx, centery, radius, turret_angle +
				breadth / 8.0, turret_angle + breadth / 8.0 +
				breadth/4.0, quadrant_color, q_one);
		// -2
		target.pie(true, centerx, centery, radius, turret_angle +
				breadth/8.0 + breadth / 4.0, turret_angle +
				breadth / 2.0, quadrant_color, q_two);
		// +1
		target.pie(true, centerx, centery, radius, turret_angle - 
				(breadth/ 8.0 + breadth/4.0), turret_angle +
				breadth / 8.0, quadrant_color, q_one);
		// +2
		target.pie(true, centerx, centery, radius, turret_angle - 
				breadth/2.0, turret_angle - (breadth/8.0 + 
					breadth/4.0), quadrant_color, q_two);
	}

	// Draw a pie at the required distance (in pixelspace coordinates)
	
	target.pie(false, centerx, centery, radius, turret_angle - breadth/2.0,
			turret_angle + breadth/2.0, color, 
			quadrant_opaquity);
}

// Draw a sonar (just a circle). If mark_angle is true, we also delineate the
// noisy angle the sonar did return, by a line from the robot to the circle
// at the specified angle. This won't work on non-square aspect ratios.
void Widgets::draw_sonar(Display & target, double centerx, double centery, 
		double radius, double given_angle, const Color & color,
		bool mark_angle, double opaquity) {

	target.circle(centerx, centery, radius, color, opaquity);
	// Don't draw anything if "given_angle" is -1, which means it didn't
	// find anything.
	if (mark_angle && given_angle >= 0) {
		// Okay, so find out where the target is. We already have
		// the radius, and we can get theta from given_angle (which is
		// in hexdegrees), making this a rather simple polar conversion.

		double destx = centerx + radius * cos(hex_to_radian(
					given_angle));
		double desty = centery + radius * sin(hex_to_radian(
					given_angle));

		// Draw the line
		target.line(centerx, centery, destx, desty, color, opaquity);
	}
}

// Draw a detailed bot stats box, without the surrounding border. Measurements
// are set according to the relative positions in ATR2, except for some
// adjustments to make the new font look acceptable.
// Bluesky: "INITIAL" which is true first time, false else. But that'd require
// that we only delete part of the box so that we don't overwrite the legends
// and info that stay the same.
void Widgets::large_botinfo(Display & target, const Color & bot_color, 
		const Color & dark_meter, const Color & no_error, 
		Font & typeface, string name, string message, 
		double armor_fraction, double heat_fraction, int wins,
		int kills, int deaths, bool has_error, signed short error) {

	// With the corner at 0,0 and max at 153,64, we get
	// 	Title starts at 3,7
	// 	Wins at 79, 7
	// 	"A:" at 10, 29
	// 	"H:" at 10, 39
	// 	"K:" at 10, 51
	// 	"D:" at 78, 51
	// 	"Error:" at 10, 61
	// 	"(None)" at 64, 61
	// 	Armor bar from 29, 24 to 128, 29
	// 	Heat bar from 29, 34 to 128, 39
	// 	Numbers aligned so that they start where the bars start
	// 	(for K) and end where the bars end (for D). Latter will be
	// 	hard for a proportional font.
	
	// Turning this into normalized ones:
	// 	Title starts at 0.0196, 0.109
	// 	Wins: 0.516, 0.109
	// 	A: 0.06535, 0.45312
	// 	H: 0.06535, 0.61
	// 	K: 0.06535, 0.80
	// 	D: 0.50980, 0.80
	// 	Error: 0.06535, 0.95
	// 	None: 0.47407, 0.95
	// 	Armor bar: 0.215, 0.375 to 0.95, 0.45312
	// 	Heat bar: 0.215, 0.53125 to 0.95, 0.60937
	
	// Also, message, K, and D should be in darker type.

	// Should the wins be padded to four digits? Hm.
	string winstr = itos(wins, 4);
	string diestr = itos(deaths, 4);
	string killstr = itos(kills, 4);
	string errstr = itos(error); // not padded
	string hexerr = uppercase(itos_hex((unsigned short)error, 4)); // padded error in unsigned short hex.

	Color dark_bot_color(bot_color.get_by_idx(0) * 0.66, 
			bot_color.get_by_idx(1) * 0.66, 
			bot_color.get_by_idx(2) * 0.66);

	// Perhaps get monospace?
	double vsz = 0.15;
	double barheight = 0.096;
	// Title
	target.print(0.02, 0, vsz, typeface, bot_color, 1.0, name);
	// Wins
	target.print(0.51, 0, vsz, typeface, bot_color, 1.0, "Wins: " + winstr);
	// Message
	target.print(0.02, 0.14, vsz, typeface, dark_bot_color, 1.0, message);
	// Designators
	target.print(0.065, 0.30, vsz, typeface, bot_color, 1.0, "A:");
	target.print(0.065, 0.46, vsz, typeface, bot_color, 1.0, "H:");
	target.print(0.065, 0.65, vsz, typeface, dark_bot_color, 1.0, "K: " + 
			killstr);
	target.print(0.51, 0.65, vsz, typeface, dark_bot_color, 1.0, "D:" + diestr);
	// Error
	target.print(0.065, 0.82, vsz, typeface, bot_color, 1.0, "Error:");
	if (has_error) {
		target.print(0.474, 0.82, vsz, typeface, bot_color, 1.0,
				errstr);
		target.print(0.80, 0.82, vsz, typeface, bot_color, 1.0,
				hexerr);
	} else {
		target.print(0.474, 0.82, vsz, typeface, no_error, 1.0, 
				"(No error)");
	}
	// Bars
	linear_meter(target, 0.215, 0.354, 0.95, 0.354 + barheight, true, 
			armor_fraction, bot_color, dark_meter);
	linear_meter(target, 0.215, 0.514, 0.95, 0.514 + barheight, true, 
			heat_fraction, bot_color, dark_meter);
}

// Interface to make it easier to render the bot stats.
void Widgets::large_botinfo(Display & target, const robot & source,
		const Color & dark_meter, const Color & no_error,
		Font & typeface) {
	large_botinfo(target, source.assigned_color, dark_meter, no_error,
			typeface, source.get_name(), source.get_message(),
			source.get_armor(), source.get_heat(), 
			source.get_all_victories(), source.get_all_kills(),
			source.get_all_deaths(), source.has_error(),
			source.get_last_error());
}

// Same, but smaller: only title wins, armor and health is visible.
void Widgets::small_botinfo(Display & target, const Color & bot_color,
		const Color & dark_meter, Font & typeface, 
		string name, double armor_fraction, double heat_fraction, 
		int wins) {

	// The box has dimensions 154x32, but to be consistent, 153x
	// With fonts at the lower left, we have
	// Title starts at 2,7
	// "Wins" starts at 78,7
	// "A:" starts at 10,17
	// "H:" starts at 10,27
	// Armor bar (upper left to lower right): 28,12 to 128,17
	// Health bar: 28,22 to 128,27.
	
	// Normalized: Dimensions: 0.24062 (or perhaps as above) x 0.06^
	// Title: 0.01307, 0.21875
	// Wins: 0.50980, 0.21875
	// A: 0.06535, 0.53125
	// H: 0.06535, 0.84375
	// Armor bar: 0.20740, 0.375 to 0.94814, 0.53125
	// But as it seems the fonts are referred by upper, we'll subtract .20.

	double vsz = 0.31;
	double barheight = 0.167;

	target.print(0.01, 0.01, vsz, typeface, bot_color, 1.0, name);
	string winstr = "Wins: " + itos(wins, 4);
	target.print(0.51, 0.01, vsz, typeface, bot_color, 1.0, winstr);
	target.print(0.065, 0.32, vsz, typeface, bot_color, 1.0, "A:");
	target.print(0.065, 0.63, vsz, typeface, bot_color, 1.0, "H:");

	 // Bars
	linear_meter(target, 0.207, 0.440, 0.95, 0.440 + barheight, true,
			armor_fraction, bot_color, dark_meter);
	linear_meter(target, 0.207, 0.750, 0.95, 0.750 + barheight, true,
			heat_fraction, bot_color, dark_meter);
}

void Widgets::small_botinfo(Display & target, const robot & source,
		const Color & dark_meter, Font & typeface) {

	small_botinfo(target, source.assigned_color, dark_meter, typeface, 
			source.get_name(), source.get_armor(), 
			source.get_heat(), source.get_all_victories());
}

// Round/match data, like what round we're in, how many rounds there are,
// etc. ATR2 used to have "free memory" here, too, but since we're running
// with large amounts of memory, there's no point in enumerating it, so we
// leave that spot for MatchID (random seed).

void Widgets::round_stats(Display & target, const Color & text_color,
		Font & typeface, const matchid_t matchid, 
		const int cur_cycle, const int max_cycle, const int cur_match, 
		const int max_match) {

	// The box is 154 x 44, but in order to match the others, we calc
	// as if it were 153 x ..
	
	// "FreeMem" (MatchID) at 2,9	0.01307, 0.20454.. 
	// 	and its numbers at 74,9	0.48366..
	// Cycle: at 2,19
	// Limit: at 2,29
	// Match: at 2,39
	
	// Cycle and Limit numbers are padded to have nine digits.
	
	double vsz = 0.21;
	target.print(0.013, 0.0, vsz, typeface, text_color, 1.0, "MatchID:");
	target.print(0.013, 0.25, vsz, typeface, text_color, 1.0, "Cycle:");
	target.print(0.013, 0.50, vsz, typeface, text_color, 1.0, "Limit:");
	target.print(0.013, 0.75, vsz, typeface, text_color, 1.0, "Match:");

	// Make strings out of the numbers.
	string matchid_str = lltos(matchid), cycle_str = itos(cur_cycle, 9),
	       limit_str = itos(max_cycle, 9);
	// Should this be + 1 or do we do that in the call?
	string match_str = itos(cur_match) + "/" + itos(max_match);

	// Now print the numbers.
	target.print(0.48, 0.00, vsz, typeface, text_color, 1.0, matchid_str);
	target.print(0.48, 0.25, vsz, typeface, text_color, 1.0, cycle_str);
	target.print(0.48, 0.50, vsz, typeface, text_color, 1.0, limit_str);
	target.print(0.48, 0.75, vsz, typeface, text_color, 1.0, match_str);
}

#endif
