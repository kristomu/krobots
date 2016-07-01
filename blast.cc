// Blast effects upon missile (and later, mine) explosions.

#ifndef _KROB_BLAST
#define _KROB_BLAST

#include "color.cc"
#include "coordinate.cc"
#ifndef NOSDL
#include "display.cc"
#endif
#include <list>
#include <vector>

using namespace std;

typedef enum {B_MINE = 0, B_MISSILE = 1, B_ROBOT = 2, B_ALL = 3} blast_type;

typedef struct blast {
	coordinate impact_point;
	blast_type kind;
	double maxradius; // Maximum radius of blast circle. Should be equal
			  // to blast radius.
	double start_time;// When the explosion happened.
	double maxtime;	  // How many seconds we have to display the animation.
};

// The "blasts" class handle the introduction of new blasts, the rendering of
// old, and the removal of those who did time out.
class blasts {

	private:
		list<blast> ongoing_explosions;
		// With which colors should we draw the various blasts?
		vector<Color> associated_colors;
		// What's the absolute time at this moment?
		double present_time;
		double duration;

		Color get_color(blast_type type) const;
#ifndef NOSDL
		void draw_blast(const coordinate base, Display & target, 
				list<blast>::const_iterator & pos,
				coordinate arena_size) const;
#endif

	public:
		void reinit();
		blasts() { reinit(); }
		blasts(double time) { reinit(); present_time = time; }
		blasts(double time, double duration);
		void set_color(blast_type type, const Color & assoc_color);
		void set_standard_duration(double dur_in) { duration = dur_in; }
		void update_time(double new_time) { present_time = new_time; }

		void add_blast(coordinate location, blast_type type, double
				maxradius);
		void prune_blasts(); // remove those past animation.
		void update_all(double new_time);
#ifndef NOSDL
		void draw_blasts(const coordinate base, Display & target, 
				coordinate arena_size) const;
#endif
};

// Draws an explosion/impact effect. The effect starts with a circle at radius
// 0 (i.e, a point), at time of impact, then expands outwards until it reaches
// maxradius at time "maxtime". Circles will be distorted if the aspect ratio
// is incorrect.

Color blasts::get_color(blast_type type) const {
	// If it's an unreasonable request, give an unreasonable mauve color.
	//cout << "Blast: " << (int)type << endl;
	assert ((int) type >= 0 && (int) type < B_ALL);
	if ((int)type < 0 || (int)type >= B_ALL) return(Color(1, 0, 1));

	return(associated_colors[(int)type]);
}

void blasts::reinit() {
	present_time = 0;
	duration = 1;
	associated_colors.resize((int)B_ALL, Color(0, 1, 0));
}
	
blasts::blasts(double time, double duration_in) {
	reinit();
	present_time = time;
	duration = duration_in;
}

// Base is used for displacing where we actually draw for the purposes of making
// the edges of the arena less crowded.
#ifndef NOSDL
void blasts::draw_blast(const coordinate base, Display & target, 
		list<blast>::const_iterator & cur, coordinate arena_size) const{

	// First find out how far we're into the animation, and abort if we're
	// past 1 (shouldn't happen).
	double animprogress = (present_time - cur->start_time)/(cur->maxtime -
			cur->start_time);

	if (animprogress > 1) return;

	// Okay, so normalize the coordinates and tell the display to draw it.
	
	coordinate normalized = (base + cur->impact_point) / arena_size;
	// Why not xsize?
	double norm_radius = cur->maxradius / (double)arena_size.y;

	target.circle(normalized.x, normalized.y, norm_radius * animprogress,
			get_color(cur->kind), 1.0);

}
#endif

void blasts::set_color(blast_type type, const Color & assoc_color) {
	if ((int)type < 0 || (int)type >= B_ALL) return;

	associated_colors[(int)type] = assoc_color;
}

void blasts::add_blast(coordinate location, blast_type type, double
		maxradius) {
	// Add the blast structure parameters.
	blast new_blast;
	new_blast.impact_point = location;
	new_blast.maxradius = maxradius;
	new_blast.start_time = present_time;
	new_blast.kind = type;
	new_blast.maxtime = new_blast.start_time + duration;

	// And add
	ongoing_explosions.push_back(new_blast);
}

void blasts::prune_blasts() {

	list<blast>::iterator pos = ongoing_explosions.begin();

	while (pos != ongoing_explosions.end())
		if (pos->maxtime < present_time)
			pos = ongoing_explosions.erase(pos);
		else	++pos;
}

void blasts::update_all(double new_time) {
	// First set the new time.
	update_time(new_time);
	// Then remove those who have now expired.
	prune_blasts();
}

#ifndef NOSDL
// This is the interface to the private draw_blast. We reason there's no point
// in drawing less than all the blasts, so this goes through the entire list.
void blasts::draw_blasts(const coordinate base, Display & target, 
		coordinate arena_size) const {

	for (list<blast>::const_iterator pos = ongoing_explosions.begin();
			pos != ongoing_explosions.end(); ++pos)
		draw_blast(base, target, pos, arena_size);
}
#endif
#endif
