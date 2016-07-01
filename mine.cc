// Mines
// These are really just stationary movers with a user-defined explosion radius.
// They can also be triggered manually (by opo 23, I think), so they need an IFF
// tag.

// It might be better to have detection radius be a reference to the robot, so
// that we can change the detection radius of all mines at once, but ATR2
// doesn't let us change them and they may be individualized, so probably not.

#ifndef _KROB_MINE
#define _KROB_MINE

#include "object.cc"

class mine : public Unit {

	private:
		int layer;	// Who laid this mine?
		double hit_at;
		bool s_explode;

	public:
		// Mines can't move, turn, or accelerate.
		mine(coordinate center, int laid_by, int detection_radius) :
			Unit(0, 0, 0, 0, 0, 0, 0, detection_radius, -laid_by,
					true) {
				set_pos(center);
				layer = laid_by; hit_at = -1; 
			s_explode = false;}

		void update_hit_time(double relative_time);
		int layer_UID() { return(layer); }
		double get_hit_time() { return(hit_at); }
		void explode();
		bool should_explode() { return(s_explode); }
};

void mine::update_hit_time(double relative_time) {
	hit_at = relative_time;
}

// This is a quite ugly hack. Basically, when triggering this function, it sets
// a flag that the mine should explode. Later, in collision detection, the mine
// checks the flag and if it's set, acts as if something collided with it, thus
// exploding. Better would be to have a separate "explode_mines/damage_all"
// function.
void mine::explode() {
	s_explode = true;
}

#endif
