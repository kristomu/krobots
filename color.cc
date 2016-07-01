#ifndef __COLOR
#define __COLOR

#ifndef NOSDL
#include <SDL/SDL.h>
#endif
#include <math.h>

class Color {
	private:
		double r, g, b;

		void HSV_to_RGB_internal(double h, double s, double v,
				double & R, double & G, double & B);

	public:
		Color();
		Color(double r_in, double g_in, double b_in);
		void set(double r_in, double g_in, double b_in);
		void set_hsv(double h_in, double s_in, double v_in);
#ifndef NOSDL
		Uint32 SDL_get(const SDL_PixelFormat * kind,
				double opaquity) const; // Leaky abstraction?
		SDL_Color SDLC_get(double opaquity) const;
#endif
		void mass_get(double & r_out, double & g_out, double & b_out) const;
		double get_by_idx(int index) const;

		Color & operator=(const Color & source);
};

// Transform HSV into RGB.
void Color::HSV_to_RGB_internal(double h, double s, double v, double & R,
		double & G, double & B) {

	// H, S, and V are given on [0, 1].
	// RGB are returned on [0,1].
	
	double m, n, f;
	int i;

	h = h * 6;

	if (s == 0) { R = v; G = v; B = v; return; }

	i = (int)floor(h);
	f = h - i;
	if ( !(i&1) ) f = 1 - f; // if i is even
	m = v * (1 - s);
	n = v * (1 - s * f);

	switch(i) {
		case 6:
		case 0: R = v; G = n; B = m; break;
		case 1: R = n; G = v; B = m; break;
		case 2: R = m; G = v; B = n; break;
		case 3: R = m; G = n; B = v; break;
		case 4: R = n; G = m; B = v; break;
		case 5: R = v; G = m; B = n; break;
	}
}


void Color::set(double r_in, double g_in, double b_in) {
	if (r_in < 0) r_in = 0;
	if (g_in < 0) g_in = 0;
	if (b_in < 0) b_in = 0;
	if (r_in > 1) r_in = 1;
	if (g_in > 1) g_in = 1;
	if (b_in > 1) b_in = 1;
	r = r_in; g = g_in; b = b_in;
}

void Color::set_hsv(double h_in, double s_in, double v_in) {
	double r, g, b;
	HSV_to_RGB_internal(h_in, s_in, v_in, r, g, b);
	set(r, g, b);
}

Color::Color(double r_in, double g_in, double b_in) {
	set(r_in, g_in, b_in);
}

Color::Color() {
	set(0, 0, 0); // Default to black
}

#ifndef NOSDL

// Doesn't work with SDL_gfx
Uint32 Color::SDL_get(const SDL_PixelFormat * kind,
		double opaquity) const {
	SDL_Color v = SDLC_get(opaquity);

	return(SDL_MapRGBA(kind, v.r, v.g, v.b, v.unused));
	/*			round(r * 255), round(g * 255), 
				round(b * 255), round(opaquity * 255)));*/
}

SDL_Color Color::SDLC_get(double opaquity) const {
	return (SDL_Color){round(r * 255), round(g * 255), round(b * 255),
		round(opaquity * 255)};
}

#endif

void Color::mass_get(double & r_out, double & g_out, double & b_out) const {
	r_out = r;
	g_out = g;
	b_out = b;
}

double Color::get_by_idx(int index) const {
	switch(index) {
		case 0: return(r);
		case 1: return(g);
		case 2: return(b);
		default: return(-1);
	}
}

Color & Color::operator=(const Color & source) {
	r = source.r;
	g = source.g;
	b = source.b;
	return(*this);
}

#endif
