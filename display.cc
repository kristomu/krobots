// Display for ATR2 reimplementation.
// The display consists of a main surface (where all the action happens), and
// an auxiliary surface (which displays stats and the like). This necessitates
// a class (Display) that is either virtual or real. Then we write to the
// virtual surfaces and blit the entire thing to the real surface at the end
// of each cycle.

// All references are floating point since we're not doing any bitmap blitting,
// and thus the display is resolution independent.

#ifndef SDL_DISP
#define SDL_DISP

#include "coordinate.cc"
#include "color.cc" 
#include "font.cc"

#include <SDL/SDL.h>
#include <SDL/SDL_gfxPrimitives.h>
#include <SDL/SDL_ttf.h>

// SDL_LockSurface? circles may be faster on software surfaces

// DONE: Optimization with ridiculous radius parameters for circle, arc, etc,
// and rectangles where part of the surface is outside of the surface. Instead
// of drawing, this aborts the call, returning with a false value.

class Display {
	private:
		SDL_Surface * surface;
		int xsize, ysize, alloc_depth;
		double aspect; // So resize isn't corrupted by rounding errs.
		bool is_real;
		bool inited_properly;

		void construct_masks(Uint32 & rmask, Uint32 & gmask, 
				Uint32 & bmask, Uint32 & amask);

		SDL_Rect const_area(int x1, int y1, int x2, int y2) const;
		SDL_Rect translated_area(double x1, double y1, double x2,
				double y2) const;
		bool valid(double float_coord) { 
			return(float_coord >= 0 && float_coord <= 1); }

		Uint32 getpixel_p(int x, int y);

		// Simple cut and paste of SDL_gfx, only with double so we
		// can handle fractional angles.
		int doRPieColor(SDL_Surface * dst, Sint16 x, Sint16 y, double
				radius, double start, double end, Uint32 color,
				Uint8 filled);

	protected:
		// Used for copying between two Displays.
		void get_copy_info(SDL_Surface * & pointer, int & xsize_here,
				int & ysize_here);
		int blit_onto(SDL_Surface * destination, SDL_Rect *
				area) const;

	public:
		void allocate(bool real, int depth);
		Display(bool real, int xsize_in, int ysize_in, int depth);
		Display(const Display & source);
		~Display();

		bool resize(int new_xsize, int new_ysize);
		bool resize(int new_xsize); // preserve aspect ratio.

		// because copy constructor isn't called here.
		const Display & operator=(const Display & source);

		bool ready() const; // See SDLProducer.
		
		int get_xsize() const { return(xsize); }
		int get_ysize() const { return(ysize); }

		coordinate get_size() const { return(coordinate(xsize, ysize));}

		int get_depth() const { return(alloc_depth); }

		// For stretching polar coordinates so they fit with circle
		void stretch_to_aspect(double & x, double & y);

		// Geometric primitives
		Uint32 getpixel(double xpos, double ypos);
		bool circle(double xpos, double ypos, double radius,
				const Color & color, double opaquity);
		bool rect(double xmin, double ymin, double xmax, double ymax,
				const Color & color, double opaquity);
		bool line(double xmin, double ymin, double xmax, double ymax,
				const Color & color, double opaquity);
		bool triangle(double x1, double y1, double x2, double y2,
				double x3, double y3, const Color & outer_color,
				const Color & inner_color, bool filled,
				double outer_opaquity, double inner_opaquity);
		bool triangle(double x1, double y1, double x2, double y2,
				double x3, double y3, const Color & line_color,
				double opaquity);
		// For these, the angles are in degrees
		bool pie(bool filled, double xpos, double ypos, double radius,
				double start_angle, double end_angle,
				const Color & color, double opaquity);

		// .. and other drawing actions
		// Maybe not really optimal, but Polishing Comes Later.
		bool print(double xpos, double ypos, double rel_size,
				Font & typeface, const Color & color,
				double opaquity, string text);
		bool box(double xmin, double ymin, double xmax, double ymax,
				const Color & color, double opaquity);
		bool clear(const Color & color);
		bool clear();
		int blit(const Display & from, double xmin, double ymin,
				double xmax, double ymax);
		int blit_all(const Display & from); 

		// Does nothing if the display isn't real.
		void render(); 
		bool lock();
		bool unlock();
		bool is_display_real() const { return(is_real); }

		// Copy from source to the box defined by (xstart, ystart) -
		// (xend, yend).
		void acquire_from(Display & source, int xstart, int ystart,
				int xend, int yend);
		SDL_Surface * expose_surface(); // have to decide if this is
				// proper. Probably not. Something about
				// friends?
};

void Display::construct_masks(Uint32 & rmask, Uint32 & gmask, Uint32 & bmask,
		Uint32 & amask) {
	// Kludgery required because SDL isn't endian neutral.
	// This function returns the red, green, blue, and alpha masks.
	
	if (SDL_BYTEORDER == SDL_BIG_ENDIAN) {
		rmask = 0xFF000000;
		gmask = 0x00FF0000;
		bmask = 0x0000FF00;
		amask = 0x000000FF;
	} else {
		rmask = 0x000000FF;
		gmask = 0x0000FF00;
		bmask = 0x00FF0000;
		amask = 0xFF000000;
	}
}

// Construct SDL_Rect structures -- less cumbersome than doing it manually
// every time.
SDL_Rect Display::const_area(int x1, int y1, int x2, int y2) const {
	if (x1 > x2) return(const_area(x2, y1, x1, y2));
	if (y1 > y2) return(const_area(x1, y2, x2, y1));

	SDL_Rect toRet;
	toRet.x = x1;
	toRet.y = y1;
	toRet.w = x2 - x1;
	toRet.h = y2 - y1;

	return(toRet);
}

SDL_Rect Display::translated_area(double x1, double y1, double x2,
		double y2) const {
	return(const_area(round(x1 * xsize), round(y1 * ysize),
				round(x2 * xsize), round(y2 * ysize)));
}


// Assumes SDL has been inited. Pass a fake class around for that or just
// assume the programmer will do right?
// Probably class - that way it can call SDL_Quit on deinit too.

void Display::allocate(bool real, int depth) {

	alloc_depth = depth;

	if (real)
		surface = SDL_SetVideoMode(get_xsize(), get_ysize(), depth,
				SDL_HWSURFACE | SDL_RESIZABLE | SDL_DOUBLEBUF);
	else {
		Uint32 rmask, gmask, bmask, amask;
		construct_masks(rmask, gmask, bmask, amask);

		surface = SDL_CreateRGBSurface(SDL_HWSURFACE, get_xsize(),
				get_ysize(), depth, rmask, gmask, bmask,
				amask);
	}
}

bool Display::resize(int new_xsize, int new_ysize) {
	if (new_xsize <= 0 || new_ysize <= 0) return(false);

	xsize = new_xsize;
	ysize = new_ysize;
	aspect = xsize/(double)ysize;

	// It's only necessary to free first if it's not a real display.
	if (!is_display_real())
		if (inited_properly)
			SDL_FreeSurface(surface);

	allocate(is_display_real(), alloc_depth);

	inited_properly = (surface != NULL);

	clear();

	return(true);
}

bool Display::resize(int new_xsize) { // preserve aspect ratio.
	double retain_aspect = aspect;
	resize(new_xsize, round(new_xsize / retain_aspect));
	aspect = retain_aspect;
}


Display::Display(bool real, int xsize_in, int ysize_in, int depth) {
	// DONE: Real checks here, what if < 0? or insane size? Uint?
	assert (xsize_in >= 0 && ysize_in >= 0);
	xsize = xsize_in;
	ysize = ysize_in;
	aspect = xsize/(double)ysize;
	is_real = real;

	allocate(is_real, depth);

	inited_properly = (surface != NULL);
	assert(inited_properly);
}

Display::~Display() {
	// If we have inited a surface, remove it.
	if (inited_properly)
		SDL_FreeSurface(surface);
}

Display::Display(const Display & source) {
	// Copy constructor (deep copy).
	// We could cheat this by using reference counting, but bah.
	// Also, I'm not sure if it's possible to have more than one real
	// surface.
	xsize = source.get_xsize();
	ysize = source.get_ysize();
	is_real = source.is_display_real();

	allocate(is_real, source.get_depth());

	inited_properly = (surface != NULL);
	if (inited_properly) {
		clear();			// Set alpha to 1
		blit_all(source);
	}
}

const Display & Display::operator=(const Display & source) {
	if (&source == this) return(*this);

	// Deinit then init. Bluesky would be to check size: if we're larger
	// than the source, just blit it over.
	
	if (inited_properly)
		SDL_FreeSurface(surface);

	xsize = source.get_xsize();
	ysize = source.get_ysize();
	is_real = source.is_display_real();

	allocate(is_real, source.get_depth());

	inited_properly = (surface != NULL);
	if (inited_properly) {
		clear();		// Set alpha to 1
		blit_all(source);
	}

	return(*this);
}

bool Display::ready() const { return(inited_properly); }

// Maybe const?
void Display::get_copy_info(SDL_Surface * & pointer, int & xsize_here,
		int & ysize_here) {
	xsize_here = xsize;
	ysize_here = ysize;
	pointer = surface;
}

int Display::blit_onto(SDL_Surface * destination, SDL_Rect * area) const {
	if (destination == NULL) return(false);

	// Be aware of a return value of -2 which means everything was lost.
	return(SDL_BlitSurface(surface, NULL, destination, area));
}


// Mapper's problem - I don't see how this is required. All I see is that
// drawing circle natively creates a round circle whereas drawing it in
// straightforward polar->cartesian transformation doesn't. I think the
// problem will show itself the other way if I had an unusual aspect ratio
// at full screen.
// (Consider the case where ysize = 1 and xsize = 600. The "uncorrected" circle
//  will draw something that's round in this projection; so it's actually
//  circle and not line/etc that should be corrected.)
void Display::stretch_to_aspect(double & x, double & y) {
	y *= (xsize / (double)ysize);
}

// Stolen from SDL docs
Uint32 Display::getpixel_p(int x, int y) {
	// DONE: Error checking.
	assert (x >= 0 && x < xsize && y >= 0 && y < ysize);

	// Get bits per pixel
	int bpp = surface->format->BytesPerPixel;

	// Calculate address of the point to peek at.
	Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;

	// Now return a value that makes sense, given the bpp configuration.
	
	switch(bpp) {
		case 1:
			return *p;
		case 2:
			return *(Uint16 *)p;
		case 3:
			if(SDL_BYTEORDER == SDL_BIG_ENDIAN)
				return p[0] << 16 | p[1] << 8 | p[2];
			else	return p[0] | p[1] << 8 | p[2] << 16;
		case 4:
			return *(Uint32 *)p;

		default:
			return 0; // Shouldn't happen
	}
}

Uint32 Display::getpixel(double xpos, double ypos) {
	return(getpixel_p((int)round(xpos * xsize), (int)round(ypos * ysize)));
}

// Usage as in SDL_gfx. Don't mind that I'm compressing LOCs as much as
// possible; go to SDL_gfxPrimitives.c if you want comments.
int Display::doRPieColor(SDL_Surface * dst, Sint16 x, Sint16 y, double
		radius, double start, double end, Uint32 color,
		Uint8 filled) {

	Sint16 left, right, top, bottom, x1, y1, x2, y2;
	int result;
	double angle, start_angle, end_angle, deltaAngle, dr;
	int posX, posY;
	int numpoints, i;
	Sint16 * vx, * vy;

	if ((dst->clip_rect.w==0) || (dst->clip_rect.h==0))
		return(0);

	if (radius < 0)
		return(-1);

	if (radius < 1)
		return(pixelColor(dst, x, y, color));

	start = fmod(start, 360);
	end = fmod(end, 360);

	// Clipping
	x2 = round(x + radius);
	left = dst->clip_rect.x;
	if (x2 < left) return(0);
	x1 = round(x - radius);
	right = dst->clip_rect.x + dst->clip_rect.w - 1;
	if (x1 > right) return(0);
	y2 = round(y + radius);
	top = dst->clip_rect.y;
	if (y2 < top) return(0);
	y1 = round(y - radius);
	bottom = dst->clip_rect.y + dst->clip_rect.h - 1;
	if (y1 > bottom) return(0);

	// Variable setup
	dr = radius;
	deltaAngle = 3.0 / dr;
	start_angle = start * (2.0 * M_PI / 360.0);
	end_angle = end * (2.0 * M_PI / 360.0);
	if (start > end)
		end_angle += (2.0 * M_PI);

	numpoints = 1;
	angle = start_angle;
	while (angle <= end_angle) {
		angle += deltaAngle;
		++numpoints;
	}

	if (numpoints == 1)
		return(pixelColor(dst, x, y, color));
	else if (numpoints == 2) {
		posX = x + (int)rint(dr * cos(start_angle));
		posY = y + (int)rint(dr * sin(start_angle));
		return(lineColor(dst, x, y, posX, posY, color));
	}

	// Allocate. I would use vector<Coordinate> here, but my hands
	// are tied by Polygon.
	vx = vy = (Sint16 *) malloc(2 * sizeof(Uint16) * numpoints);
	if (vx == NULL) return(-1);
	vy += numpoints;
	vx[0] = x;
	vy[0] = y;

	i = 1;
	angle = start_angle;
	while (angle <= end_angle) {
		vx[i] = x + (int) rint(dr * cos(angle));
		vy[i] = y + (int) rint(dr * sin(angle));
		angle += deltaAngle;
		++i;
	}

	if (filled)
		result = filledPolygonColor(dst, vx, vy, numpoints, color);
	else	result = polygonColor(dst, vx, vy, numpoints, color);

	free(vx);

	return(result);
}


// No need for aaCircle as they're hard to erase and don't take fractional
// coordinates anyway.
// Radius is given in terms of the x length so as to preserve the circularity
// in non-square pixel conditions.
bool Display::circle(double xpos, double ypos, double radius, 
		const Color & color, double opaquity) {

	if (!ready() || !valid(xpos) || !valid(ypos)) return(false);

	// Maybe RGBA would be better. At least it would be device independent.
	// Something like..
	
	SDL_Color tc = color.SDLC_get(opaquity);

	circleRGBA(surface, round(xpos * xsize), round(ypos * ysize), 
			round(radius * xsize), 
			tc.r, tc.g, tc.b, tc.unused);

	return(true);
}

bool Display::rect(double xmin, double ymin, double xmax, double ymax,
		const Color & color, double opaquity) {

	// Normalize
	xmin = max(0.0, min(xmin, 1.0));
	ymin = max(0.0, min(ymin, 1.0));
	xmax = max(0.0, min(xmax, 1.0));
	ymax = max(0.0, min(ymax, 1.0));

	if (!ready() || !valid(xmin) || !valid(ymin) || !valid(xmax) || 
			!valid(ymax)) return(false);

	SDL_Color tc = color.SDLC_get(opaquity);

	rectangleRGBA(surface, round(xmin * xsize), round(ymin * ysize), 
			round(xmax * xsize), round(ymax * ysize),
			tc.r, tc.g, tc.b, tc.unused);

	return(true);

}

bool Display::line(double xmin, double ymin, double xmax, double ymax,
		const Color & color, double opaquity) {

	// If at least one value is wrong at min or max, bail out.
	// The ideal would be to truncate based on slope.
	if ( (!valid(ymin) || !valid(xmin)) && (!valid(ymax) || 
				!valid(xmax))) return(false);
	if (!ready()) return(false);

	SDL_Color tc = color.SDLC_get(opaquity);

	lineRGBA(surface, round(xmin * xsize), round(ymin * ysize),
			round(xmax * xsize), round(ymax * ysize),
			tc.r, tc.g, tc.b, tc.unused);

	return(true);
}

bool Display::triangle(double x1, double y1, double x2, double y2,
		double x3, double y3, const Color & outer_color,
		const Color & inner_color, bool filled, double outer_opaquity,
		double inner_opaquity) {

	if (!ready()) return(false);

	if (!valid(x1) || !valid(y1) || !valid(x2) || !valid(y2) ||
			!valid(x3) || !valid(y3))
		return(false);

	SDL_Color outer = outer_color.SDLC_get(outer_opaquity),
		  inner = inner_color.SDLC_get(inner_opaquity);

	// Draw the inner triangle.
	if (filled)
		filledTrigonRGBA(surface, round(x1 * xsize), round(y1 * ysize),
				round(x2 * xsize), round(y2 * ysize),
				round(x3 * xsize), round(y3 * ysize),
				inner.r, inner.g, inner.b, inner.unused);

	// Draw the outer triangle.
	trigonRGBA(surface, round(x1 * xsize), round(y1 * ysize), round(x2 *
				xsize), round(y2 * ysize), round(x3 *
				xsize), round(y3 * ysize), outer.r, outer.g,
			outer.b, outer.unused);

	return(true);
}

bool Display::triangle(double x1, double y1, double x2, double y2, double x3, 
		double y3, const Color & line_color, double opaquity) {

	return(triangle(x1, y1, x2, y2, x3, y3, line_color, Color(0, 0, 0),
				false, opaquity, 0));
}


bool Display::pie(bool filled, double xpos, double ypos, double radius, 
		double start_angle, double end_angle, const Color & color, 
		double opaquity) {

	if (!ready() || !valid(xpos) || !valid(ypos)) return(false);

	SDL_Color tc = color.SDLC_get(opaquity);

	// Limited to 32768x32768.
	// Now handles fractional angles, at the cost of some code duplication.

	Uint32 c_color = ((Uint32) tc.r << 24) | ((Uint32) tc.g << 16) | 
		((Uint32) tc.b << 8) | (Uint32)tc.unused;

	if (filled)
		doRPieColor(surface, round(xpos * xsize), round(ypos * ysize), 
				radius * xsize, start_angle, end_angle,
				c_color, 1);
	else
		doRPieColor(surface, round(xpos * xsize), round(ypos * ysize),
				radius * xsize, start_angle, end_angle,
				c_color, 0);

	return(true);
}

bool Display::print(double xpos, double ypos, double rel_size,
		Font & typeface, const Color & color, double opaquity, 
		string text) {

	if (surface == NULL) return(false);
	if (!valid(xpos) || !valid(ypos)) return(false);

	SDL_Color clrFg = {round(color.get_by_idx(0) * 255),
		round(color.get_by_idx(1) * 255), 
		round(color.get_by_idx(2) * 255),
		round(opaquity * 255)};

	SDL_Rect rect;
	rect.x = round(xpos * xsize);
	rect.y = round(ypos * ysize);

	int abs_sz = round(rel_size*ysize);

	SDL_Surface * text_temp = TTF_RenderText_Blended(
			typeface.get_font(abs_sz), text.c_str(), clrFg);

	if (text_temp == NULL) return(false);

	SDL_BlitSurface(text_temp, NULL, surface, &rect);
	SDL_FreeSurface(text_temp);

	return(true);
}

bool Display::box(double xmin, double ymin, double xmax, double ymax,
		const Color & color, double opaquity) {
	assert (xmin >= 0 && xmin <= 1 && ymin >= 0 && ymin <= 1);
	if (surface == NULL) return(false);
	
	SDL_Rect area = translated_area(xmin, ymin, xmax, ymax);

	SDL_Color clrFg = {round(color.get_by_idx(0) * 255),
		round(color.get_by_idx(1) * 255),
		round(color.get_by_idx(2) * 255),
		round(opaquity * 255)};

	// What an odd name for the alpha channel!

	return (SDL_FillRect(surface, &area, SDL_MapRGBA(surface->format, 
					clrFg.r, clrFg.g,clrFg.b, 
					clrFg.unused)) != -1);
}

bool Display::clear(const Color & color) {
	if (surface == NULL) return(false);
	// Clear the entire surface
	SDL_Rect area = translated_area(0, 0, 1, 1);

	// Fully opaque
	SDL_Color clrFg = {round(color.get_by_idx(0) * 255),
		round(color.get_by_idx(1) * 255),
		round(color.get_by_idx(2) * 255), 255};

	return(SDL_FillRect(surface, &area, SDL_MapRGBA(surface->format,
					clrFg.r, clrFg.g,clrFg.b, 
					clrFg.unused)) != -1);
}

bool Display::clear() {
	Color black(0, 0, 0);
	return(clear(black));
}

int Display::blit(const Display & from, double xmin, double ymin, double xmax,
		double ymax) {

	// Basic sanity checks
	if (surface == NULL) return(false);
	if (!from.ready()) return(false);

	SDL_Rect area = translated_area(xmin, ymin, xmax, ymax);

	return(from.blit_onto(surface, &area));
}

int Display::blit_all(const Display & from) {
	// To be aware of: Blit_onto/Blit doesn't narrow down the rectangle
	// if it's too large for the destination. This may cause slowdowns.
	return(blit(from, 0, 0, 1, 1));
}

void Display::render() {
	if (is_real && ready())
		SDL_Flip(surface);
}

bool Display::lock() {
	if (ready()) {
		SDL_LockSurface(surface);
		return(true);
	} else	return(false);
}

bool Display::unlock() {
	if (ready()) {
		SDL_UnlockSurface(surface);
		return(true);
	} else	return(false);
}

#endif
