
// This class simulates a font where we can write different sized characters on
// the fly. It may be slow and use disk space, but it's a useful abstraction.

#ifndef __SDL_FONT
#define __SDL_FONT

#include <SDL/SDL_ttf.h>
#include <iostream>
#include <assert.h>
#include <string>

using namespace std;

class Font {

	private:
		string filename;
		TTF_Font * cached_font;
		int last_size;
		void free_current_font();
		bool load_current_font(int size);

	public:
		Font();
		Font(string ttf_in);
		~Font();
		bool load_new_font(string ttf_in);
		string get_filename() { return(filename); }
		TTF_Font * get_font();
		TTF_Font * get_font(int point_size);
};

void Font::free_current_font() {
	if (cached_font != NULL)
		TTF_CloseFont(cached_font);
	last_size = -1;
}

bool Font::load_current_font(int size) {
	// Don't do anything if it's loaded and the size fits
	if (cached_font != NULL && size == last_size) return(true);
	if (size < 0) size = 16;
	free_current_font();
	//cout << "Size man " << size << endl;
	cached_font = TTF_OpenFont(filename.c_str(), size);
	if (cached_font != NULL)
		last_size = size;
	else	last_size = -1;

	return(cached_font != NULL);
}

Font::Font() {
	last_size = -1; // denotes that nothing has been loaded into
			// cached_font.
	filename = "";
	cached_font = NULL;
}

Font::~Font() { free_current_font(); }

bool Font::load_new_font(string ttf_in) {
	free_current_font();
	filename = ttf_in;
	return(load_current_font(-1));
}

Font::Font(string ttf_in) {
	last_size = -1;
	cached_font = NULL;
	// If we specify the font in the constructor, it better well be there.
	assert(load_new_font(ttf_in));
}

TTF_Font * Font::get_font(int point_size) {
	// If there's nothing there or we have a size mismatch, check if we can
	// physically load.
	// Default size is 16.
	if (point_size == -1) point_size = 16;
	if (last_size == -1 || cached_font == NULL || point_size != last_size) 
		load_current_font(point_size);
	// If there's still nothing, then bail out.
	if (last_size == -1 || cached_font == NULL) return(NULL);

	// Otherwise return our value
	return(cached_font);
}

TTF_Font * Font::get_font() {
	return(get_font(last_size));
}

#endif
