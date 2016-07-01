// This class handles SDL (and SDL_ttf), initing upon construction and shutting down
// afterwards. Don't have more than one SDLHandler.

#ifndef __SDL_PROD
#define __SDL_PROD

#include <SDL/SDL.h>
#include <SDL/SDL_ttf.h>
#include <SDL/SDL_framerate.h>

#include <assert.h>

#include <iostream>
#include <string>

class SDLHandler {

	public:
		// Keep him from allocating more than one.
		static SDLHandler & instantiate(bool graphics, bool timing) {
			static SDLHandler theHandler(graphics, timing);
			return(theHandler);
		}

		// DONE: Made constructor private (operator = can't work with
		// private constructor, so no need to make that private.)
		SDLHandler(bool graphics, bool timing); // Init
		~SDLHandler(); // Free
	
		bool ready(); // Returns false if there was an error.
				// Bluesky: Create operator overloading so I can
				// do something equiv to iostream x; 
				// if (!x) then error.
		int get_next_event(); // -1 on error. Maybe get_this_event.
		int wait_next_event(); // Blocks until an event arrives.
		int get_last_event(); // For dividing up the checks.
		int get_event_keypress(); // If the event is a KEYUP or KEYDOWN,
					// get its parameters
		SDL_ResizeEvent get_event_resize();


		void set_title(std::string title, std::string minimized_title);
		void set_framerate(int fps);
		bool wait_for_frame_refresh();

	private:
		FPSmanager FPS_man; // for synchronizing display output
		SDL_Event events;

		int last_event;
		bool inited;
		bool has_produced_real;
};

SDLHandler::~SDLHandler() {
	SDL_Quit();
	TTF_Quit();
}

SDLHandler::SDLHandler(bool graphics, bool timing) {
	// Something here about assertions.
	int mask = 0;
	if (graphics) mask |= SDL_INIT_VIDEO;
	if (timing) mask |= SDL_INIT_TIMER;

	if (SDL_Init(mask) < 0)
		inited = false;
	else	inited = true;

	inited = ((TTF_Init() != -1) && inited);
	SDL_initFramerate(&FPS_man);

	last_event = -1;
}

bool SDLHandler::ready() { return(inited); }

int SDLHandler::get_next_event() {
	if (!inited) return(-1);

	//int ret = SDL_PollEvent(&events);

	// Are there no new events?
	if (SDL_PollEvent(NULL) == 0) 
		return(-1); // No, let them know
	else	SDL_PollEvent(&events); // load into Events

	// Otherwise, load the event.
	last_event = events.type;
	return(last_event);
}

int SDLHandler::wait_next_event() {
	if (!inited) return(-1);

	int ret = SDL_WaitEvent(&events);

	if (ret == 0)
		return(-1);

	last_event = events.type;
	return(last_event);
}
	
int SDLHandler::get_last_event() { return(last_event); }

int SDLHandler::get_event_keypress() {
	if (last_event != SDL_KEYDOWN && last_event != SDL_KEYUP) return(-1);
	return(events.key.keysym.sym);
}

// Undefined if it's not SDL_VIDEORESIZE
SDL_ResizeEvent SDLHandler::get_event_resize() {
	return(events.resize);
}

void SDLHandler::set_title(std::string title, std::string minimized_title) {
	SDL_WM_SetCaption(title.c_str(), minimized_title.c_str());
}

void SDLHandler::set_framerate(int fps) {
	assert (fps > 0);
	SDL_setFramerate(&FPS_man, fps);
}

bool SDLHandler::wait_for_frame_refresh() {
	if (!inited) return(false);
	SDL_framerateDelay(&FPS_man);
	return(true);
}


#endif
