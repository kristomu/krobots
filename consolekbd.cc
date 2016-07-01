#include <stdio.h>
#include <termios.h>
#include <unistd.h>

class ConsoleKeyboard {

	public:
		static ConsoleKeyboard & instantiate() {
			static ConsoleKeyboard theKbd;
			return(theKbd);
		}

		int getchar(); // unsigned char if any keypress, otherwise -1

		~ConsoleKeyboard();

	private:
		struct termios oldt, newt;

		ConsoleKeyboard();

};

ConsoleKeyboard::ConsoleKeyboard() {

	// Set non-canonical mode, so we can read keypresses directly.
	
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO | ECHONL |IEXTEN);
	newt.c_cc[VTIME] = 0;
	newt.c_cc[VMIN] = 0;
	tcsetattr( STDIN_FILENO, TCSANOW, &newt);
}

ConsoleKeyboard::~ConsoleKeyboard() {

	// Return to old mode.
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

int ConsoleKeyboard::getchar() {
	static char line[2];
	if (read (0, line, 1)) 
		return(line[0]);
	else	return(-1);
}
