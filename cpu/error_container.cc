
// Compiler error info: includes error type and exactly which token tripped the
// error. We use this to give an error message to the user if he tries to input
// incorrect code.
// It's very simple - we don't need setters and getters here. Yet, at least.

#ifndef _KROB_ERRCONT
#define _KROB_ERRCONT

#include "errors.h"
#include "../tools.cc"
#include <string>

using namespace std;

class error_container {
	public:
		compile_error error;
		string token;
		int constraint;

		error_container(string token_in, int constraint_in,
				compile_error error_in);
		error_container(compile_error error_in);

		string construct_error_message();
};

error_container::error_container(string token_in, int constraint_in, 
		compile_error error_in) {
	constraint = constraint_in;
	token = token_in; error = error_in;
}

error_container::error_container(compile_error error_in) {
	token = ""; error = error_in;
	constraint = -1;
}

string error_container::construct_error_message() {
	switch(error) {
		case CER_INVALID_NLABEL: return ("Invalid :label ("+token+")");
		case CER_INVALID_REF:	return("Invalid identifier (" + token +
					"). Check typing.");
		case CER_WRONG_AXX: 
			return("Memory access out of range (" + token +")");
		case CER_TOO_FEW:	return("Not enough robots for combat.");
		case CER_EMPTY_ARENA:	
			return((string)"Robot names and settings must be "+
					"specified. An empty arena is no fun.");
		case CER_CONF_NOT_FOUND:
			return("Config file not found (" + token + ")");
		case CER_NESTED_CONF:	
			return((string)"Cannot access a config file from a "
					+ "config file (" + token + ")");
		case CER_NOFOUND:
			return("Robot not found (" + token + "). Perhaps you " +
					"mistyped it?");
		case CER_TOO_CROWDED:
			return("Too many robots! We can only handle " + 
					itos(constraint) + " at maximum!");
		case CER_DUPLICATE_DEF:
			return("User variable " + token + " already defined" +
					" by #def.");
		case CER_LONG_VARIABLE:
			return("User variable name (" + token + ") is too long!"
					+ " Max length is " + itos(constraint));
		case CER_XLBL_COLLIDE:
			return("!Label ( " + token + ") already defined!");
		case CER_TOO_MANY_VARS:
			return("Too many variables! (" + token + ") broke the" +
					" camel's back of max " + 
					itos(constraint) + ".");
		case CER_TOO_MANY_XLBL:
			return("Too many !labels (" + token + ") broke the " +
					"camel's back of max " +
					itos(constraint) + ".");
		case CER_LONG_PROGRAM:
			return("Robot program is too long! Program is " + 
					token + " effective lines, vs maximum"+
					" of " + itos(constraint) + ".");
		case CER_MISSING_XLBL:
			return("!Label missing error: !Label " + token);
		case CER_XLBL_OORANGE:
			return("!Label out of range: " + token);
		case CER_XLBL_NOTFOUND:
			return("!Label not found. (" + token + ")");
		case CER_INVALID_DEV:
			return("Invalid device #config option (" + token + 
					").");
		case CER_INV_DEV_ASN:
			return("Invalid #config assignment value (" + token +
					").");
		case CER_CHEATER:
			return((string)"Robot is attempting to cheat: Too " +
					"many config points (" + token + "/" +
						itos(constraint) +").");
		case CER_MICRO_SCARCE:
			return("Insufficient data in data statement: " + token);
		case CER_MICRO_DENSE:	return("Too many asterisks: " + token);
		case CER_MICRO_NONINT:	
			return("Microcode is non-integer: " + token);
		case CER_MICRO_NONMICRO: // Shouldn't happen
			return((string)"Microcode parsing requested, but " + 
					"isn't microcode: " + token);
		case CER_NOACCESS:
			return("Cannot access robot file " + token);
		case CER_OOBOUNDS:
			return((string)"Number too large - cannot be " + 
					"referenced: " + token);
		case CER_NLABEL_OORANGE:
			return(":Label out of range: " + token);
		case CER_F_AMBIGUOUS:
			return("Ambiguous file name stub \"" + token + "\", " +
					"found " + itos(constraint) + 
					" files with this name and differing " +
					"extensions.");
		case CER_LOCKED:
			return("Robot is locked, which is not supported.");

		default: return("Unknown compile error! ( " + token + ")");
	}
}

#endif
