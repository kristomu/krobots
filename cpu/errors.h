#ifndef _KROB_ERROR
#define _KROB_ERROR

// DONE: Make into .cc, and then make "error_divulger" class that returns the
// commented error strings. Even better would be to pass the error refs around
// directly (so that we can say ":@AX No such identifier" instead of "No such
// identifier at line 3").

// Run-time errors
typedef enum {
	ERR_NOT_IMPLEMENTED = -1, // Placeholder
	ERR_NOERR = 0,
	ERR_STACK_FULL = 1, 	// Stack full - Too many CALLs?
	ERR_NOLABEL = 2,	// Label not found. Hmmm.
	ERR_READONLY = 3,	// Can't assign value - Tisk tisk.
	ERR_CANT_REF_MEM = 4,	// Illegal memory reference.
	ERR_STACK_EMPTY = 5,	// Stack empty - Too many RETs?
	ERR_UNKN_CMD = 6,	// Illegal instruction. How bizarre.
	ERR_OUT_OF_RANGE = 7,	// Return out of range - Woops!
	ERR_DIVIDE_ZERO = 8,	// Divide by zero
	ERR_UNRESOLVED_TXT = 9,	// Unresolved !label. WTF?
	ERR_INVALID_INT = 10,	// Invalid Interrupt Call
	ERR_INVALID_PORT = 11,	// Invalid Port Access
	ERR_COMQUEUE_EMPTY = 12,// Com Queue empty
	ERR_NOLAYER = 13,	// No mine-layer, silly.
	ERR_NOMINES = 14,	// No mines left
	ERR_NOSHIELD = 15,	// No shield installed - Arm the photon torpedoes instead :-)
	ERR_MICROCODE = 16,	// Invalid Microcode in instruction.
	ERR_WRITE_ONLY_PORT =17,// **NEW** Trying to read from a write-only port
	ERR_READ_ONLY_PORT = 18,// **NEW** Trying to write to a read-only port!
	ERR_INPUT_OOBOUNDS = 19 // **NEW** Input parameter out of bounds.
} run_error;

// Compile-time errors
// Perhaps move some of these to parse error (too few robots, too many, config
// file error...)
typedef enum {
	CER_NOT_APPLICABLE = -1,
	CER_NOERR = 0,		// (was "User error")
	CER_INVALID_NLABEL = 1,	// Invalid :label [%s], silly mortal.
	CER_INVALID_REF = 2,	// Invalid identifier %s. A typo perhaps?
	CER_WRONG_AXX = 3,	// Memory access out of range - %s
	// These should be in a separate struct
	CER_TOO_FEW = 4,	// Not enough robots for combat. Maybe we should just drive in circles.
	CER_EMPTY_ARENA = 5,	// Robot names and settings must be specified. An empty arena is no fun.
	CER_CONF_NOT_FOUND = 6,	// Config file not found - %s
	CER_NESTED_CONF = 7,	// Cannot access a config file from a config file - %s
	CER_NOFOUND = 8,	// Robot not found - %s. Perhaps you mistyped it?
//	CER_OUT_OF_MEMORY = 9,	// Insufficient RAM to load robot %s... This is not good.
	CER_TOO_CROWDED = 10,	// Too many robots! We can only handle (max_robots+1)! Blah.. limits are limits.
	CER_DUPLICATE_DEF = 11,	// You already have a perfectly good #def for %s, silly.
	CER_LONG_VARIABLE = 12,	// Variable name too long! (Max: max_var_len) %s
	CER_XLBL_COLLIDE = 13,	// !Label already defined - %s - silly.
	CER_TOO_MANY_VARS = 14,	// Too many variables! (Var limit: max_vars)
	CER_TOO_MANY_XLBL = 15,	// Too many !labels (!Label limit: max_labels)
	CER_LONG_PROGRAM = 16,	// Robot program too long! Boldly we simplify, simplify along...
	CER_MISSING_XLBL = 17,	// !Label missing error. !Label #%s
	CER_XLBL_OORANGE = 18,	// !Label out of range: %s
	CER_XLBL_NOTFOUND = 19,	// !Label not found. %s
	CER_INVALID_DEV = 20,	// Invalid config option: %s. Inventing a new device?
	CER_INV_DEV_ASN = 27,	// **NEW** Invalid device weighting number.
	CER_CHEATER = 21,	// Robot is attempting to cheat; Too many config points (%i)
	CER_MICRO_SCARCE = 22,	// Insufficient data in data statement: %s
	CER_MICRO_DENSE = 23,	// Too many asterisks: %s
	CER_MICRO_NONINT = 24,	// **NEW** Microcode is non-integer
	CER_MICRO_NONMICRO = 25,// **NEW** Requested microcode parse, but isn't
	CER_NOACCESS = 26,	// **NEW** Cannot access file
	CER_OOBOUNDS = 28,	// **NEW** Number out of bounds (can't address)
	CER_NLABEL_OORANGE = 29,// **NEW** :label out of range.
	CER_F_AMBIGUOUS = 30,	// **NEW** Ambiguous file name.
	CER_LOCKED = 31,	// **NEW** Robot is locked, we don't support it.
} compile_error;

#endif
