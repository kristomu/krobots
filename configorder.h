// This header defines the default #config order which is used when assembling
// #config data. It is taken from the FAQ: Scanner Weapon Armor Engine Heatsinks
// Mines Shield.

#ifndef _KROB_CONFORD
#define _KROB_CONFORD

typedef enum { CNF_SCANNER = 0, CNF_WEAPON = 1, CNF_ARMOR = 2, CNF_ENGINE = 3,
	CNF_HEATSINKS = 4, CNF_MINES = 5, CNF_SHIELD = 6, C_NUMDEVICES = 7 }
	config_order;

#endif
