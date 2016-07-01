#ifndef __TOOLS
#define __TOOLS

#include <math.h>
#include <ctype.h>
#include <sys/time.h>
#include <assert.h>
#include <sstream>
#include <algorithm>

#include "cpu/errors.h"

#include <stdio.h> // Eugh; required to differentiate between file-not-exist
#include <errno.h> // and file-no-permissions. Such a HACK.

#include <stdlib.h> // stoi calls atoi for now.

#include <iostream>

using namespace std;

// For itos and stoi operators. Doesn't work yet.
stringstream qr;
ostringstream qo;

int sign(const double & in) {
	if (in < 0) return(-1);
	return(1);
}

// EXPERIMENTAL, just to see how much we gain
// We gain a lot when naive, but caching does almost as well and gives us
// accurate angles.
double cos_coeff[256], sin_coeff[256];

double hex_to_radian(const double hexdegrees);

void init_coeffs() {
	for (int counter = 0; counter < 256; ++counter) {
		cos_coeff[counter] = cos(hex_to_radian(counter));
		sin_coeff[counter] = sin(hex_to_radian(counter));
	}
}

double vcos64(int hex) { return(cos_coeff[hex]); }
double vsin64(int hex) { return(sin_coeff[hex]); }

const double PI = 4 * atan(1);

double square(double a) { return(a*a); }

double get_abs_time() {
	timeval tv;
	if (gettimeofday(&tv, NULL) == 0)
		return (tv.tv_sec + tv.tv_usec/1e6);
	else    return (0);
}


/*template<typename T> T hexangle(const T & in) {
	return(fmod(256 + in, 256));
}*/

// Gnarly optimization: Since hexangles are 0..256, we "round" by forcing it
// into a char. 16% improvement.
double hexangle(const double & in) {
	if (!finite(in)) return(in);
	double f = floor(in);
	return ((unsigned char)(f) + (in-f));
}

// To check: whether -25.9 rounds the same way as (256 - 25.9).
int quant_hexangle(const double & in) {
	assert(finite(in));
	return((unsigned char)(round(in)));
}

// Angular measurement

// Hexdegrees are rotated so as to fit with the rotation system of ATR2, where
// 0 points right up and 64 is to the east (when facing north).
// (Check these for all quadrants)

// No, they aren't. Not yet. (It screws up missiles, etc)

double hex_to_radian(const double hexdegrees) {
	double hexsrc = hexangle(hexdegrees - 64);
	return(hexsrc / (128/PI));
}

double radian_to_hex(const double radians) {
	// Handle negative radians.
	double output = radians * (128 / PI);
	if (output < 0) output += 256;

	return(hexangle(output + 64));
}

double deg_to_radian(const double degrees) {
	return(degrees / (180/PI));
}

double radian_to_deg(const double radians) {
	double output = radians * (180 / PI);
	if (output < 0) output += 360;

	return(output);
}

double hex_to_deg(const double hex) {
	// A bit inefficient, but this makes it neutral.
	return (radian_to_deg(hex_to_radian(hex)));
}

// Other angular tools

double normalize_sum(const double a, const double b, int circle) {
	double toRet = a + b;
	while (toRet < 0) toRet = circle + toRet;
	return(fmod(toRet, circle));
}

double normalize_hex(const double a, const double b) {
	// DEBUG:
	/*if (hexangle(a+b) != normalize_sum(a, b, 256)) {
		cout << "NHE: " << a << ", " << b << ", hexangle: " << hexangle(a+b) << ", normalize_sum: " << normalize_sum(a, b, 256) << endl;
	}*/

	// Get around loss of floating point precision
	if (a > 1024 || b > 1024 || a < -1024 || b < -1024)
		return(normalize_sum(a, b, 256));
	else	return(hexangle(a+b));
}

// Angle within determines if to_check is within the arc from start_angle to
// end_angle. Circle is the number of units that form a circle (256 for hex,
// 360 for degrees).
// Well, that's neat. How big is my blind spot, when I couldn't see this simple
// solution?
bool angle_within(double start_angle, double end_angle, double to_check,
		double circle) {

	end_angle = fmod(circle + end_angle - start_angle, circle);
	to_check = fmod(circle + to_check - start_angle, circle);

	return(end_angle >= to_check);
}

// Specialization.
bool hexangle_within(const double start_angle, const double end_angle,
		const double to_check) {
	return ( hexangle(end_angle - start_angle) >= 
			hexangle(to_check - start_angle));
}
	
/*bool angle_within(double start_angle, double end_angle, double to_check,
		double circle) {

	// General idea: if we're on an edge case around 0 degrees, then
	// rotate everything by half a circle so we're no longer at that
	// edge case.
	
	// Convert to_check. The function assumes start_angle and end_angle
	// are on the right interval.
	// Perhaps replace with fmod.
	if (to_check < 0) to_check += circle;
	if (to_check > circle) to_check -= circle;
	if (start_angle < 0) start_angle += circle;
	if (start_angle > circle) start_angle -= circle;
	if (end_angle < 0) end_angle += circle;
	if (end_angle > circle) end_angle -= circle;

	if (end_angle < start_angle) {
		if (to_check < start_angle && to_check > end_angle)
			return(false);

		to_check += circle * 0.5;
		start_angle += circle * 0.5;
		end_angle += circle * 0.5;

		if (to_check > circle) to_check -= circle;
		if (end_angle > circle) end_angle -= circle;
		if (start_angle > circle) end_angle -= circle;
	}

	return (to_check >= start_angle && to_check <= end_angle);
}*/

// Euclidean distance and squared

double euc_sq_distance(const double x1, const double y1, const double x2, 
		const double y2) {
	double dx = x1 - x2;
	double dy = y1 - y2;

	return(dx*dx + dy*dy);
}

double euc_distance(const double x1, const double y1, const double x2, 
		const double y2) {
	return(sqrt(euc_sq_distance(x1, y1, x2, y2)));
}

// Integer to string

string itos (int source) {
	ostringstream q;
	q << source;
	string toRet = q.str();
	//cout << "ITOS: [" << source << "] is [" << toRet << "]" << endl;
	return(toRet);
}

string lltos(long long source) {
	ostringstream q;
	q << source;
	return(q.str());
}

string dtos (double source) {
	ostringstream q;
	q << source;
	return(q.str());
}

string dtos (double source, double precision) {
	return(dtos(round(source * pow(10.0, precision)) / 
				pow(10.0, precision)));
}

// Integer to string, padded to size.
string itos(int source, unsigned int minlen) {
	string basis = itos(source);
	if (basis.size() < minlen) {
		string q(minlen - basis.size(), '0');
		return(q + basis);
	} else
		return(basis);
}

string lltos(long long source, unsigned int minlen) {
	string basis = lltos(source);
	if (basis.size() < minlen) {
		string q(minlen - basis.size(), '0');
		return(q + basis);
	} else
		return(basis);
}

// Hexadecimal versions of the above

string itos_hex(int source) {
	ostringstream q;
	q.flags(ios::hex);
	q << source;
	return(q.str());
}

string lltos_hex(long long source) {
	ostringstream q;
	q.flags(ios::hex);
	q << source;
	return(q.str());
}

string itos_hex(int source, unsigned int minlen) {
	string basis = itos_hex(source);
	if (basis.size() < minlen) {
		string q(minlen - basis.size(), '0');
		return(q + basis);
	} else
		return(basis);
}

string lltos_hex(long long source, unsigned int minlen) {
	string basis = lltos_hex(source);
	if (basis.size() < minlen) {
		string q(minlen - basis.size(), '0');
		return(q + basis);
	} else
		return(basis);
}

// String to integer.

// For comparison purposes
// (Perhaps include a warning that the number will wrap around?)
// Perhaps also alias both to a common function, to promote code reuse.
// Two levels: strict only returns is_integer on shorts. Normal returns them
// on anything. 
template<class T> void arch_stoi(T & dest, bool hex, const string source) {
	stringstream qra;
	if (hex)
		qra.flags(ios::hex);
	qra << source;
	qra >> dest;
	return;
}
long long comp_stoi(const string source) {
	long long output = 0; // In case source is empty
	arch_stoi(output, false, source);
	return(output);
}

unsigned int stoui(const string source) {
	unsigned int output = 0;
	arch_stoi(output, false, source);
	return(output);
}

int stoi(const string source) {
	int output = 0;
	// This is faster. TODO: Fix stringstream so we won't have to kludge
	// like this.
	output = atoi(source.c_str());
	//int comparison = 0;
	//arch_stoi(comparison, false, source);
	//if (output != comparison)
	//	cout << output << "\t" << comparison << " from " << source << endl;
	return(output);
}

long long comp_stoi_hex(const string source) {
	long long output = 0;
	arch_stoi(output, true, source);
	return(output);
}

int stoi_hex(const string source) {
	int output = 0;
	arch_stoi(output, true, source);
	return(output);
}

int stoi_generalized(const string source) {
	// If it's too short to have the hex qualifiers, go right to stoi
	if (source.size() < 2) return(stoi(source));
	// Okay, test if it's hex. If so, strip off the qualifier and return
	// stoi_hex for the integer.
	if (source.size() > 2 && source[0] == '0' && source[1] == 'x')
		return(stoi_hex(source.substr(2, source.size() - 2)));
	if (*(source.end()-1) == 'h') 
		return(stoi_hex(source.substr(0, source.size() - 1)));

	return(stoi(source));
}

// Optimized versions of the above, since we check whether something's an
// integer a great number of times. TODO: Remove this once we got global
// stringstream allocation working.
bool isint_decimal(const string source) {
	string::const_iterator pos = source.begin();

	while (pos != source.end() && *pos == ' ') ++pos;
	if (pos != source.end() && *pos == '-') ++pos;

	if (pos == source.end()) return(false);

	for (; pos != source.end(); ++pos)
		if (*pos < '0' || *pos > '9') return(false);

	//if (!(source == itos(comp_stoi(source), source.size())))
	//	cout << "IID? [" << source << "]" << endl;

	return(true);
}

// Without any prefixes like 0x.. or ..h
bool isint_hex(const string source) {
	return(source == itos_hex(comp_stoi_hex(source), source.size()));
}

bool is_integer(const string source, bool permit_hex) {
	// DEBUG
	//cout << "Checking is_integer: [" << source << "]" << endl;
	// First, find out if it's hex and strip if so.
	bool is_hex = false;

	// Special case of -0. We permit this because of precedent; quite a
	// number of robots use this (perhaps because of rounding from nonints).
	if (source == "-0")
		return(true);

	if (permit_hex && source.size() >= 2) {

		// Either 0x[value] or [value]h, but not both.
		
		if (source.size() > 2 && source[0] == '0' &&source[1] == 'x') 
			return(isint_hex(source.substr(2, source.size() - 2)));

		if (*(source.end()-1) == 'h')
			return(isint_hex(source.substr(0, source.size() - 1)));
	}

	// If we get here, either it's not hex, or we don't accept hex, so
	// check it as decimal.
	return(isint_decimal(source));
	//return (source == itos(comp_stoi(source), source.size()));
}

// Misc string modifications
string lowercase(const string mixed) {
	string toRet = mixed;
	transform(toRet.begin(), toRet.end(), toRet.begin(),
			(int(*)(int)) tolower); // Isn't this funny?
	return(toRet);
}

string uppercase(const string mixed) {
	string toRet = mixed;
	transform(toRet.begin(), toRet.end(), toRet.begin(),
			(int(*)(int)) toupper);	// Second verse..
	return(toRet);
}

string remove_extension(const string fn) {
	// Just search from the end and then chop off at point of first .
	// if any, otherwise entire string.
	// Might fail if there are no extensions but path has . somewhere.
	// 	(Can be "solved" by breaking at first /, then causes problems
	// 	 with filenames with \/ in their extensions.)
	// Fix later. And you could probably use some sort of nifty STL
	// trick to do this without a loop.
	
	bool found = false;
	int pos = -1;
	// > 0 because a file that starts with a period shouldn't have that
	// period counted as the start of an extension.
	for (int counter = fn.size()-1; counter > 0 && !found; --counter) {
		if (fn[counter] == '.') {
			found = true;
			pos = counter;
		}
	}

	if (found)
		return(fn.substr(0, pos));
	else	return(fn);
}

string remove_path(const string fn) {

	// Same as r_e, only that it turns pathnames into filenames by cutting
	// away /.
	
	bool found = false;
	int pos = -1;

	for (int counter = fn.size()-1; counter > 0 && !found; --counter) {
		if (fn[counter] == '/') {
			found = true;
			pos = counter;
		}
	}

	if (found)
		return(fn.substr(pos+1, fn.size()-pos));
	else	return(fn);
}

// Filename check. This is the Linux way of determining whether a file failed
// because it doesn't exist or because we don't have permissions, since fstream
// is remarkably uncooperative about this facet. We return CER_NOACCESS upon
// permissions errors (including trying to open a directory), CER_NOFOUND 
// upon "file isn't here", or CER_NOERR if it's openable.

compile_error check_filename(const string filename, const compile_error 
		passthrough) {
	FILE * p = fopen(filename.c_str(), "r");

	if (p != NULL) {
		fclose(p);
		return(CER_NOERR);
	} else {
		// Perhaps have an entry, and error, for EISDIR here?
		switch(errno) {
			case ENOENT:	return(CER_NOFOUND);
			case EPERM:
			case EACCES:	return(CER_NOACCESS);
			default:	return(passthrough);
		}
	}

	// Shouldn't happen
	return(passthrough);
}

// --- Emulation of rotation ops for ATR2

unsigned short rotate_left(unsigned short in, unsigned char how_far) {
	unsigned short outval;
	char how_far_norm = how_far & 15;
	outval = in << how_far_norm;
	outval |= in >> (16 - how_far_norm);
	return(outval);
}

unsigned short rotate_right(unsigned short in, unsigned char how_far) {
	unsigned short outval;
	char how_far_norm = how_far & 15;
	outval = in >> how_far_norm;
	outval |= in << (16 - how_far_norm);
	return(outval);
}

// Display normalization ops.
template<typename T> T norm(T min, T cur, T max) {
	return((cur-min)/(max-min));
}

template<typename T> T renorm(T min_in, T max_in, T cur, T min_out, T max_out) {
	return(norm(min_in, cur, max_in) * (max_out-min_out) + min_out);
}

#endif
