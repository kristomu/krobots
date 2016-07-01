#ifndef _KROB_COMMS
#define _KROB_COMMS

#include <vector>
#include <assert.h>
#include <math.h>

using namespace std;

// Communications queue. The mechanics are as follows:
// 	- The queue is a n-value array. When we get a message, it's pushed onto
// 	  the end (with wrapping), and the "last message at" counter is updated.
// 	- Reading increments a "last read" counter unless this would bring it
// 	  above the number of messages read (in which case we return 0).

// For speed purposes, n (the size of the queue) must be a power of two.
// We also increment the messages_read counter when we would otherwise be
// writing past the number of messages read; this emulates the fact that
// the old unread message is already lost.

class comms {

	private:
		unsigned int n;
		vector<short> data;
		// unwrapped
		unsigned int messages_received, messages_read;

	public:
		comms(unsigned int n_in);
		bool is_message_pending() const;
		short get_last_msg();
		short get_last_read_pos() const;
		short get_last_recv_pos() const;
		short get_at_location(unsigned int location) const;
		short get_num_accessible_messages() const;
		void add(short to_add);
		void null(); // Empty queue by setting # read to # recvd.

};

comms::comms(unsigned int n_in) {
	// Check that n is a power of two
	unsigned int powtwo = round(exp(round(log(n_in)/log(2))*log(2)));
	assert(powtwo == n_in);
	
	n = n_in;
	data.resize(n);
	messages_received = 0;
	messages_read = 0;
};

bool comms::is_message_pending() const {
	return(messages_received > messages_read);
}

short comms::get_last_msg() {
	// If there's any unread message, read it and increment the counter
	// of how many messages we've read. 
	if (messages_received > messages_read)
		return(data[messages_read++ & (n-1)]);
	else	return(0);
}

short comms::get_last_read_pos() const {
	return(messages_read & (n-1));
}

short comms::get_last_recv_pos() const {
	return(messages_received & (n-1));
}

short comms::get_at_location(unsigned int location) const {
	if (location < 0 || location >= n) return(0);
	return(data[location]);
}

short comms::get_num_accessible_messages() const {
	return((messages_received - messages_read) & (n-1));
}

void comms::add(short to_add) {
	//cout << "COMMS: Adding something. " << to_add << " wwt " << (messages_received & (n-1)) << endl;
	// Increment messages received and add the message where appropriate.
	data[messages_received++ & (n-1)] = to_add;

	// Update messages_read to reflect the reality of the queue's limited
	// storage. There may be an off-by-one here. DONE, check if there is.
	// (No.)
	if (messages_received - messages_read >= n)
		messages_read = messages_received - (n-1);

	//cout << "After this, received: " << messages_received << ", read: " << messages_read << endl;
}

void comms::null() {
	messages_read = messages_received;
}

#endif
