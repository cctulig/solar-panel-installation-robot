#include "Approve.h"

//User function to be called when a packet comes in
// Buffer contains data from the packet coming in at the start of the function
// User data is written into the buffer to send it back
void Approve::event(float * buffer) {
	robotPointer->Approve(buffer);

}
