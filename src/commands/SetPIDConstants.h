/*
 * PIDConfigureSimplePacketComsServer.h
 *
 *  Created on: Oct 19, 2018
 *      Author: hephaestus
 */

#ifndef SRC_COMS_PIDCONFIGURESIMPLEPACKETCOMSSERVER_H_
#define SRC_COMS_PIDCONFIGURESIMPLEPACKETCOMSSERVER_H_
#include <SimplePacketComs.h>
#include "../pid/PIDMotor.h"

class SetPIDConstants: public PacketEventAbstract {
	PIDMotor ** pidlist;
	int numPID;
public:
	SetPIDConstants(int num, PIDMotor ** list);
	virtual ~SetPIDConstants(){}
	void event(float * buffer);

};

#endif /* SRC_COMS_PIDCONFIGURESIMPLEPACKETCOMSSERVER_H_ */
