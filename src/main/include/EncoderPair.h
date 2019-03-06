/*
 * EncoderPair.h
 *
 *  Created on: Dec 5, 2018
 *      Author: Jesus Velarde
 */

#ifndef SRC_ENCODERPAIR_H_
#define SRC_ENCODERPAIR_H_

#include "Robot.h"
#include <frc/Encoder.h>
#include <DigitalInput.h>
#include <Timer.h>
#include <frc/DriverStation.h>
#include <math.h>

class EncoderPair {
private:
	frc::Encoder *rightEncoder, *leftEncoder;
	frc::Timer *timer;
	double timeInitial = 0.0;
	double rightPositionDelta, leftPositionDelta, rightPositionInitial, leftPositionInitial, rightVelocity, leftVelocity;
	int cycle = 0;

public:
	frc::Encoder getRightEncoder();
	frc::Encoder getLeftEncoder();
	void Update();
	double getDeltafVelocity();
	double getRightEncoderVelocity();
	double getLeftEncoderVelocity();

	EncoderPair(int pinR1, int pinR2, int pinL1, int pinL2);
};

class EncoderSingle {
private:
	frc::Encoder *_Encoder;
	frc::Timer *timer;
	double timeInitial = 0.0;
	int iZeroValue;

public:
	frc::Encoder getEncoder();
	void Update();
	int Get();
	void Zero();

	EncoderSingle(int pin1, int pin2);
};

#endif /* SRC_ENCODERPAIR_H_ */
