/*
 * EncoderPair.cpp
 *
 *  Created on: Dec 5, 2018
 *      Author: Jesus Velarde
 */

#include <EncoderPair.h>
#include <stdio.h>
#include <math.h>


EncoderPair::EncoderPair(int pinR1, int pinR2, int pinL1, int pinL2){
	rightEncoder = new frc::Encoder(pinR1, pinR2, true, frc::Encoder::EncodingType::k4X);
	rightEncoder->SetMaxPeriod(.1);
	rightEncoder->SetMinRate(10);
	rightEncoder->SetDistancePerPulse((5 * M_PI)/128);
	rightEncoder->SetReverseDirection(true);
	rightEncoder->SetSamplesToAverage(7);
	rightEncoder->Reset();

	leftEncoder = new frc::Encoder(pinL1, pinL2, false, frc::Encoder::EncodingType::k4X);
	leftEncoder->SetMaxPeriod(.1);
	leftEncoder->SetMinRate(10);
	leftEncoder->SetDistancePerPulse((5 * M_PI)/128);
	leftEncoder->SetReverseDirection(false);
	leftEncoder->SetSamplesToAverage(7);
	leftEncoder->Reset();

	timer = new frc::Timer();
	rightPositionDelta = leftPositionDelta = rightPositionInitial = leftPositionInitial = rightVelocity = leftVelocity = 0.0;

}

void EncoderPair::Update(){

	++cycle;
	if(cycle == 10){
		cycle = 1;
	}else{
		return;
	}

	rightPositionDelta = rightEncoder->GetDistance() - rightPositionInitial;
	leftPositionDelta = leftEncoder->GetDistance() - leftPositionInitial;
	rightPositionInitial = rightEncoder->GetDistance();
	leftPositionInitial = leftEncoder->GetDistance();

	double timeFinal = timer->Get();
	double timeDelta = timeFinal - timeInitial;

	if(timeDelta > 0.0){
		char buf[1024];
		buf[0] = 0;

		//Velocity is in terms of in/s
		rightVelocity = rightPositionDelta/timeDelta;
		leftVelocity = leftPositionDelta/timeDelta;

		//sprintf(buf, "R: %i - L: %i", rightEncoder->Get(), leftEncoder->Get());
		sprintf(buf, "R: %f, L: %f", rightVelocity, leftVelocity);


		frc::DriverStation::ReportError(buf);

	} else{
		frc::DriverStation::ReportError("Time Zero Error");
	}

	timer->Reset();
	timer->Start();
	return;
}


EncoderSingle::EncoderSingle(int pin1, int pin2){
	_Encoder = new frc::Encoder(pin1, pin2, true, frc::Encoder::EncodingType::k4X);
	_Encoder->SetMaxPeriod(.1);
	_Encoder->SetMinRate(10);
	_Encoder->SetDistancePerPulse((5 * M_PI)/128);
	_Encoder->SetReverseDirection(true);
	_Encoder->SetSamplesToAverage(7);
	_Encoder->Reset();
	this->iZeroValue = _Encoder->Get();

	timer = new frc::Timer();
}

void EncoderSingle::Update(){
	return;
}

int EncoderSingle::Get() {
	char buf[1024];
		sprintf(buf, "In Get %d : %d : %d", _Encoder->Get(), this->iZeroValue, this->iZeroValue-_Encoder->Get());
//		frc::DriverStation::ReportError(buf);

	return _Encoder->Get()-this->iZeroValue;
}

void EncoderSingle::Zero() {
	char buf[1024];
		sprintf(buf, "ZERO Get %d : %d : %d", _Encoder->Get(), this->iZeroValue, this->iZeroValue-_Encoder->Get());
		frc::DriverStation::ReportError(buf);
	this->iZeroValue = _Encoder->Get();
}

