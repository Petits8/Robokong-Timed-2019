

#include <DriverControl.h>
#include <math.h>
#include <EncoderPair.h>

DriverControl::DriverControl(bool bJoystick){
	this->bJoystick = bJoystick;
	for(int i=0; i<3; ++i){
		for(int j=0; j<11; ++j){
			this->buttons[i][j] = false;
		}
	}
	for(int i=0; i<2; ++i){
		for(int j=0; j<3; ++j){
			this->controllerVector[i][j] = 0.0;
		}
	}
	this->timer = new frc::Timer();
}
bool DriverControl::getStationButton(int id){
	return this->d_station_controller.GetRawButton(id);
}
int DriverControl::getStationButton(){
	if (this->getStationButton(1)) return 1;
	if (this->getStationButton(2)) return 2;
	if (this->getStationButton(3)) return 3;
	if (this->getStationButton(4)) return 4;
	if (this->getStationButton(5)) return 5;
	if (this->getStationButton(6)) return 6;
	if (this->getStationButton(7)) return 7;
	if (this->getStationButton(8)) return 8;
	if (this->getStationButton(9)) return 9;
	if (this->getStationButton(10)) return 10;
}

bool DriverControl::IsJoystick(){return this->bJoystick;}
double DriverControl::GetVectorValue(int axis){
	double Y_AXIS_VALUE;
	double Y_AXIS_VALUE_NEW;
	if(bJoystick){
		switch(axis){

		case DRVFORWARD:
			return this->l_joystick.GetY()/this->divider;
			break;
			//Y_AXIS_VALUE = this->l_joystick.GetY()/this->divider;
		case DRVROTATE:
			return this->r_joystick.GetX()/this->divider;
			break;
		case Z_AXIS:
			return (this->l_joystick.GetZ()+this->r_joystick.GetZ())/2;
			break;
		case DRVARM:
			return this->l_joystick.GetX()/this->divider;
			break;
		case DRVWRIST:
			return this->r_joystick.GetRawAxis(1)/this->divider;	
		default:
			return 0.0;

		};
	} else {
			switch(axis){

		case Y_AXIS:


			Y_AXIS_VALUE = this->l_joystick.GetRawAxis(1)/this->divider;
			break;


		case X_AXIS:
			return this->l_joystick.GetRawAxis(4)/this->divider;


		case Z_AXIS:
		
			return 0.0;
		default:
			return 0.0;

		};

		double timeDelta = timer->Get() - timeInitial;
		timeInitial = timer->Get();

		if(timeDelta == 0.0){
			return Y_AXIS_VALUE;
		}

		yDelta = Y_AXIS_VALUE - yInitial;
		if(yDelta/timeDelta > PWRVEL){
			Y_AXIS_VALUE_NEW = yInitial + PWRVEL*timeDelta;
		} else if(yDelta/timeDelta < -PWRVEL){
			Y_AXIS_VALUE_NEW = yInitial - PWRVEL*timeDelta;
		} else{
			Y_AXIS_VALUE_NEW = Y_AXIS_VALUE;
		}

		timer->Reset();
		timer->Start();
		yInitial = Y_AXIS_VALUE_NEW;
		return Y_AXIS_VALUE_NEW;

	}
}
double DriverControl::GetLiftValue(){
	return this->r_joystick.GetY();
}

bool DriverControl::GetButtonValue(int stick, int button){
	if(stick == L_STICK){
		return this->l_joystick.GetRawButton(button);
	} else if(stick == R_STICK){
		return this->r_joystick.GetRawButton(button);
	}
	return false;
}

void DriverControl::Update(){
	
	if(this->bJoystick){
		
		//Checking Slow Down Button
		if(this->buttons[1][2] == false && this->r_joystick.GetRawButton(3)){
			if(this->divider == 1) this->divider = SPEED_DIVIDE;
			else if(this->divider == SPEED_DIVIDE) this->divider = 1;
		} 

		//Checks if R_Trigger is pressed and toggles pneumatic claw
		if(this->buttons[2][8] == false && this->d_station_controller.GetRawButton(9)){
			ToggleClaw();
			//frc::DriverStation::ReportError("BUTTON");
		}

		//Updating Array

		for(int i=0; i<3; i++){
			for(int j=0; j<11; j++){
				if(i==0){
					this->buttons[i][j] = this->l_joystick.GetRawButton(j+1);
				} else if(i==1){
					this->buttons[i][j] = this->r_joystick.GetRawButton(j+1);
				} else if(i == 2){
					this->buttons[i][j] = this->d_station_controller.GetRawButton(j+1);
				}
			}
		}
	}
}

bool DriverControl::isFullSpeed(){
	if(this->divider == 1) return true;
	return false;
}

void DriverControl::ToggleClaw(){
	if(this->clawPiston.Get() == frc::DoubleSolenoid::Value::kForward){
		this->clawPiston.Set(frc::DoubleSolenoid::Value::kReverse);
	} else if(frc::DoubleSolenoid::Value::kReverse){
		this->clawPiston.Set(frc::DoubleSolenoid::Value::kForward);
	} else{
		this->clawPiston.Set(frc::DoubleSolenoid::Value::kForward);
	}
}

void Arm::Move(double vector){
	char buf[1024];
	sprintf(buf, "Move: %f", vector);
//	frc::DriverStation::ReportError(buf);

	this->_arm->Set(vector);
}

void Arm::Goto(int target, int spot, double w_target) {	
	char buf[1024];
	int error;
	double speed;
	double HoldSpeed;
	double ArmUp;
	double ArmDown;
	double ArmMidSpeed;
	double ArmHighSpeed;
	int ArmMidCutoff;
	int ArmHighCutoff;
	bool bMove = true; 


	if (target < LOW_CUTOFF){
		frc::DriverStation::ReportError("LOW");
		HoldSpeed=LOW_ARM_HOLD;
		ArmUp=LOW_ARM_UP;
		ArmDown=LOW_ARM_DOWN;
		ArmMidSpeed=LOW_MID_SPEED;
		ArmHighSpeed=LOW_HIGH_SPEED;
		ArmMidCutoff=LOW_MID_CUTOFF;
		ArmHighCutoff=LOW_HIGH_CUTOFF;
	} else if (target >= LOW_CUTOFF && target < MID_CUTOFF){
		frc::DriverStation::ReportError("MID");
		HoldSpeed=MID_ARM_HOLD;
		ArmUp=MID_ARM_UP;
		ArmDown=MID_ARM_DOWN;
		ArmMidSpeed=MID_MID_SPEED;
		ArmHighSpeed=MID_HIGH_SPEED;
		ArmMidCutoff=MID_MID_CUTOFF;
		ArmHighCutoff=MID_HIGH_CUTOFF;
	} else if (target>=MID_CUTOFF && target<HIGH_CUTOFF) { 
		frc::DriverStation::ReportError("HIGH");
		HoldSpeed=HIGH_ARM_HOLD;
		ArmUp=HIGH_ARM_UP;
		ArmDown=HIGH_ARM_DOWN;
		ArmMidSpeed=HIGH_MID_SPEED;
		ArmHighSpeed=HIGH_HIGH_SPEED;
		ArmMidCutoff=HIGH_MID_CUTOFF;
		ArmHighCutoff=HIGH_HIGH_CUTOFF;
	} else {
		frc::DriverStation::ReportError("GRAB");
		HoldSpeed=GRAB_ARM_HOLD;
		ArmUp=GRAB_ARM_UP;
		ArmDown=GRAB_ARM_DOWN;
		ArmMidSpeed=GRAB_MID_SPEED;
		ArmHighSpeed=GRAB_HIGH_SPEED;
		ArmMidCutoff=GRAB_MID_CUTOFF;
		ArmHighCutoff=GRAB_HIGH_CUTOFF;
	}


	speed=0;
	error=abs(spot-target);

	//
	// Once we get to the target, move the wrist into position
	//
	if (error<2) {
		//sprintf(buf, "WRIST1 %f %f",w_target, this->GetWrist()->GetEncoderValue());
//		frc::DriverStation::ReportError(buf);
		//sprintf(buf, "WRIST2: T:%d S:%d Sp: %f E:%d %f", target, spot, speed, error, HoldSpeed);
		//frc::DriverStation::ReportError(buf);
		
		this->timer->Start();
		this->_arm->Set(HoldSpeed);
		this->GetWrist()->Goto(w_target, this->GetWrist()->GetEncoderValue(), spot);
		return;
	}
	
	if(target != this->prev_target){

		if(spot < GRAB_CUTOFF && spot > GRAB_CUTOFF-GRAB_ZONE_OFFSET){
			frc::DriverStation::ReportError("GRAB ZONE");
		} else if(wrist->Tuck(0, this->GetWrist()->GetEncoderValue())){
			this->prev_target = target;
		} else{
			bMove = false; 
		}
	}

 	//
	// Tuck the wrist in before we travel
	//
/* 	if (this->GetWrist()->GetEncoderValue()>4 && this->timer->Get()>3){
		sprintf(buf, "Tuck %f %d=%d-%d T:%d", this->GetWrist()->GetEncoderValue(), error, spot, target, this->timer->Get());
		frc::DriverStation::ReportError(buf);
		this->timer->Stop();
//		this->timer->Reset();
		this->GetWrist()->Tuck(1,this->GetWrist()->GetEncoderValue());
		return;
	}
 */
	//
	// Figure out how fast to move
	//
	if (error >=0 && error < ArmMidCutoff) {
		if (target>spot) {
			speed=ArmUp;
		} else {
			speed=ArmDown;
		}
		sprintf(buf, "LOW %f ",speed);
	}
	if (error >= ArmMidCutoff && error < ArmHighCutoff) { 
		speed=ArmMidSpeed; 
		sprintf(buf, ">MID %f ",speed);
	}
	if (error >= ArmHighCutoff) {
		speed=ArmHighSpeed; 
		sprintf(buf, ">HIGH %f ",speed);
	}

	sprintf(buf, "%s Goto: T:%d S:%d Sp: %f E:%d  %f : %f : %f : %f : %f : %d : %d T:%d", buf, target, spot, speed, error, HoldSpeed, ArmUp, ArmDown, ArmMidSpeed, ArmHighSpeed, ArmMidCutoff, ArmHighCutoff, this->timer->Get());
	frc::DriverStation::ReportError(buf);

	//
	// Now move to the target
	//

	if(bMove == false){
		return; 
	}

	if (target < spot) {
		this->_arm->Set(speed);
	} else if (target > spot){
		this->_arm->Set(-speed);
	} else {
		this->_arm->Set(-speed);
	}
}	

bool Wrist::Tuck(int target, int spot){
	char buf[1024];
	double speed;

	/*if (spot < LOW_CUTOFF) {
		speed = LOW_WRIST_SPEED;
	} else if (spot >= LOW_CUTOFF && spot < MID_CUTOFF){
		speed = MID_WRIST_SPEED;
	} else if (spot >= MID_CUTOFF && spot < HIGH_CUTOFF){
		speed = HIGH_WRIST_SPEED;
	} else {
		speed = GRAB_WRIST_SPEED;
	}
	*/
	
	int difference = target-this->GetEncoderValue();
	speed = difference * .03;
	if(speed > .15){
		speed = .15;
	}else if(speed < -.15){
		speed = -.15;
	}
	

	sprintf(buf, "Speed: %f", speed);
	frc::DriverStation::ReportError(buf);
	this->currentWristSpeed = speed;

	if(abs(target-this->GetEncoderValue()) <= .5){
		return true;
	}

	
	return false; 
	
}

void Wrist::Move(double vector){
	char buf[1024];

	this->_wrist->Set(vector);
	sprintf(buf, "Wrist: %f : %f", vector, this->_wrist->Get());
//	frc::DriverStation::ReportError(buf);
}

void Wrist::Goto(int target, int spot, int ArmPosition){
	char buf[1024];
	double HoldSpeed;
	double speed;

	if (ArmPosition < LOW_CUTOFF){
		HoldSpeed=LOW_WRIST_HOLD;
		speed=LOW_WRIST_SPEED;
	} else if (ArmPosition >= LOW_CUTOFF && ArmPosition < MID_CUTOFF){
		HoldSpeed=MID_WRIST_HOLD;
		speed=MID_WRIST_SPEED;
	} else if (ArmPosition >= MID_CUTOFF && ArmPosition < HIGH_CUTOFF){
		HoldSpeed=HIGH_WRIST_HOLD;
		speed=HIGH_WRIST_SPEED;
	} else {
		HoldSpeed=GRAB_WRIST_HOLD;
		speed=GRAB_WRIST_SPEED;
	}

	int error = abs(target-spot);
	if(error < 2){ 
		speed = HoldSpeed;
	}

	sprintf(buf, "Wrist Target:%d Spot:%d W:%f S:%f E:%d %f %f", target, spot, this->GetEncoderValue(), speed, error, HoldSpeed, speed);
	frc::DriverStation::ReportError(buf);

	if (0==error){
		this->Move(HoldSpeed);
		return;
	}
	

	if (target < spot) {
		this->Move(-speed);
	} else if (target > spot){
		this->Move(speed);
	} else {
		this->Move(speed);
	}
}

void Wrist::Update(){

}

int Wrist::Get(){

}

void Wrist::Zero(){

}
void Wrist::Init(){
	this->encoderInitial = _wrist->GetEncoder().GetPosition();
}

double Wrist::GetEncoderValue(){
	return this->_wrist->GetEncoder().GetPosition() - this->encoderInitial;
}
Wrist* Arm::GetWrist(){
	return this->wrist;
}
void Wrist::Stay(double spot){
	double difference = spot - this->GetEncoderValue();
	double speed = difference * .2;
	if(speed > WRIST_HOLD_CAP){
		speed = WRIST_HOLD_CAP;
	}
	_wrist->Set(speed); 
	return;
}

void Arm::Stay(int spot, int current){
	int difference = spot - current;
	double speed = (int)difference * .04;
	if(speed > WRIST_HOLD_CAP){
		speed = WRIST_HOLD_CAP;
	}
	_arm->Set(-speed); 
	return;
}

void Arm::_Goto(int target, int spot, double w_target){
	bool bMove = true;
	double tuckLocation = 0;
	if(spot > 300){
		tuckLocation = WRIST_FULL_ROT/2.0 + 13;
		frc::DriverStation::ReportError("BACKWARDS TUCK");
	}

	if(abs(target-spot) > 5){

		/*if(spot < GRAB_CUTOFF && spot > GRAB_CUTOFF-GRAB_ZONE_OFFSET){
			frc::DriverStation::ReportError("GRAB ZONE");
		} else */if(!wrist->Tuck(tuckLocation, this->GetWrist()->GetEncoderValue())){
			bMove = false; 
		}
	}
	if(bMove){
		if(_Stay(target, spot)){
			this->GetWrist()->_Stay(w_target);
		}
	}
	this->GetWrist()->Move(this->wrist->currentWristSpeed);

}

void Wrist::_Stay(double spot){
	double difference = spot - this->GetEncoderValue();
	double speed = difference * .03;
	if(speed > .15){
		speed = .15;
	} else if(speed < -.15){
		speed = -.15;
	this->currentWristSpeed =  speed;
	}
	return;
}

bool Arm::_Stay(int spot, int current){
	char buf[1024];
	buf[0] = 0;
	int difference = spot - current;
	double speed = (float)difference * .06;
	double Angle_A = ((double)spot/600.0) * 2 * M_PI;
	Angle_A -= M_PI/2;
	double maxSpeed = .6;
	/*
	if((difference > 0 && spot < 300)
		|| (difference < 0 && spot > 300)){
		maxSpeed = .2;
		frc::DriverStation::ReportError("Low Speed");
	} else if((difference > 0 && spot > 300)
		|| (difference < 0 && spot < 300)){
		maxSpeed = .6;
		frc::DriverStation::ReportError("High Speed");
	} else{
		maxSpeed = .3;
		frc::DriverStation::ReportError("Max Speed Error");
	}
	sprintf(buf, "Angle: %f * PI ;: Current: %i", Angle_A/M_PI, current);
	frc::DriverStation::ReportError(buf);
	*/

	if(speed > maxSpeed){
		speed = maxSpeed;
	} else if(speed < -maxSpeed){
		speed = -maxSpeed;
	}
	_arm->Set(-speed); 
	

	if(abs(difference) < 10){
		return true;
	} 
	return false;
}

double Arm::getMaxAngle_W(int Value_A){
	char buf[1024];
	buf[0] = 0;
	double Angle_A = ((double)Value_A/600.0) * 2 * M_PI;
	Angle_A -= M_PI/2;
	
	if(Angle_A >= M_PI/2 && Angle_A <= M_PI){
		Angle_A = M_PI - Angle_A;
	} else if(Angle_A>=M_PI && Angle_A<=1.5*M_PI){
		Angle_A -= M_PI;
	} else if(Angle_A >= 1.5*M_PI && Angle_A <= 2*M_PI){
		Angle_A = 2*M_PI - Angle_A;
	}
	if((WRIST_L + ARM_L*cos(Angle_A))>= MAX_L){
	double MAX_W = acos((MAX_L - ARM_L*cos(Angle_A))/WRIST_L);

	//sprintf(buf, "MAX_W: %f : ANGLE_A: %f : CONDITIONAL_CALC: %f : MAX_CALC: %f : TICK: %i", MAX_W, Angle_A, (WRIST_L + ARM_L*cos(Angle_A)), (MAX_L - ARM_L*cos(Angle_A))/WRIST_L, Value_A);
	//frc::DriverStation::ReportError(buf);

	} else{
		return 0.0;
	}
}
