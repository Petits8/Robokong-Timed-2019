/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <frc/Spark.h>
#include <frc/SpeedControllerGroup.h>
#include <Drive/DifferentialDrive.h>
#include <DriverControl.h>
#include <EncoderPair.h>
#include <CameraServer.h>
#include <frc/CAN.h>

frc::Spark MRight_0(0);
frc::Spark MRight_1(1);
frc::Spark MRight_2(2);
frc::Spark MLeft_0(3);
frc::Spark MLeft_1(4);
frc::Spark MLeft_2(5);

//frc::Spark M_ARM1(6);
//frc::Spark M_ARM2(7);

frc::Spark M_INTAKEROLLER(8);
frc::Spark M_INTAKEARM(9);



//frc::SpeedControllerGroup MCG_ARM(M_ARM1, M_ARM2);
frc::SpeedControllerGroup RightMotors(MRight_0, MRight_1, MRight_2);
frc::SpeedControllerGroup LeftMotors(MLeft_0, MLeft_1, MLeft_2);


//frc::Spark YawCameraController(5);
//frc::Spark PitchCameraController(6);

frc::DifferentialDrive m_robotDrive{LeftMotors, RightMotors};

//EncoderPair *pEncoderPair = new EncoderPair(4, 5, 2, 3);
EncoderSingle *pShoulderEncoder = new EncoderSingle(0,1);

DriverControl *pDriverControl = new DriverControl(true);
Arm *arm = new Arm(6,7,1);

bool bInitLatch; 
char buf[1024];
int b7Latch;
bool bButtonReleaseLatch; 
float w_last;
int arm_last;
int roller_state;
int d_joystick_latch;
double maxAngle_w;
double angle_w;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
  frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
   arm->GetWrist()->Init();
	pShoulderEncoder->Zero(); 
  bInitLatch = false; 
	buf[0] = 0;
	m_robotDrive.SetSafetyEnabled(true);
  b7Latch=0;
  bButtonReleaseLatch = true; 
	w_last;
	roller_state = 0;
	d_joystick_latch = 0;
	maxAngle_w = 0.0;
	angle_w = 0.0;
	pDriverControl->clawPiston.Set(frc::DoubleSolenoid::Value::kForward);
}

void Robot::AutonomousPeriodic() {
  TeleopPeriodic();
}

void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
  pShoulderEncoder->Update();
		pDriverControl->Update();
			
		// Drive with arcade style (use right stick)

		

		switch (pDriverControl->getStationButton()) {
			case 5:
				arm->_Goto(STOP_GRAB, pShoulderEncoder->Get(), WRIST_GRAB);
				bButtonReleaseLatch = false;
				break;
			case 6:
				arm->_Goto(STOP_TOP, pShoulderEncoder->Get(), WRIST_TOP);
				bButtonReleaseLatch = false;
				break;
			case 7:
				arm->_Goto(STOP_CENTER, pShoulderEncoder->Get(), WRIST_CENTER);
				bButtonReleaseLatch = false;
				break;
			case 8:
				arm->_Goto(STOP_LOW, pShoulderEncoder->Get(), WRIST_LOW);
				bButtonReleaseLatch = false;
				break;
			case 4:
				arm->_Goto(STOP_LOW_H, pShoulderEncoder->Get(), WRIST_LOW_H);
				bButtonReleaseLatch = false;
				if (pDriverControl->getStationButton(1) && pDriverControl->getStationButton(1)){
					arm->GetWrist()->Init();
					pShoulderEncoder->Zero(); 
				}
				break;
			case 3:
				arm->_Goto(STOP_CENTER_H, pShoulderEncoder->Get(), WRIST_CENTER_H);
				bButtonReleaseLatch = false;
				break;
			case 2: 
				arm->_Goto(STOP_TOP_H, pShoulderEncoder->Get(), WRIST_TOP_H);
				bButtonReleaseLatch = false;
				break;
			case 1: 
				arm->_Goto(600-STOP_LOW_H, pShoulderEncoder->Get(), WRIST_FULL_ROT/2.0 + 7);
				bButtonReleaseLatch = false;
				break;
			case 10:
				
				if(abs(pDriverControl->d_station_controller.GetX()) > .1 || abs(pDriverControl->d_station_controller.GetY()) > .1){
					arm->Move(pDriverControl->d_station_controller.GetX() * .5);
					arm->GetWrist()->Move(pDriverControl->d_station_controller.GetY() * .15);
					bButtonReleaseLatch = false;
				} else{
						if(!bButtonReleaseLatch){
						w_last = arm->GetWrist()->GetEncoderValue();
						arm_last = pShoulderEncoder->Get();
						bButtonReleaseLatch = true;
						
						}
					arm->Stay(arm_last, pShoulderEncoder->Get());
					arm->GetWrist()->Stay(w_last);
				maxAngle_w = arm->getMaxAngle_W(pShoulderEncoder->Get());
				if(maxAngle_w > 0.0 && maxAngle_w){
					angle_w = ((arm->GetWrist()->Get() - WRIST_X_AXIS)/WRIST_FULL_ROT)*2*M_PI;
					/*if(angle_w > maxAngle_w){
						arm->GetWrist()->_Stay(WRIST_X_AXIS+maxAngle_w);

					} else if(angle_w < -maxAngle_w){
						arm->GetWrist()->_Stay(WRIST_X_AXIS-maxAngle_w);
					}
					sprintf(buf, "maxAngle_w: %f : angle_w : %f", maxAngle_w, angle_w);
					*/
					
					
				}
				frc::DriverStation::ReportError(buf);
				}
				break;
			default:
				//arm->Move(pDriverControl->GetVectorValue(DRVARM));
				//arm->GetWrist()->Move(pDriverControl->GetVectorValue(X_AXIS));
				if(!bButtonReleaseLatch){
					w_last = arm->GetWrist()->GetEncoderValue();
					arm_last = pShoulderEncoder->Get();
					bButtonReleaseLatch = true;
				}
				arm->Stay(arm_last, pShoulderEncoder->Get());
				arm->GetWrist()->Stay(w_last);

				M_INTAKEARM.Set(pDriverControl->d_station_controller.GetX() * -.7);
		
				double d_joy_y = pDriverControl->d_station_controller.GetY();
				M_INTAKEROLLER.Set(d_joy_y * .7);


				break;
		}

		//arm->GetWrist()->Move(pDriverControl->GetVectorValue(DRVWRIST));
		
		

		//sprintf(buf, "INTAKE ARM: %f", pDriverControl->d_station_controller.GetX());
		//frc::DriverStation::ReportError(buf);
		
		//m_robotDrive.ArcadeDrive(0, 0, pDriverControl->isFullSpeed());
		m_robotDrive.ArcadeDrive(pDriverControl->GetVectorValue(X_AXIS), pDriverControl->GetVectorValue(Y_AXIS), pDriverControl->isFullSpeed());
//		sprintf(buf, "Y: %f - X: %f", -pDriverControl->GetVectorValue(Y_AXIS), pDriverControl->GetVectorValue(X_AXIS));
//		sprintf(buf, "%d", pDriverControl->getStationButton(3));
		//sprintf(buf, "Shoulder: %d %d", pShoulderEncoder->Get(), pDriverControl->getStationButton());
		
			 //sprintf(buf, "WRIST: %f : ARM: %i", arm->GetWrist()->GetEncoderValue(), pShoulderEncoder->Get());
			 
		

		//Lift->Set(-pDriverControl->GetLiftValue());

		//YawCameraController.Set(pDriverControl->GetVectorValue(X_AXIS));
		//PitchCameraController.Set(pDriverControl->GetVectorValue(Y_AXIS));
		

		// The motors will be updated every 5ms
		
		frc::Wait(0.005);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
