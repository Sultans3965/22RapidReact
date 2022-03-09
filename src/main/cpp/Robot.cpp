// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <ctre/phoenix.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/livewindow/LiveWindow.h>
#define AUTO_MODE 0
#define FORWARD_DRIVE_TIME 2.0 //seconds 

#define LEFT_FORWARD_SLOW 0.5		//Speed variables for auto
#define RIGHT_FORWARD_SLOW 0.5

using namespace frc;

/**
 * This is a demo program showing how to use Mecanum control with the
 * MecanumDrive class.
 */
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {
    // Invert the right side motors. You may need to change or remove this to
    // match your robot.
    RightFrontController.SetInverted(true);
    RightRearController.SetInverted(true);
    SelectedAutonomous = 0;

    
  }

  void AutonomousInit() override {
  LeftFrontController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  LeftRearController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
  RightFrontController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
  RightRearController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
  CurrentState = 0; 
  }


void AutonomousPeriodic()
	{

		switch(AUTO_MODE)
		{
		case 0: //Call left switch procedure
		  DriveForward((units::second_t)FORWARD_DRIVE_TIME);
			break;
    case 1:
      LowHub(); 
      break;
		default: //Decide
			break;
		}


	}// end, AutonomousPeriodic

  void TeleopPeriodic() override {
    /* Use the joystick X axis for lateral movement, Y axis for forward
     * movement, and Z axis for rotation.
     */
    double X,Y,Z;
  double powLF,powLB,powRF,powRB;
  double D1RightTrigger = 0; // Right trigger means pick up ball 
  bool D1LeftBumper = false;   // Right bumper means eject ball
  bool D2RightBumper = false;   // Right bumper means raise arms 
  bool D2LeftBumper = false;   // Left bumper means lower arms
  /*

  setup Robot Drive

   */
  X = Robot::Driver1.GetLeftX();
  Y = Robot::Driver1.GetLeftY();
  Z = Robot::Driver1.GetRightX();

  if ((X < 0.1) && (X > -0.1 )) X = 0;
  if ((Y < 0.1) && (Y > -0.1 )) Y = 0;
  if ((Z < 0.05) && (Z > -0.05 )) Z = 0;

  if ((X == 0) && (Y==0) && (Z == 0))
  {
    X = Robot::Driver2.GetLeftX();
    Y = Robot::Driver2.GetLeftY();
    Z = Robot::Driver2.GetRightX();
    if ((X < 0.1) && (X > -0.1 )) X = 0;
    if ((Y< 0.1) && (Y > -0.1 )) Y = 0;
  } 


  SmartDashboard::PutNumber("X", X);
  SmartDashboard::PutNumber("Y", Y);
  SmartDashboard::PutNumber("Z", Z);

  powLF = Y + X + Z ;
  powLB = Y - X + Z ;
  powRF = Y - X - Z ;
  powRB = Y + X - Z ;

  if((abs(powLF)>1) || 
     (abs(powLB)>1) ||
     (abs(powRF)>1) ||
     (abs(powRB)>1))
  {
    double max;
    if (abs(powLF)> abs(powLB))
      max = abs(powLF);
    else
      max = abs(powLB);
    
    if (abs(powRF)> max)
      max = abs(powRF);
    if (abs(powRB)> max)
      max = abs(powRB);
    powLF /= max;
    powLB /= max;
    powRF /= max;
    powRB /= max;
  }

  SmartDashboard::PutNumber("Left Front", powLF);
  SmartDashboard::PutNumber("Left Rear", powLB);
  SmartDashboard::PutNumber("Right Front", powRF);
  SmartDashboard::PutNumber("Right Rear", powRB);

  LeftFrontController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, powLF);
  LeftRearController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,powLB);
  RightFrontController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,powRF);
  RightRearController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,powRB);
  
  /*
  setup The Ball Pick-up
  */
  D1RightTrigger = Robot::Driver1.GetRightTriggerAxis();
  BallPickUp.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput,D1RightTrigger);
  D1LeftBumper = Robot::Driver1.GetLeftBumper();
  D2RightBumper = Robot::Driver2.GetRightBumper();
  D2LeftBumper = Robot::Driver2.GetLeftBumper();
  if (D1LeftBumper) 
  { 
   EjectBall.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,1);
  }
  else 
  {
    EjectBall.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
  }
  if  (D2RightBumper) 
  {
// Raise the arm 
    RaiseArm1.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0.5);
    RaiseArm2.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0.5);
  }
  else if (D2LeftBumper)
  {
    RaiseArm1.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,-0.5);
    RaiseArm2.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,-0.5);
  }
  else 
  {
    RaiseArm1.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
    RaiseArm2.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
  }
  }
  
 
void DriveForward(units::second_t OurTime)
	{
		double angle = 0;
    int NextState;
		units::second_t CurrentTime;
		switch(CurrentState)
		{
		case 0:
	LeftFrontController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  LeftRearController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
  RightFrontController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
  RightRearController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
		//ourGyro->Reset();
			OurTimer.Reset();
			OurTimer.Start();
			NextState = 1;
			break;
		case 1: // Drive Forward
			//angle = ourGyro->GetAngle();
			if (angle > 180)
			{
				angle = angle - 360;
			}
	    LeftFrontController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, LEFT_FORWARD_SLOW);
      LeftRearController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,LEFT_FORWARD_SLOW);
      RightFrontController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,RIGHT_FORWARD_SLOW);
      RightRearController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,RIGHT_FORWARD_SLOW);
			CurrentTime = OurTimer.Get();
			if (CurrentTime >= OurTime)
			{
				NextState = 2;
			}
			else
			{
				NextState = 1;
			}
			break;
		case 2: // Stop
			LeftFrontController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      LeftRearController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
      RightFrontController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
      RightRearController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
			NextState = 2;
			break;
		default:
			NextState = 2;
		}
		CurrentState = NextState;
	}
void LowHub(void)
	{
		double angle = 0;
    int NextState;
		units::second_t CurrentTime;
  	units::second_t MotorSpin = (units::second_t)2.0;
  	units::second_t ShootTime = (units::second_t)4.0;
  	units::second_t DriveTime = (units::second_t)9.0;

		switch(CurrentState)
		{
		case 0:
	LeftFrontController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  LeftRearController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
  RightFrontController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
  RightRearController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
		//ourGyro->Reset();
			OurTimer.Reset();
			OurTimer.Start();
			NextState = 1;
			break;
    case 1: 
      EjectBall.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,1);
			CurrentTime = OurTimer.Get();

      if (CurrentTime >= MotorSpin)
			{
				NextState = 2;
			}
			else
			{
				NextState = 1;
			}
      break;
    case 2: 
       BallPickUp.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput,1);
			CurrentTime = OurTimer.Get();
      if (CurrentTime >= ShootTime)
			{
				NextState = 3;
			}
			else
			{
				NextState = 2;
			}

      break;
		case 3: // Drive Forward
			//angle = ourGyro->GetAngle();
			if (angle > 180)
			{
				angle = angle - 360;
			}
      EjectBall.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
      BallPickUp.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);

	    LeftFrontController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, LEFT_FORWARD_SLOW);
      LeftRearController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,LEFT_FORWARD_SLOW);
      RightFrontController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,RIGHT_FORWARD_SLOW);
      RightRearController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,RIGHT_FORWARD_SLOW);
			CurrentTime = OurTimer.Get();
			if (CurrentTime >= DriveTime)
			{
				NextState = 4;
			}
			else
			{
				NextState = 3;
			}
			break;
		case 4: // Stop
			LeftFrontController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      LeftRearController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
      RightFrontController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
      RightRearController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
			NextState = 4;
			break;
		default:
			NextState = 4;
    }
		CurrentState = NextState;
	}


 private:
  static constexpr int kFrontLeftChannel = 5;
  static constexpr int kRearLeftChannel = 6;
  static constexpr int kFrontRightChannel = 3;
  static constexpr int kRearRightChannel = 4;
  static constexpr int kBallPickUpChannel = 7;
  static constexpr int kBallEjectChannel = 8;
  static constexpr int kRaiseArm1Channel = 9;
  static constexpr int kRaiseArm2Channel = 10;
/*
  frc::PWMSparkMax RightFrontControllerontLeft{kFrontLeftChannel};
  frc::PWMSparkMax m_rearLeft{kRearLeftChannel};
  frc::PWMSparkMax RightFrontControllerontRight{kFrontRightChannel};
  frc::PWMSparkMax m_rearRight{kRearRightChannel};
  */
 
  VictorSPX LeftFrontController{kFrontLeftChannel};
  VictorSPX LeftRearController{kRearLeftChannel};
  TalonSRX RightFrontController{kFrontRightChannel};  //Talon
  VictorSPX RightRearController{kRearRightChannel};
  TalonSRX BallPickUp{kBallPickUpChannel};
  TalonSRX EjectBall{kBallEjectChannel};
  TalonSRX RaiseArm1{kRaiseArm1Channel};
  TalonSRX RaiseArm2{kRaiseArm2Channel};
  /*
  frc::MecanumDrive m_robotDrive{RightFrontControllerontLeft, m_rearLeft, RightFrontControllerontRight,
                                 m_rearRight};
*/

  frc::XboxController Driver1{0};
  frc::XboxController Driver2{1};
 // frc::Joystick m_stick{kJoystickChannel};
  int SelectedAutonomous;
  int CurrentState;
  frc::LiveWindow* lw = LiveWindow::GetInstance();		//Opens control dashboard 
  frc::Timer OurTimer; 
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
