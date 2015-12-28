/******************************************************************************
 * 
 * 	Sprockets 2011 Code
 * 	Authors: 	Duane Jeffery
 * 				Matt Aceto
 *				Brendan van Ryn
 * 				Terry Shi
 * 				Stuart Sullivan
 *				Will Surmak
 * 	Last Updated:	2/3/2011
 * 
 ******************************************************************************/

/* These two values select the stopping distances during autonomous. The first is for straighline, and the second is
 * for the Y-branch. Simply replace the number beside the capitalized name and re-download the code. To do so, press
 * Ctrl+Shift+A to compile, then select FIRST->Download from the top menu. Finally, make sure to reboot the robot. */
#define STRAIGHTLINEDISTANCE 240
#define BRANCHEDLINEDISTANCE 821

/* Left = back, right = front */

// Header files
#include "WPILib.h"
#include "vision\AxisCamera.h"
#include "math.h"
#include "stdlib.h"

// Drive system definitions. Use these to specify which drive system is in use.
#define MECANUM 32
#define SWERVE 51

// Handles the tedious driverstation output overhead
// Simply pass a number and it will be sent to the driver station
#define CLEARMESSAGE DriverStationLCD::GetInstance()->Clear()
#define DISPLAYMESSAGE(b, a) DriverStationLCD::GetInstance()->Printf(DriverStationLCD::kUser_Line ## b, 1, "%d", a); DriverStationLCD::GetInstance()->UpdateLCD()
#define DISPLAYSTRING(b, a) DriverStationLCD::GetInstance()->Printf(DriverStationLCD::kUser_Line ## b, 1, a); DriverStationLCD::GetInstance()->UpdateLCD()

// Autonomous mode definitions. Use these to specify which autonomous mode to use.
#define STRAIGHT 0
#define BRANCH 1

// Determine which drive system we are using
// If the drive system is defined as MECANUM, the mecanum
// drive code is used. If it is defined as SWERVE, the swerve
// drive code is used. Otherwise, the compiler generates an error
#define DRIVESYSTEM SWERVE
#if (DRIVESYSTEM !=  MECANUM && DRIVESYSTEM != SWERVE) || !defined DRIVESYSTEM
#error "Drive system not specified"
#endif

// Ultrasonic stopping distance for autonomous
// Branch angle for y-autonomous
#define BRANCHSTOPDISTANCE 400
#define BRANCHANGLE -0.12

// Steering mode macros for the swerve drive function
#define STRAFE 0
#define TANK 1
#define MONSTER 2
#define CAR 3
#define AUTOMONSTER_LEFT 4
#define AUTOMONSTER_RIGHT 5

// Number of pulses per rotation on the steering encoders
// To acheive a pulse value, multiply an angle by this value
// AUTOALIGNSTOP is a time limit before the autoalign function should automatically exit
#define PULSES_PER_STEER (1020)
#define AUTOALIGNSTOP 1000

// Speed of claw rollers
#define CLAWROLLERSPEED 0.6

// Motor macros to be changed depending on whether we use Jaguars or Victors
#define LPDRIVEMOTOR Jaguar*
#define DRIVEMOTOR(a) Jaguar(a)

// Joystick deadbands
#define JOYSTICK_DEADBAND_X 0.0525f
#define JOYSTICK_DEADBAND_Y 0.100f

// Steering synchronization macros
#define STEERINGSYNCDEADBAND 5 // Manimum difference between encoders for synchronization to take effect
#define STEERINGSYNCDISTANCE 100 // Maximum distance between drive units before one steering motor is stopped completely
#define STEERINGSYNCDIVIDER 200 // Resolution of the steering synchronization code (smaller values means more frequent synchronization)

// When the lock button is pressed, multiply the drive speed by this to reduce chances of tipping
#define LOCKBUTTONLIMIT 0.625

// Button Macros
// An x indicates that an imaginary button was specified, such as for an untested feature
#define CLAWREVERSEBUTTON !clawStick->GetRawButton(1)
#define PIVOTPOSITIONHOME clawStick->GetRawButton(2)
#define PIVOTPOSITIONMIDDLE clawStick->GetRawButton(4)
#define PIVOTPOSITIONLOW clawStick->GetRawButton(80) // x
#define PIVOTPOSITIONFULL clawStick->GetRawButton(5)
#define EXTENDBUTTON clawStick->GetRawButton(6)
#define RETRACTBUTTON clawStick->GetRawButton(7)
#define LOCKBUTTONPRESSED leftStick->GetRawButton(2)
#define PIVOTDOWNBUTTON clawStick->GetRawButton(10)
#define PIVOTUPBUTTON clawStick->GetRawButton(11)
#define PIVOTPOSITIONHIGH clawStick->GetRawButton(30) // x
#define STEERALIGNBUTTON leftStick->GetRawButton(10)
#define MONSTERENABLEBUTTON leftStick->GetRawButton(1)
#define DEPLOYMINIBOT leftStick->GetRawButton(6)
#define RETRACTMINIBOT leftStick->GetRawButton(7)
#define SWERVEENABLEBUTTON rightStick->GetRawButton(1)
#define CARENABLEBUTTON rightStick->GetRawButton(20) // x
#define EXITSTEERALIGN leftStick->GetRawButton(11)
#define PICKUPBUTTON clawStick->GetRawButton(9)
#define DRIVEREVERSEOFF (leftStick->GetRawButton(4) || leftStick->GetRawButton(5) || leftStick->GetRawButton(3))
#define DRIVEREVERSEON (rightStick->GetRawButton(4) || rightStick->GetRawButton(5) || rightStick->GetRawButton(3))
#define PREPICKUPBUTTON clawStick->GetRawButton(8)

// Slot for second digital sidecar
#define DIGITALSIDECARTWO 6

// Autonomous travel time (how long should it strafe after the branch?)
#define AUTONOMOUSTRAVELTIME 3300

// Autonomous straight section time
#define AUTONOMOUSSTRAIGHT 1600

// Autonomous manipulation time
#define MANIPULATIONTIME 6000

// Delay between arriving at rack and deploying tube in autonomous
#define RETRACTTIME 3000

// Amount of time for the tube to be manipulated when the claw switch is triggered
#define AUTOMANIPULATETIME 1750

// Travel speed an angle along branch
#define BRANCHSPEED 0.5

// Port number macros in case things get moved around
#define RIGHTJOYSTICKPORT 1
#define LEFTJOYSTICKPORT 4
#define CLAWJOYSTICKPORT 3

#define LIGHTSENSORPORT1 1
#define LIGHTSENSORPORT2 2
#define LIGHTSENSORPORT3 3
#define LIGHTSENSORPORT4 13

#define ARMENCCHANNELA 7
#define ARMENCCHANNELB 6
#define LEFTSTEERINGENCODERA 12
#define LEFTSTEERINGENCODERB 11
#define RIGHTSTEERINGENCODERA 14
#define RIGHTSTEERINGENCODERB 13

#define EXTENDLIMITSWITCHPORT 4
#define RETRACTLIMITSWITCHPORT 8
#define PIVOTLIMITSWITCHUPPORT 10
#define PIVOTLIMITSWITCHDOWNPORT 9
#define MINIBOTLIMITSWITCHPORT 1

#define STEERINGLIMITFRONTRIGHT 2
#define STEERINGLIMITFRONTLEFT 3
#define STEERINGLIMITBACKRIGHT 4
#define STEERINGLIMITBACKLEFT 5

#define LFMOTORPORT 1
#define LRMOTORPORT 2
#define RFMOTORPORT 3
#define RRMOTORPORT 4
#define LEFTSTEERINGPORT 5
#define RIGHTSTEERINGPORT 6

#define EXTENDMOTORPORT 9
#define PIVOTMOTORPORT 10
#define CLAWTOPMOTORPORT 7
#define CLAWBOTTOMMOTORPORT 8

#define MINIBOTDEPLOYPORT 1
#define MINIBOTRETRACTPORT 2
#define BRAKEPORT 3
#define MINIBOTDEPLOYPORT2 4

#define COMPRESSORSWITCHPORT 12
#define COMPRESSORSPIKEPORT 8
#define CLAWSWITCHPORT 9

#define ULTRAPORT 3
#define ARMLIGHTSENSOR 10

#define SWERVEBLOCKENCODER1 8 
#define SWERVEBLOCKENCODER2 6 

#define AUTOSWITCHPORT 12


#define PREPICKUPEXTEND 35


// Arm positions
#define ARM_FULLUP 140
#define ARM_RACKTOP 120
#define ARM_RACKMIDDLE 30
#define ARM_RACKBOTTOM 12
#define ARM_HOME 0

// Arm extension limits
#define ARMEXTENDHOME 33
#define ARMEXTENDRACK 57

// Movement speed limits
#define ARMLIMITHEIGHT 25
#define ARMLIMITFACTOR 0.8

// Arm approach distances and speeds
#define ARMAPPROACHUP 25
#define ARMAPPROACHDOWN 25
#define ARMUPSPEED 1.0
#define ARMDOWNSPEED -1.0
#define ARMEXTENDSPEED 1
#define ARMDEADBAND 6

// Steering approach distance
#define STEERING_APPROACH_DISTANCE 100.0f
#define STEERING_DEADBAND 0.02f

// Function prototypes
float Abs(float); // Absolute value function

class MecanumDrive
{
public:
	LPDRIVEMOTOR lfDrive; // Create a pointer to a Jag/Victor for the LF Motor
	LPDRIVEMOTOR lrDrive; // Create a pointer to a Jag/Victor for the LR Motor
	LPDRIVEMOTOR rfDrive; // Create a pointer to a Jag/Victor for the RF Motor
	LPDRIVEMOTOR rrDrive; // Create a pointer to a Jag/Victor for the RR Motor
	Victor* leftSteer; // Create a pointer to a Jag/Victor for the left steering motor
	Victor* rightSteer; // Create a pointer to a Jag/Victor for the right steering motor
	Victor* extendMotor; // Create a pointer to a victor for the arm extend motor
	Victor* pivotMotor; // Create a pointer to a victor for the arm pivot motor
	Victor* topclawmotor; // Create a pointer to a victor for the top claw motor
	Victor* bottomclawmotor; // Create a pointer to a victor for the bottom claw motor
	Encoder *leftSteeringEncoder; // Create a pointer to an encoder for the left steering motor
	Encoder *rightSteeringEncoder; // Create a pointer to an encoder for the right steering motor
	RobotDrive *robotDrive; // Create a pointer to a default drive system
	DigitalInput *frontleftSteerLimit; // These limit switches prevent wheel rotation beyond limits
	DigitalInput *frontrightSteerLimit; // 
	DigitalInput *backleftSteerLimit; //
	DigitalInput *backrightSteerLimit; //
	DigitalInput *ClawSwitch; // Limit switch on claw to detect tube presence
	float output_speed; // Used by swerve drive
	bool flag2, flag3; // Various flags used by swerve drive
	int DriveReverse; // Either 1 or -1 for forward or reversed drive, respectively
	float y_sign; // Sign of joystick y-movement for the drive reverse feature
	int oldLeftEncoderValue, oldRightEncoderValue; // Hold the values of the encoders since the last time the swerve-synchronizing code was called
	int trip; // Increment each time third sensor is tripped, to check for consistency and prevent noise from being a problem

public:
	MecanumDrive(UINT32 lfMotor, UINT32 lrMotor, UINT32 rfMotor, UINT32 rrMotor)
	{
		lfDrive = new DRIVEMOTOR(lfMotor); // Create drive motor pointers
		lrDrive = new DRIVEMOTOR(lrMotor);
		rfDrive = new DRIVEMOTOR(rfMotor);
		rrDrive = new DRIVEMOTOR(rrMotor);
		extendMotor = new Victor(EXTENDMOTORPORT); // Create pointer to arm extend motor
		pivotMotor = new Victor(PIVOTMOTORPORT); // Create pointer to arm pivot motor
		topclawmotor = new Victor(CLAWTOPMOTORPORT); //Create a pointer to the top claw motor
		bottomclawmotor = new Victor(CLAWBOTTOMMOTORPORT); // Create a pointer to the bottom claw motor
		robotDrive = new RobotDrive(lfMotor, lrMotor, rfMotor, rrMotor); // The mecanum function assumes that a robot drive with this setup has been created
		frontrightSteerLimit = new DigitalInput(DIGITALSIDECARTWO, STEERINGLIMITFRONTRIGHT); // Create steering limit switches
		frontleftSteerLimit = new DigitalInput(DIGITALSIDECARTWO, STEERINGLIMITFRONTLEFT);
	    backrightSteerLimit = new DigitalInput(DIGITALSIDECARTWO, STEERINGLIMITBACKRIGHT);
		backleftSteerLimit = new DigitalInput(DIGITALSIDECARTWO, STEERINGLIMITBACKLEFT);
		ClawSwitch = new DigitalInput(DIGITALSIDECARTWO, CLAWSWITCHPORT); // Create a limit switch to detect tube presence in claw 

		leftSteeringEncoder = new Encoder(LEFTSTEERINGENCODERA, LEFTSTEERINGENCODERB); // Encoders to track rotation positions of motors
		rightSteeringEncoder = new Encoder(RIGHTSTEERINGENCODERA, RIGHTSTEERINGENCODERB);
		leftSteer = new Victor(LEFTSTEERINGPORT); // Motor controller for steering movement
		rightSteer = new Victor(RIGHTSTEERINGPORT);
		DriveReverse = 1; // Store the state of the drive reverse feature (1 = forward)
		reset_variables(); // Reset and start all encoders and clear various state variables

	}//End of MecanumDrive constructor

	// Track the line and check the ultrasonic sensor distance for autonomous
	int MoveSwerve(int leftSensor, int rightSensor, int endOfLineSensor, int armPivot, AnalogChannel *ultraDistance, int AutoMode)
	{
		int AUTOSTOPDISTANCE; // Stores the distance to stop (as read by the ultrasonic sensor)
		
		if(AutoMode) AUTOSTOPDISTANCE = STRAIGHTLINEDISTANCE; // Straightline stopping distance
		else AUTOSTOPDISTANCE = BRANCHEDLINEDISTANCE; // Y-branch stopping distance
		
		// If we are within the decided distance from the wall
		if(ultraDistance->GetVoltage() * 1000 < AUTOSTOPDISTANCE)
		{
			// Increment a trip counter
			trip++;
			
			// If we have been within the given distance for long enough, we can hopefully
			// Rule out error on the part of the sensor, and stop, returning our state
			if(trip > 320)
			{
				lfDrive->Set(0);
				lrDrive->Set(0);
				rfDrive->Set(0);
				rrDrive->Set(0);
				
				// Robot should stop moving
				return 1;
			}
		}
		
		// Anything too short to reach 320 was probably just interference from debris, so lower the trip value again
		else if(trip > 0) trip--;
		
		// If the right sensor hits the line
		if (rightSensor)
		{
			// Stop the left motors
			lfDrive->Set(0);
			lrDrive->Set(0);
			rfDrive->Set(0.9);
			rrDrive->Set(0.9);
		}

		// If the right sensor has not hit the line, but the left one has,
		// set the motors to turn right
		else if (leftSensor)
		{
			lfDrive->Set(0.9);
			lrDrive->Set(0.9);
			rfDrive->Set(0);
			rrDrive->Set(0);
		}

		// If neither sensor has hit the line, move forward normally
		else
		{
			lfDrive->Set(0.4);
			lrDrive->Set(0.4);
			rfDrive->Set(0.4);
			rrDrive->Set(0.4);
		}

		// Robot should continue moving
		return 0;
	}

	// Swerve drive function
	void DriveSwerve(Joystick *leftStick, Joystick *rightStick, Joystick *clawStick, DigitalInput *ArmPivotLimitDown, DigitalInput *ArmPivotLimitUp, DigitalInput *ArmRetractLimit, DigitalInput *ArmExtendLimit, Encoder *ArmEncoder, int ArmTarget, int ArmPosition, char AutonomousOverride, int ArmPickup, int ArmDeploy, int ArmExtendTarget)
	{
		static int monsterenabled = 0, carenabled = 0, align = 0, rotateSwitch = 0; // Steering modes and alignment state variables
		static int syncDivider = 0; // We only call run the synchronization code periodically
		static int aligntime = 0; // We need to give the wheels a little extra time after they reach their goal
		float right_x, right_y, left_x, left_y, clawy; // Displacements of joystick axes
		int SteerMode; // Between TANK, MONSTER, STRAFE, AUTOMONSTER(RIGHT/LEFT), and CAR
		
		if(DRIVEREVERSEON) DriveReverse = -1;
		if(DRIVEREVERSEOFF) DriveReverse = 1;
		
		if (AutonomousOverride == 1)
		{
			right_x = left_x = -0.32;
			right_y = left_y = 0.45;
		}

		else if (AutonomousOverride == 3)
		{
			right_x = left_x = 0.32;
			right_y = left_y = 0.45;
		}

		else if (AutonomousOverride == 2)
		{
			// Return wheels to home and stop
			right_x = 0.01;
			right_y = 0.01;
			left_x = 0.01;
			left_y = 0.01;
		}

		// If the wheels are turned full left, indicating the end of the first stage of auto-align,
		// reset the encoders so that this new position is considered center. Continue to stage three
		else if (align == 2)
		{
			align = 3;
			DriverStationLCD::GetInstance()->Clear();
			DriverStationLCD::GetInstance()->Printf(DriverStationLCD::kUser_Line1, 1, "mode 2");
			DriverStationLCD::GetInstance()->UpdateLCD();
			rightSteeringEncoder->Reset();
			leftSteeringEncoder->Reset();
		}

		// If we are in the final stage of wheel alignment, we set the goal to "full-right", which should translate to centre
		// because the wheels are turned full-left. 
		// Once the wheels are centred, we reset the encoders again and exit wheel-alignment mode
		else if (align >= 3)
		{
			DriverStationLCD::GetInstance()->Clear();
			DriverStationLCD::GetInstance()->Printf(DriverStationLCD::kUser_Line1, 1, "mode 3");
			DriverStationLCD::GetInstance()->Printf(DriverStationLCD::kUser_Line2, 1, "%d",
					leftSteeringEncoder->Get());
			DriverStationLCD::GetInstance()->Printf(DriverStationLCD::kUser_Line3, 1, "%d",
					rightSteeringEncoder->Get());
			DriverStationLCD::GetInstance()->UpdateLCD();

			right_x = 1.0;
			left_x = 1.0;
			right_y = 0.01;
			left_y = 0.01;

			// If the steering motors have finished centering, reset them to center
			// and exit auto-align mode
			if(leftSteeringEncoder->Get() > AUTOALIGNSTOP && rightSteeringEncoder->Get() < -AUTOALIGNSTOP) 
			{
				aligntime--;
				if(aligntime < 1)
				{
					leftSteeringEncoder->Reset();
					rightSteeringEncoder->Reset();
					align = 0;
				}
			}
			
			else aligntime++;
		}

		else
		{
			// Read position of left and right drive joysticks
			right_x = rightStick->GetRawAxis(1);
			right_y = rightStick->GetRawAxis(2);
			left_x = leftStick->GetRawAxis(1);
			left_y = leftStick->GetRawAxis(2);
		}
		
		// Get position of claw y-axis
		clawy = clawStick->GetRawAxis(2);

		// If the claw stick is less than the deadband, ignore it
		if (Abs(clawy) <= JOYSTICK_DEADBAND_Y)
			clawy = 0.0f;

		// Check the monster/car/swerve buttons
		if (SWERVEENABLEBUTTON)
			monsterenabled = 0, carenabled = 0;
		else if (MONSTERENABLEBUTTON)
			monsterenabled = 1, carenabled = 0;
		else if (CARENABLEBUTTON)
			monsterenabled = 0, carenabled = 1;

		// If the auto-align button is pressed, set the align flag and enter strafe mode
		if (STEERALIGNBUTTON)
		{
			align = 1;
			SteerMode = STRAFE;
			monsterenabled = 0;
			carenabled = 0;
		}
		
		// In case something goes wrong, this button exits auto-align mode prematurely
		if (EXITSTEERALIGN && align > 0)
		{
			leftSteeringEncoder->Reset();
			rightSteeringEncoder->Reset();
			align = 0;
		}

		// If the arm should extend, set the motor forward
		// If the arm should retract, set the motor backward
		// Otherwise, stop any previous motion
		if ((EXTENDBUTTON || ArmPickup == 1) && ArmExtendLimit->Get())
			extendMotor->Set(1);
		else if ((RETRACTBUTTON || ArmDeploy || ArmPickup == 2) && ArmRetractLimit->Get())
			extendMotor->Set(-1);
		else if(ArmExtendTarget == -1) // *&^%$#@!~`
			extendMotor->Set(0);

		// If the arm should pivot up, and the arm has not hit the limit switch, set the motor forward
		// If the arm should pivot down, and the arm has not hit the limit switch, set the motor backward
		// Otherwise, stop any previous motion
		if (PIVOTUPBUTTON && ArmPivotLimitUp->Get())
			pivotMotor->Set(ARMUPSPEED);
		else if (PIVOTDOWNBUTTON && ArmPivotLimitDown->Get())
			pivotMotor->Set(ARMDOWNSPEED);
		else if (ArmTarget == -1)
			pivotMotor->Set(0);

		// If the arm hits the lower limit switch, reset the encoder
		if (!ArmPivotLimitDown->Get())
			ArmEncoder->Reset();
		
		if(!ClawSwitch->Get() && clawy > 0 && !CLAWREVERSEBUTTON) clawy = 0;
		
		/*// If the tube triggers the claw switch, enter rotate mode
		// Otherwise, exit and reset the time counter
		if(!ClawSwitch->Get()) rotateSwitch = 1;
		else rotateSwitch = 0, rotateTime = 0;*/
		
		// If it's in auto-pickup mode, the rollers should suck in
		// Unless the switch has been hit
		if(ArmPickup == 1 && !rotateSwitch)
		{
			topclawmotor->Set(-1);
			bottomclawmotor->Set(1);
		}
		
		/*// If the switch has been hit, manipulate the tube for a period of time
		// to point it upward
		else if(ArmPickup && rotateTime < AUTOMANIPULATETIME) 
		{
			rotateTime++;
			topclawmotor->Set(-1);
			bottomclawmotor->Set(-1);
		}*/
		
		// If it's in auto-deploy mode, the rollers should eject
		else if(ArmDeploy)
		{
			topclawmotor->Set(1);
			bottomclawmotor->Set(-1);
		}
		
		// If the trigger on the claw stick is pressed, set both motors in the same
		// direction, according to the joystick axis
		else if(!CLAWREVERSEBUTTON)
		{
			topclawmotor->Set(-clawy);
			bottomclawmotor->Set(clawy);
		}
		
		else
		{
			// Otherwise, spin the motors inverse to each other.
			// The motor that spins outward travels more slowly to
			// prevent the tube from sliding out.
			if(clawy > 0)
			{
				topclawmotor->Set(-clawy);
				bottomclawmotor->Set(-clawy / 2);
			}
			
			else
			{
				topclawmotor->Set(-clawy / 2);
				bottomclawmotor->Set(-clawy);
			}
		}

		// If both joysticks have been tilted sideways by more than the dead band amount and monster mode is enabled, set to monster mode
		// Otherwise, if both joysticks have been tilted sideways by more than the dead band amount, set to strafe mode
		// Otherwise, set to tank mode
		if ((Abs(right_x) > JOYSTICK_DEADBAND_X && Abs(left_x) > JOYSTICK_DEADBAND_X) || LOCKBUTTONPRESSED)
		{
			if (monsterenabled)
				SteerMode = MONSTER;
			else if (carenabled)
				SteerMode = CAR;
			else
				SteerMode = STRAFE;
		} 
		
		// If the left joystick points forward and the right points backward, enter full-angle monster clockwise
		else if(left_y > JOYSTICK_DEADBAND_Y && right_y < -JOYSTICK_DEADBAND_Y)
		{
			SteerMode = AUTOMONSTER_RIGHT;
		}
		
		// If the right joystick points backward and the left joystick points backward, enter full-angle monster counter-clockwise
		else if(left_y < -JOYSTICK_DEADBAND_Y && right_y > JOYSTICK_DEADBAND_Y)
		{
			SteerMode = AUTOMONSTER_LEFT;
		}
		
		else SteerMode = TANK;

		//*******************************************************************************************
		//STEERING ALIGNMENT
		//*******************************************************************************************
		// If steering alignment buttons are held on joystick and the auto-align mode is not running
		if (rightStick->GetRawButton(8) && rightStick->GetRawButton(9) && !align)
		{

			if (rightStick->GetRawButton(6))
			{ //move left steering to right
				if (backrightSteerLimit->Get())
					leftSteer->Set(-0.4);
				else
					leftSteer->Set(0);

				leftSteeringEncoder->Stop();
				leftSteeringEncoder->Reset();
				flag2 = true; //***j
			}

			else if (rightStick->GetRawButton(7))
			{ //move lrft steering to left
				if (backleftSteerLimit->Get())
					leftSteer->Set(0.4);
				else
					leftSteer->Set(0);

				leftSteeringEncoder->Stop();
				leftSteeringEncoder->Reset();
				flag2 = true; //***j
			}

			else if (flag2)
			{
				leftSteer->Set(0);
				leftSteeringEncoder->Start();
				flag2 = false; //***j
			}

			if (rightStick->GetRawButton(10))
			{ //move right steering to right
				if (frontrightSteerLimit->Get())
					rightSteer->Set(-0.4);
				else
					rightSteer->Set(0);

				rightSteeringEncoder->Stop();
				flag3 = true;
				rightSteeringEncoder->Reset();
			} else if (rightStick->GetRawButton(11))
			{ //move right steering to left
				if (frontleftSteerLimit->Get())
					rightSteer->Set(0.4);
				else
					rightSteer->Set(0);

				rightSteeringEncoder->Stop();
				flag3 = true;
				rightSteeringEncoder->Reset();
			} else if (flag3)
			{
				rightSteer->Set(0);
				rightSteeringEncoder->Start();
				flag3 = false;
			}
		} //end of if steering alignment buttons are held on joystick
		
		// If auto-align has been initiated, move until both limit switches are hit
		else if (align == 1)
		{

			DriverStationLCD::GetInstance()->Clear();
			DriverStationLCD::GetInstance()->Printf(DriverStationLCD::kUser_Line1, 1, "mode 1");
			DriverStationLCD::GetInstance()->UpdateLCD();

			if (frontleftSteerLimit->Get())
				rightSteer->Set(0.5);
			else
				rightSteer->Set(0);

			if (backrightSteerLimit->Get())
				leftSteer->Set(-0.5);
			else
				leftSteer->Set(0);

			if (!rightSteer->Get() && !leftSteer->Get())
				align = 2;
		}

		//if steering position must come from drive joystick position (x,y)
		else
		{
			//read position of each steering encoder
			float leftSteeringEncoderVal = leftSteeringEncoder->Get();
			float rightSteeringEncoderVal = rightSteeringEncoder->Get();
			float leftSyncPercent = 1.0f; // These are used to scale the steering motor speeds
			float rightSyncPercent = 1.0f; // to synchronize them
			
			//CLEARMESSAGE;
			//DISPLAYMESSAGE(2, (int)(leftSteeringEncoderVal));
			//DISPLAYMESSAGE(3, (int)(rightSteeringEncoderVal));
			
			//normalize the values (convert from counted integer >> real number from +1 to -1)
			leftSteeringEncoderVal = leftSteeringEncoderVal / PULSES_PER_STEER;
			rightSteeringEncoderVal = rightSteeringEncoderVal / PULSES_PER_STEER;

			float leftgoal = 0;
			float rightgoal = 0;
			
			syncDivider++; // Count another call
			
			// If enough of a delay has been incurred already, perform the synchronization code
			if(syncDivider > STEERINGSYNCDIVIDER)
			{
				// Reset the divider
				syncDivider = 0;
				
				// Determine which steering unit is turning faster
				int deltaLeft;
				int deltaRight;
				
				deltaLeft = oldLeftEncoderValue - (int)leftSteeringEncoderVal;
				deltaRight = oldRightEncoderValue - (int)rightSteeringEncoderVal;
				oldLeftEncoderValue = (int)leftSteeringEncoderVal;
				oldRightEncoderValue = (int)rightSteeringEncoderVal;
				
				if(abs(deltaLeft) > abs(deltaRight) + STEERINGSYNCDEADBAND && abs(deltaRight) > STEERINGSYNCDEADBAND)
				{
					// Left is moving too fast, so adjust
					leftSyncPercent = (float)abs(oldLeftEncoderValue - oldRightEncoderValue) / STEERINGSYNCDISTANCE;
					
					if(leftSyncPercent > 1.0f) leftSyncPercent = 1.0f;
					else if(leftSyncPercent < 0.0f) leftSyncPercent = 0.0f;
					
					leftSyncPercent = 1.0f - leftSyncPercent;
				}
				
				else if(abs(deltaRight) > abs(deltaLeft) + STEERINGSYNCDEADBAND && abs(deltaLeft) > STEERINGSYNCDEADBAND)
				{
					// Right is moving too fast, so adjust
					rightSyncPercent = (float)abs(oldLeftEncoderValue - oldRightEncoderValue) / STEERINGSYNCDISTANCE;
										
					if(rightSyncPercent > 1.0f) rightSyncPercent = 1.0f;
					else if(rightSyncPercent < 0.0f) rightSyncPercent = 0.0f;
										
					rightSyncPercent = 1.0f - rightSyncPercent;
				}
			}
			
			if (SteerMode == TANK)
			{
				//point left wheels straignt towards the wide front
				leftgoal = 0;
				rightgoal = 0;
				
				// Reverse the drive if necessary (DriveReverse is either 1 or -1). This is only for tank
				left_y *= DriveReverse;
				right_y *= DriveReverse;
						
				// Swap the left and right drive settings if the drive has been reversed
				if(DriveReverse == -1)
				{
					float temp;
					
					temp = left_x;
					left_x = right_x;
					right_x = temp;
				}
						
				// If the arm is above a certain height, reduce movement sensitivity
				if(ArmEncoder->Get() > ARMLIMITHEIGHT)
				{
					lfDrive->Set(left_y * ARMLIMITFACTOR);
					lrDrive->Set(left_y * ARMLIMITFACTOR);
					rfDrive->Set(right_y * ARMLIMITFACTOR);
					rrDrive->Set(right_y * ARMLIMITFACTOR);
				}
				
				// Otherwise, don't limit speed
				else
				{
					lfDrive->Set(left_y);
					lrDrive->Set(left_y);
					rfDrive->Set(right_y);
					rrDrive->Set(right_y);
				}
			}

			else if (SteerMode == STRAFE || SteerMode == MONSTER || SteerMode
					== CAR || SteerMode == AUTOMONSTER_LEFT || SteerMode == AUTOMONSTER_RIGHT)
			{
				float deflection = 0;
				float steer_angle = 0;
				short quadrant = 0;
				float y_avg = 0;
				
				y_avg = (right_y + left_y) / 2;
				if(y_avg == 0.0f) y_avg = JOYSTICK_DEADBAND_Y / 100;
				y_sign = y_avg / Abs(y_avg); y_sign *= -1;

				float x_avg;
				x_avg = (left_x + right_x) / 2;
				
				if(Abs(y_avg) > 0.8 && SteerMode == MONSTER) 
				{
					SteerMode = CAR;
				}
				
				//determine output speed based on net joyustick deflection
				deflection = sqrt((y_avg * y_avg) + (x_avg * x_avg)); //Pythagorean Theorem

				//keep number within range. Ignore corners of joystick motion
				if (deflection > 1)
				{
					deflection = 1;
				} 
				
				else if (deflection < -1)
				{
					deflection = -1;
				}
				
				// If we are in auto-monster, the deflection should be set to the difference between the two joystick positions
				if(SteerMode == AUTOMONSTER_LEFT || SteerMode == AUTOMONSTER_RIGHT) deflection = Abs(left_y - right_y) / 2;
				
				if (align >= 1)
				{ //no drive during alignment mode
					deflection = 0;
				}
				
				// If the arm is above a certain height, reduce drive sensitivity
				if(ArmEncoder->Get() > ARMLIMITHEIGHT) deflection *= ARMLIMITFACTOR;
				
				//determine steering position based on joystick angle (measured in radians)
				steer_angle = atan(Abs(x_avg) / Abs(y_avg)); //trigonometry Tan-1 = Opp / Adj

				//conversion from radians to degrees
				steer_angle = steer_angle * (180 / 3.14159);

				//convert to normalized value (between forward >> 0.85 and sideways >> 0)
				steer_angle = (steer_angle * 0.85) / 90;
				
				// Reverse drive direction if necessary
				deflection *= DriveReverse;
								
				// If the lock button is pressed, we use a special quadrant
				if(LOCKBUTTONPRESSED)
				{
					quadrant = 7;
					SteerMode = STRAFE;
					deflection *= LOCKBUTTONLIMIT;
				}
				
				//if joysticks are pointed in the right straight deadband
				else if(x_avg > 0 && Abs(y_avg) <= JOYSTICK_DEADBAND_Y)
				{
					quadrant = 6;
				}
				
				//if joysticks are pointed in the left straight deadband
				else if (x_avg < 0 && Abs(y_avg) <= JOYSTICK_DEADBAND_Y)
				{
					quadrant = 5;
				}
				
				//if joysticks are pointed in 4th quadrant
				else if (x_avg > 0 && y_avg >= 0)
				{
					//to make driving easier, when the arm is up, the arm side becomes the "front"
					//when the arm is down, the kicker side becomes the "front"
					quadrant = 4;
				}
				
				//if joysticks are pointed in 3rd quardant
				else if (x_avg < 0 && y_avg > 0)
				{ 
				quadrant = 3;
				}
				
			//if joysticks are pointed in 2nd quardant
			else if (x_avg < 0 && y_avg <= 0)
			{
				quadrant = 2;
			}
				
			//if joysticks are pointed in 1st quadrant
			else
			{
				quadrant = 1;
			}

			//set direction to drive and angle to steering based on quardant of motion
			if (quadrant == 4)
			{
				rfDrive->Set(deflection);
				rrDrive->Set(deflection);
				lfDrive->Set(deflection);
				lrDrive->Set(deflection);
				leftgoal = steer_angle * -1;
				rightgoal = steer_angle;
			}
			
			else if (quadrant == 3)
			{
				rfDrive->Set(deflection);
				rrDrive->Set(deflection);
				lfDrive->Set(deflection);
				lrDrive->Set(deflection);
				leftgoal = steer_angle;
				rightgoal = steer_angle * -1;
			}
			
			else if (quadrant == 2)
			{
				rfDrive->Set(deflection * -1);
				rrDrive->Set(deflection * -1);
				lfDrive->Set(deflection * -1);
				lrDrive->Set(deflection * -1);
				leftgoal = steer_angle * -1;
				rightgoal = steer_angle;
			}
			
			else if (quadrant == 1)
			{
				rfDrive->Set(deflection * -1);
				rrDrive->Set(deflection * -1);
				lfDrive->Set(deflection * -1);
				lrDrive->Set(deflection * -1);
				leftgoal = steer_angle;
				rightgoal = steer_angle * -1;
			}
			
			else if (quadrant == 5)
			{
				if (align >= 1 || Abs(leftSteer->Get()) > 0.05)
				{
					rfDrive->Set(0);
					rrDrive->Set(0);
					lfDrive->Set(0);
					lrDrive->Set(0);
					DISPLAYSTRING(4, "Wheels turning");
				}
				
				else
				{
					rfDrive->Set(deflection * -1);
					rrDrive->Set(deflection * -1);
					lfDrive->Set(deflection * -1);
					lrDrive->Set(deflection * -1);
					DISPLAYSTRING(4, "Wheels stopped");
				}
				
				leftgoal = -0.85;
				rightgoal = 0.85;
			}
			
			else if (quadrant == 6)
			{
				if(align >= 1 || Abs(leftSteer->Get()) > 0.05)
				{
					rfDrive->Set(0);
					rrDrive->Set(0);
					lfDrive->Set(0);
					lrDrive->Set(0);
					DISPLAYSTRING(4, "Wheels turning");
				}
				else
				{
					rfDrive->Set(deflection * -1);
					rrDrive->Set(deflection * -1);
					lfDrive->Set(deflection * -1);
					lrDrive->Set(deflection * -1);
					DISPLAYSTRING(4, "Wheels stopped");
				}
				
				leftgoal = 0.85;
				rightgoal = -0.85;
			}
			
			// If we are in lock-mode
			else if(quadrant == 7)
			{
				// If auto-align is enabled or the steering is still turning
				// stop the drive motors
				if(align >= 1 || Abs(leftSteer->Get()) > 0.05)
				{
					rfDrive->Set(0);
					rrDrive->Set(0);
					lfDrive->Set(0);
					lrDrive->Set(0);
					DISPLAYSTRING(4, "Wheels turning");
				}
				
				// Otherwise, use the direct joystick x-values to set the motor speed
				else
				{
					rfDrive->Set(-x_avg);
					rrDrive->Set(-x_avg);
					lfDrive->Set(-x_avg);
					lrDrive->Set(-x_avg);
					DISPLAYSTRING(4, "Wheels stopped");
				}
				
				// Set goal position for steering to full right
				leftgoal = 0.85;
				rightgoal = -0.85;
			}
		} //end of if steering in strafe mode
			
		//if MONSTER is steermode, invert the leftsteer goal
		if(SteerMode == MONSTER)
		{
			leftgoal *= -1;
			leftgoal *= DriveReverse;
			rightgoal *= DriveReverse;
			leftgoal *= y_sign;
			rightgoal *= y_sign;
			leftgoal *= 0.75;
			rightgoal *= 0.75;
		}
		
		// In the case of either auto-monster, the deflection has already been calculated,
		// so simply set the wheels to point either left and right or right and left in
		// monster style
		else if(SteerMode == AUTOMONSTER_LEFT)
		{
			leftgoal = -0.58;
			rightgoal = -0.58;
			leftgoal *= DriveReverse;
			rightgoal *= DriveReverse;
		}
		
		else if(SteerMode == AUTOMONSTER_RIGHT)
		{
			leftgoal = 0.58;
			rightgoal = 0.58;
			leftgoal *= DriveReverse;
			rightgoal *= DriveReverse;
		}
		
		// Otherwise, if car is enabled, centre the rear wheels
		else if(SteerMode == CAR)
		{
			if(lfDrive->Get() > 0)
			{
				rightgoal = 0.0f;
				leftgoal = leftgoal /2;
			}
			else
			{
				leftgoal = 0.0f;
				rightgoal = rightgoal /2;
			}
		}

		// If the drive has been reversed, the angle of the strafe should also be inverted
		//leftgoal *= DriveReverse;
		//rightgoal *= DriveReverse;
		
		//if left steering has reached it's target position
		if(Abs(leftSteeringEncoderVal - leftgoal) < STEERING_DEADBAND)
		{
			//turn left steering motor off
			leftSteer->Set(0);
		}

		//if left steering motor is expected to move
		else
		{
			float speedincrement = 1/STEERING_APPROACH_DISTANCE;
			//if left steering motor must move left to reach target
			if(leftSteeringEncoderVal < leftgoal)
			{
				// If the left-rear limit switch is hit and we're travelling left, stop
				if(!backleftSteerLimit->Get()) leftSteer->Set(0);

				//if left steering motor is within approach distance
				else if(leftSteeringEncoderVal + (STEERING_APPROACH_DISTANCE/PULSES_PER_STEER) >= leftgoal)
				{
					//adjust speed proportinal to distance from target
					//printf("Close... Going Right...\r\n");
					leftSteer->Set(speedincrement * Abs((PULSES_PER_STEER*leftgoal) - (PULSES_PER_STEER*leftSteeringEncoderVal)));
				}
				//if left steering motor is far from target
				else
				{
					//go top speed
					//printf("Far... Going Right...\r\n");
					leftSteer->Set(0.9 * leftSyncPercent);
				}
			}

			//if left steering motor must move right to reach target
			else
			{
				// If the steering motor must move right and the right-rear limit switch it hit, stop
				if(!backrightSteerLimit->Get()) leftSteer->Set(0);

				//if left steering motor is within approach distance
				else if(leftSteeringEncoderVal - (STEERING_APPROACH_DISTANCE/PULSES_PER_STEER) <= leftgoal)
				{
					//adjust speed proportinal to distance from target
					//printf("Close... Going Left...\r\n");
					leftSteer->Set(-1 * speedincrement * Abs((PULSES_PER_STEER*leftgoal) - (PULSES_PER_STEER*leftSteeringEncoderVal)));
				}
				//if left steering motor is far from target
				else
				{
					//go top speed
					//printf("Far... Going Left...\r\n");
					leftSteer->Set(-0.9 * leftSyncPercent);
				}
			}
		} //end of if left steering motor is expected to move


		//if right steering has reached it's target position
		if(Abs(rightSteeringEncoderVal - rightgoal) < STEERING_DEADBAND)
		{
			//turn right steering motor off
			rightSteer->Set(0);
		}

		//if right steering motor is expected to move
		else
		{
			float speedincrement = 1/STEERING_APPROACH_DISTANCE;

			//if right steering motor must move left to reach target
			if(rightSteeringEncoderVal < rightgoal)
			{
				// If the front steering motor must move left and the front-left limit switch is triggered, stop
				if(!frontleftSteerLimit->Get()) rightSteer->Set(0);

				//if right steering motor is within approach distance
				else if(rightSteeringEncoderVal + STEERING_APPROACH_DISTANCE >= rightgoal)
				{
					//adjust speed proportinal to distance from target
					rightSteer->Set(speedincrement * Abs((PULSES_PER_STEER*rightgoal) - (PULSES_PER_STEER*rightSteeringEncoderVal)));
				}
				//if right steering motor is far from target
				else
				{
					//go top speed
					rightSteer->Set(0.9 * rightSyncPercent);
				}

			}
			//if right steering motor must move right to reach target
			else
			{
				// If the front steering motor must move right and the front-right limit switch is triggered, stop
				if(!frontrightSteerLimit->Get()) rightSteer->Set(0);

				//if right steering motor is within approach distance
				if(rightSteeringEncoderVal - STEERING_APPROACH_DISTANCE <= rightgoal)
				{
					//adjust speed proportinal to distance from target
					rightSteer->Set(-1 * speedincrement * Abs((PULSES_PER_STEER*rightgoal) - (PULSES_PER_STEER*rightSteeringEncoderVal)));
				}
				//if right steering motor is far from target
				else
				{
					//go top speed
					rightSteer->Set(-0.9 * rightSyncPercent);
				}
			}
		} //end of if right steering motor is expected to move
		// Display steering information
		/*CLEARMESSAGE;
		DISPLAYMESSAGE(1, (int)leftgoal * 100);
	    DISPLAYMESSAGE(2, (int)rightgoal * 100);
	    DISPLAYMESSAGE(3, (int)(lfDrive->Get()*100.0f));
	    DISPLAYMESSAGE(4, (int)(rrDrive->Get()*100.0f));*/
		//DISPLAYMESSAGE(3, SteerMode);
		//DISPLAYMESSAGE(4, monsterenabled && carenabled);
	} //end of if steering position must come from joystick position (x,y)
} // End of DriveSwerve function

// Reset swerve-drive related variables
void reset_variables(void)
{
	output_speed = 0;
	oldLeftEncoderValue = 0;
	oldRightEncoderValue = 0;
	flag2 = false;
	flag3 = false;
	leftSteeringEncoder->Reset(); //***i
	leftSteeringEncoder->Start(); //***i
	rightSteeringEncoder->Reset(); //***i
	rightSteeringEncoder->Start(); //***i
} // End of reset_variables

// End of "which drive system are we using?"
}; // End of MecanumDrive class

class SimpleTracker : public SimpleRobot
{
	MecanumDrive *myRobot; // Pointer to drive system
	Joystick *rightStick; // Right joystick
	Joystick *leftStick; // Left joystick
	Joystick *clawStick; // Claw control joystick
	DigitalInput *LightSensor1; // Light sensors - Left sensor
	DigitalInput *LightSensor2; // - End of line sensor
	DigitalInput *LightSensor3; // - Right sensor
	DigitalInput *LightSensor4; // Reverse sensor
	Encoder *ArmEncoder; // Encoder for arm
	Encoder *BlockEncoder;// Encoder for the steering block
	DigitalInput *ArmRetractLimit; // Telescoping retraction limit switch
	DigitalInput *ArmExtendLimit; // Telescoping extention limit switch
	DigitalInput *ArmPivotLimitDown; // Pivoting limit switch (home)
	DigitalInput *ArmPivotLimitUp; // Pivoting limit switch (full up)
	DigitalInput *MinibotLimit; // Minibot deployment limit switch
	DigitalInput *autonomousSetting; // A jumper to select autonomous modes
	DigitalInput *ArmLightSensor; //The light sensor used for the arm (a getto encoder)
	Victor *ArmExtend; // Arm Extend Motor
	Victor *ArmPivot; // Arm pivot motor
	Victor* topclawmotor; // Create a pointer to a victor for the top claw motor
	Victor* bottomclawmotor; // Create a pointer to a victor for the bottom claw motor
	Solenoid *minibotDeploy; // Pointer to solenoid to deploy minibot
	Solenoid *minibotRetract; // Pointer to solenoid to retract minibot 
	Solenoid *engageBrake; // Pointer to solenoid to engage the brake
	Solenoid *minibotDeploy2; // Pointer to second solenoid to deploy minibot
	AnalogChannel *autoSwitch; // Pointer to autonomous analog switch
	Compressor *theCompressor; // Pointer to compressor
	Relay *compressor; // Compressor spike
	DigitalInput *pressureSwitch; // Pressure switch to control compressor
	int ArmEncoderValue; // Current arm encoder position
	int BlockEncoderValue;
	int ExtendEncoderValue; // Current arm extension position
	DigitalInput *ClawSwitch; // Limit switch in claw motor
	DigitalInput *AutonomousSwitch; // Autonomous mode switch
	AnalogChannel *ultraDistance; // Ultrasonic distance sensor for autonomous

public:
	SimpleTracker(void)
	{
		myRobot = new MecanumDrive(LFMOTORPORT, LRMOTORPORT, RFMOTORPORT, RRMOTORPORT); // Instantiate drive system
		rightStick = new Joystick(RIGHTJOYSTICKPORT); // Create the joysticks
		leftStick = new Joystick(LEFTJOYSTICKPORT);
		clawStick = new Joystick(CLAWJOYSTICKPORT);
		LightSensor1 = new DigitalInput(LIGHTSENSORPORT1); // Create the light sensors
		LightSensor2 = new DigitalInput(LIGHTSENSORPORT2);
		LightSensor3 = new DigitalInput(LIGHTSENSORPORT3);
		LightSensor4 = new DigitalInput(DIGITALSIDECARTWO, LIGHTSENSORPORT4);
		ArmEncoder = new Encoder(ARMENCCHANNELA, ARMENCCHANNELB); // Create encoder to gauge arm position
		BlockEncoder = new Encoder(DIGITALSIDECARTWO, SWERVEBLOCKENCODER1, DIGITALSIDECARTWO, SWERVEBLOCKENCODER2);
		ArmRetractLimit = new DigitalInput(RETRACTLIMITSWITCHPORT); // Create limit switch for arm retraction
		ArmExtendLimit = new DigitalInput(EXTENDLIMITSWITCHPORT); //Create a limit switch for arm extention
		ArmPivotLimitUp = new DigitalInput(PIVOTLIMITSWITCHUPPORT); // Create limit switch for arm pivoting (full up)
		ArmPivotLimitDown = new DigitalInput(PIVOTLIMITSWITCHDOWNPORT); // Create limit switch for arm pivoting (full down)
		MinibotLimit = new DigitalInput(DIGITALSIDECARTWO, MINIBOTLIMITSWITCHPORT); // Create limit switch for minibot deployment
		ArmLightSensor = new DigitalInput(DIGITALSIDECARTWO, ARMLIGHTSENSOR); // Creates a lightsensor fo the arm "encoder"
		ArmPivot = new Victor(PIVOTMOTORPORT); // This class needs access to the motor as well
		ArmExtend = new Victor(EXTENDMOTORPORT); // This class needs access to the extending motor as well
		topclawmotor = new Victor(CLAWTOPMOTORPORT); //Create a pointer to the top claw motor
		bottomclawmotor = new Victor(CLAWBOTTOMMOTORPORT); // Create a pointer to the bottom claw motor
		minibotDeploy = new Solenoid(MINIBOTDEPLOYPORT);
		minibotDeploy2 = new Solenoid(MINIBOTDEPLOYPORT2);
		minibotRetract = new Solenoid(MINIBOTRETRACTPORT);
		engageBrake = new Solenoid(BRAKEPORT);
		//autoSwitch = new AnalogChannel(AUTOSWITCHPORT);
		autonomousSetting = new DigitalInput(DIGITALSIDECARTWO, AUTOSWITCHPORT);
		compressor = new Relay(DIGITALSIDECARTWO, COMPRESSORSPIKEPORT);
		pressureSwitch = new DigitalInput(DIGITALSIDECARTWO, COMPRESSORSWITCHPORT);
		ClawSwitch = new DigitalInput(DIGITALSIDECARTWO, CLAWSWITCHPORT); // Create a limit switch to detect tube presence in claw
		ultraDistance = new AnalogChannel(ULTRAPORT);
		AutonomousSwitch = new DigitalInput(DIGITALSIDECARTWO, AUTOSWITCHPORT);
		
		
		//theCompressor = new Compressor(DIGITALSIDECARTWO, COMPRESSORSWITCHPORT, DIGITALSIDECARTWO, COMPRESSORSPIKEPORT);
		
		// Start recording pulses
		ArmEncoder->Start();
		ArmEncoder->Reset();
		
		BlockEncoder->Start();
		BlockEncoder->Reset();

		ExtendEncoderValue = 0; // Initialize extension encoder to start position
		// Enable the compressor
		//theCompressor->Start();
		//Image* cameraImage = frcCreateImage(IMAQ_IMAGE_HSL);
	} // End of SimpleTracker constructor

	void OperatorControl(void)
	{
		int minibotDeployed = 0;
		int ArmTarget = -1; // Target pivot position for arm
		int ExtendTarget = -1; // Target extension position for arm
		int ArmPickup = 0; // Pickup enabled?
		int ArmDeploy = 0; // Auto-deploy enabled?
		int prevlight = 0, curlight = 0; // Previous and current values of arm extension light sensor
		float prevarmext = 0; // Store whether the arm was moving forward or backward before it stopped

		// Reset encoder on arm pivot
		//ArmEncoder->Reset();
		//ArmTarget = -1;
		
		BlockEncoder->Start();
		BlockEncoder->Reset();
		
		// While in operator control  
		while (IsOperatorControl())
		{
			// Reset the "watchdog" timer to show that the processor hasn't frozen
			GetWatchdog().Feed();
			
			float voltage = ultraDistance->GetVoltage();
			CLEARMESSAGE;
			DISPLAYMESSAGE(5, (int)(voltage * 1000.0f));
			
			BlockEncoderValue = BlockEncoder->Get();
			//DISPLAYMESSAGE(1, BlockEncoderValue);
			//DISPLAYMESSAGE(2, ArmEncoderValue);
			
			//DISPLAYMESSAGE(1, LightSensor1->Get());
			//DISPLAYMESSAGE(2, LightSensor2->Get());
			//DISPLAYMESSAGE(3, LightSensor3->Get());
			//DISPLAYMESSAGE(4, LightSensor4->Get());
			//DISPLAYMESSAGE(6, ExtendEncoderValue);
					
			// If we are not at the desired pressure, turn on the compressor
			if(!pressureSwitch->Get()) compressor->Set(Relay::kForward);
			else compressor->Set(Relay::kOff); 
			
			// Get the current encoder position
			ArmEncoderValue = ArmEncoder->Get();
			
			curlight = ArmLightSensor->Get();
			if(ArmExtend->Get() != 0) prevarmext = ArmExtend->Get();
			
			if(curlight != prevlight)
			{
				if(prevarmext < 0) ExtendEncoderValue--;
				else if(prevarmext && ArmExtendLimit->Get()) ExtendEncoderValue++;
			}
			
			prevlight = curlight;
			
			if(!ArmRetractLimit->Get()) ExtendEncoderValue = 0;
			
			//DISPLAYMESSAGE(2, ExtendEncoderValue);
			
			// Enter auto-pickup upon button command and set arm to pivot to home position
			if(PICKUPBUTTON) 
			{
				ArmPickup = 1;
				ExtendTarget = -1;
				ArmTarget = ARM_HOME;
			}
			
			if(!ClawSwitch->Get() && ArmPickup) ArmPickup = 2;
			
			//CLEARMESSAGE;
			//DISPLAYMESSAGE(1, ArmEncoderValue);
			//DISPLAYMESSAGE(2, pressureSwitch->Get());
			
			// Send joystick data to drive system as well as limit switch pointers
			// Depending on which drive system we're using (according to the DRIVESYSTEM definition),
			// call the appropriate function
			myRobot->DriveSwerve(leftStick, rightStick, clawStick,
					ArmPivotLimitDown, ArmPivotLimitUp, ArmRetractLimit,
					ArmExtendLimit, ArmEncoder, ArmTarget, ArmEncoderValue, 0, ArmPickup, ArmDeploy, ExtendTarget);

			// If the deploy button is pressed, close all retract solenoids and
			// open the extend solenoids
			if(DEPLOYMINIBOT) 
			{
				minibotDeploy->Set(1); 
				//minibotDeploy2->Set(1);
				//minibotRetract->Set(0);
				minibotDeployed = 1;
			}
				
			// If the retract button is pressed, close all deploy solenoids and
			// open the retract solenoids
			else if(RETRACTMINIBOT && minibotDeployed)
			{
				//minibotDeploy->Set(0);
				//minibotDeploy2->Set(0);
				minibotRetract->Set(1);
				//minibotDeployed = 0;
			}
					
			// If any arm button is pressed, exit auto-pickup
			if(EXTENDBUTTON || RETRACTBUTTON || PIVOTPOSITIONHOME || PIVOTPOSITIONLOW || PIVOTPOSITIONMIDDLE || PIVOTPOSITIONHIGH || PIVOTPOSITIONFULL || PIVOTDOWNBUTTON || PIVOTUPBUTTON || PREPICKUPBUTTON) ArmPickup = 0, ArmDeploy = 0;
			if(ArmDeploy == 1 && ArmPickup != 0) ArmPickup = 0;
			if(!ArmRetractLimit->Get() && ArmPickup != 2)  ArmPickup = 0;
			
			// Read the joystick buttons and set the arm-pivot
			// target position to the corresponding encoder value
			if (PIVOTPOSITIONHOME)
			{
				// If the trigger is pressed, enable auto-deploy
				if(!CLAWREVERSEBUTTON)
				{
					ArmTarget = -1;
					ExtendTarget = -1;
					ArmDeploy = 1;
					ArmPickup = 0;
				}
				
				// Otherwise, go to home position
				else ArmTarget = ARM_HOME, ExtendTarget = 0;
			}
			
			else if (PIVOTPOSITIONLOW)
				ArmTarget = ARM_RACKBOTTOM, ExtendTarget = 0;
			else if (PIVOTPOSITIONMIDDLE)
				ArmTarget = ARM_RACKMIDDLE, ExtendTarget = 0;
			else if (PIVOTPOSITIONHIGH)
				ArmTarget = ARM_RACKTOP, ExtendTarget = 0;
			else if (PIVOTPOSITIONFULL)
				ArmTarget = ARM_FULLUP, ExtendTarget = ARMEXTENDRACK;
			else if(PREPICKUPBUTTON)
				ArmTarget = ARM_HOME, ExtendTarget = PREPICKUPEXTEND;

			// If the manual arm buttons are pressed, disable auto-movement
			if (PIVOTDOWNBUTTON || PIVOTUPBUTTON)
				ArmTarget = -1;
			
			if (EXTENDBUTTON || RETRACTBUTTON)
				ExtendTarget = -1;

			// Negative one indicates that the arm position is being set
			// manually. Otherwise, it is being moved to a preset position
			// and we need to use a PID loop to approach the correct position
			if (ArmTarget >= 0)
			{	
				// If the arm pivot has hit the home position limit switch, reset the encoder and stop
				if (!ArmPivotLimitDown->Get() && ArmTarget < ArmEncoderValue)
				{
					ArmEncoder->Reset();
					ArmPivot->Set(0);
					ArmTarget = -1;
				}

				// If the arm pivot has hit the full up limit switch, stop
				if (!ArmPivotLimitUp->Get() && ArmTarget > ArmEncoderValue)
				{
					ArmPivot->Set(0);
					ArmTarget = -1;
				}

				// If the arm needs to move down
				if (ArmEncoderValue > ArmTarget + ARMDEADBAND)
				{
					if (ArmEncoderValue - ArmTarget < ARMAPPROACHDOWN)
						ArmPivot->Set(ARMDOWNSPEED + Abs(((ARMAPPROACHDOWN - (ArmEncoderValue - ArmTarget)) / ARMAPPROACHDOWN)));
					else
						ArmPivot->Set(ARMDOWNSPEED);
				}

				// If the arm needs to move up
				else if (ArmEncoderValue < ArmTarget - ARMDEADBAND)
				{
					if (ArmTarget - ArmEncoderValue < ARMAPPROACHUP)
						ArmPivot->Set(ARMUPSPEED * 0.4 - Abs(((ARMAPPROACHUP - (ArmTarget - ArmEncoderValue)) / ARMAPPROACHUP)));
					else
						ArmPivot->Set(ARMUPSPEED);
				}
				
				else ArmPivot->Set(0);
			}
			
			DISPLAYMESSAGE(1, ArmEncoderValue);
			
			if(ExtendTarget >= 0)
			{
				if(ExtendEncoderValue > ExtendTarget + 1) 
				{
					if(ArmRetractLimit->Get()) ArmExtend->Set(-1);
					else ArmExtend->Set(0);
				}
				
				else if(ExtendEncoderValue < ExtendTarget - 1)
				{
					if((ArmExtendLimit->Get() && ArmEncoderValue > 30) || ExtendTarget == PREPICKUPEXTEND) ArmExtend->Set(1);
					else ArmExtend->Set(0);
				}
				
				else ArmExtend->Set(0);
			}
			
			// If auto-deploy is enabled
			if(ArmDeploy)
			{
				// If the arm has not finished retracting
				if(ArmRetractLimit->Get())
				{
					// Eject the tube and retract the arm
					myRobot->topclawmotor->Set(0.5); //@
					myRobot->bottomclawmotor->Set(-0.5); //@
					ArmExtend->Set(-1);
				}
				
				// Otherwise
				else
				{
					// Stop the motors and exit auto-deploy
					myRobot->topclawmotor->Set(0);
					myRobot->bottomclawmotor->Set(0);
					ArmExtend->Set(0);
					//ArmDeploy = 0;
					ArmTarget = ARM_HOME;
				}
			}
			
			if(!ArmPivotLimitDown->Get()) ArmDeploy = 0;
			
			// The arm is moving, disengage the brake. Otherwise, re-engage the brake
			if (ArmPivot->Get())
				engageBrake->Set(0);
			else
				engageBrake->Set(1);
		} // End of loop while in operator control
	} // End of OperatorControl function

	void Autonomous(void)
	{
		// Has the robot reached the end of the line ("T")?
		int stop = 0; // Has the robot reached the end of the line?
		//int strafe = 0; // Is the robot strafing along the branch?
		//int ArmTarget = -1; // Target pivot position for arm
		int ExtendTarget = ARMEXTENDRACK; // Target extension position for arm
		unsigned int timesetting = 0; // Used to track timing of various scripted events
		int prevlight, curlight; // Previous and current states of light sensor on arm extension
		float prevarmext = 0; // Previous setting of arm extension motor
		int trip = 0;
		
		BlockEncoder->Start();
		BlockEncoder->Reset();
		myRobot->trip = 0;
		
		// While in autonomous mode
		while (IsAutonomous())
		{
			// Feed "watchdog" to prevent an assumed lockup 
			GetWatchdog().Feed();
			CLEARMESSAGE;
			DISPLAYMESSAGE(6, (int)(ultraDistance->GetVoltage() * 1000.0f));
			
			
			// Arm encoder overhead
				// If the light sensor detects a different colour than last time, it has moved
				curlight = ArmLightSensor->Get();
				if(ArmExtend->Get() != 0) prevarmext = ArmExtend->Get(); // Only change the stored motor direction if the motor is running
				if(curlight != prevlight)
				{
					// Increment or decrement encoder value based on direction of extension motor
					if(prevarmext < 0) ExtendEncoderValue--;
					// Stop adding if the exension limit has been hit
					else if(prevarmext > 0 && ArmExtendLimit->Get()) ExtendEncoderValue++;
				}
				
				// Set previous state for next time
				prevlight = curlight;
			
				// If the arm retracts fully, reset value to zero, just in case
				if(!ArmRetractLimit->Get()) ExtendEncoderValue = 0;
				ArmEncoderValue = ArmEncoder->Get();
				//DISPLAYMESSAGE(2, ExtendEncoderValue);
			// End of arm encoder overhead
						
			// If the robot has not reached the end, track the line
			// When the return value becomes one, the robot should begin executing
			// The scripted events below. 0 == first stage of autonomous (line tracking)
			if (stop == 0) stop = myRobot->MoveSwerve(LightSensor1->Get(), LightSensor3->Get(), LightSensor2->Get(), ArmPivotLimitUp->Get(), ultraDistance, AutonomousSwitch->Get());
			
			// Extend the arm to the desired position, if one exists (-1 means neutral)
			if(ExtendTarget >= 0)
			{
				if(ExtendEncoderValue > ExtendTarget + 1) 
				{
					if(ArmRetractLimit->Get()) ArmExtend->Set(-1);
					else ArmExtend->Set(0);
				}
							
				else if(ExtendEncoderValue < ExtendTarget - 1)
				{
					if(ArmExtendLimit->Get() && ArmEncoderValue > 80) ArmExtend->Set(1);
					else ArmExtend->Set(0);
				}
							
				else ArmExtend->Set(0);
			}
			
			// Once we've reached the end of the first line, stop and wait for the arm to finish
			// pivoting and extending
			if(stop == 1)
			{
				// Make sure to stop
				myRobot->lfDrive->Set(0);
				myRobot->lrDrive->Set(0);
				myRobot->rfDrive->Set(0);
				myRobot->rrDrive->Set(0);
				
				if(Abs(ArmPivot->Get()) < 0.05 && Abs(ArmExtend->Get()) < 0.05) stop = 2;
			}
			
			// Once we're prepared to deploy the first tube, do so
			else if(stop == 2 && AutonomousSwitch->Get())
			{
				// Neutralize the arm extension for later
				ExtendTarget = -1;
				
				// Allow the arm to retract some before deploying the tube, while also moving back
				// slightly from the rack
				if(timesetting < RETRACTTIME) 
				{
					timesetting++;
					myRobot->lfDrive->Set(-0.2);
					myRobot->lrDrive->Set(-0.2);
					myRobot->rfDrive->Set(-0.2);
					myRobot->rrDrive->Set(-0.2);
				}
				
				// Then, rotate the tube briefly
				else if(timesetting < MANIPULATIONTIME)
				{
					myRobot->lfDrive->Set(0);
					myRobot->lrDrive->Set(0);
					myRobot->rfDrive->Set(0);
					myRobot->rrDrive->Set(0);
					myRobot->bottomclawmotor->Set(-CLAWROLLERSPEED);
					myRobot->topclawmotor->Set(-CLAWROLLERSPEED);
					timesetting++;
				}

				// Finally, deploy the tube
				else
				{

					// If the arm has not finished retracting, continue to deploy tube with the rollers
					if (ArmRetractLimit->Get())
					{
						myRobot->bottomclawmotor->Set(-CLAWROLLERSPEED);
						myRobot->topclawmotor->Set(CLAWROLLERSPEED);
					}

					// Once the arm has finished retracting, stop the rollers
					else
					{
						myRobot->bottomclawmotor->Set(0);
						myRobot->topclawmotor->Set(0);
					}
				}

				// Retract the arm to the home limit switch
				if (ArmRetractLimit->Get()) ArmExtend->Set(-1);
				else 
				{
					ArmExtend->Set(0);
					BlockEncoder->Start();
					BlockEncoder->Reset();
					stop = 3;
				}
			}
			
			else if(stop == 3 && AutonomousSwitch->Get())
			{
				if(ArmPivotLimitDown->Get()) ArmPivot->Set(-1);
				else ArmPivot->Set(0);
				myRobot->lfDrive->Set(-0.2);
				myRobot->lrDrive->Set(-0.2);
				myRobot->rfDrive->Set(-0.2);
				myRobot->rrDrive->Set(-0.2);
				myRobot->bottomclawmotor->Set(0);
				myRobot->topclawmotor->Set(0);
			}
			
			else if(stop == 2 && !AutonomousSwitch->Get() && TrackSteering((int)(BRANCHANGLE * PULSES_PER_STEER), 2))
			{
				if(ultraDistance->GetVoltage() * 1000 < BRANCHSTOPDISTANCE) trip++;
				if(trip > 320) stop = 3;
				myRobot->lfDrive->Set(0.4);
				myRobot->lrDrive->Set(0.4);
				myRobot->rfDrive->Set(0.4);
				myRobot->rrDrive->Set(0.4);
			}
			
			else if(stop == 3 && !AutonomousSwitch->Get())
			{
				ExtendTarget = -1;
				
				myRobot->lfDrive->Set(0);
				myRobot->lrDrive->Set(0);
				myRobot->rfDrive->Set(0);
				myRobot->rrDrive->Set(0);
				
				// If the arm has not finished retracting, continue to deploy tube with the rollers
				if (ArmRetractLimit->Get())
				{
					myRobot->bottomclawmotor->Set(-CLAWROLLERSPEED);
					myRobot->topclawmotor->Set(CLAWROLLERSPEED);
				}

				// Once the arm has finished retracting, stop the rollers
				else
				{
					myRobot->bottomclawmotor->Set(0);
					myRobot->topclawmotor->Set(0);
				}

				// Retract the arm to the home limit switch
				if (ArmRetractLimit->Get()) ArmExtend->Set(-1);
				else 
				{
					ArmExtend->Set(0);
					BlockEncoder->Start();
					BlockEncoder->Reset();
					stop = 4;
				}
			}
			
			if(stop == 4 && !AutonomousSwitch->Get())
			{
				if(ArmPivotLimitDown->Get()) ArmPivot->Set(-1);
				else ArmPivot->Set(0);
				if(TrackSteering(0, 0))
				{
					myRobot->lfDrive->Set(-0.3);
					myRobot->lrDrive->Set(-0.3);
					//myRobot->rfDrive->Set(0.2);
					//myRobot->rrDrive->Set(0.2);
				}
				myRobot->bottomclawmotor->Set(0);
				myRobot->topclawmotor->Set(0);				
			}
			
			// Until the arm has been fully pivoted up, and we haven't finished line-tracking,
			// move it up
			if (stop == 0)
			{
				if (ArmPivotLimitUp->Get()) ArmPivot->Set(1);
				else ArmPivot->Set(0);
			}
		} // End of loop while in autonomous mode
	} // End of Autonomous
	
	// Use the encoders, with regard to the limit switches, turn the steering to the 
	// goal value passed (used by autonomous function)
	int TrackSteering(int Target, int side)
	{
		int leftSteeringEncoderValue;
		int rightSteeringEncoderValue;
		float speedincrement = 1/STEERING_APPROACH_DISTANCE;
		
		// Get the current steering positions
		leftSteeringEncoderValue = myRobot->leftSteeringEncoder->Get();
		rightSteeringEncoderValue = myRobot->rightSteeringEncoder->Get();
		
		if(side != 1)
		{
			// Turn the steering
			if(Abs(leftSteeringEncoderValue - Target) < (STEERING_DEADBAND * PULSES_PER_STEER)) myRobot->leftSteer->Set(0);
			else if(leftSteeringEncoderValue < Target)
			{
				if(!myRobot->backleftSteerLimit->Get()) myRobot->leftSteer->Set(0);
				else if(leftSteeringEncoderValue + STEERING_APPROACH_DISTANCE >= Target) myRobot->leftSteer->Set(speedincrement * Abs(Target - leftSteeringEncoderValue));
				else myRobot->leftSteer->Set(0.9);
			}
		
			else
			{
				if(!myRobot->backrightSteerLimit->Get()) myRobot->leftSteer->Set(0);
				else if(leftSteeringEncoderValue - STEERING_APPROACH_DISTANCE <= Target) myRobot->leftSteer->Set(-1 * speedincrement * Abs(Target - leftSteeringEncoderValue));
				else myRobot->leftSteer->Set(-0.9);
			}
		}
		
		// Right steering uses inverse angle
		Target *= -1;
		
		if(side != 2)
		{
			// Turn the steering
			if(Abs(rightSteeringEncoderValue - Target) < (0.05f * PULSES_PER_STEER)) myRobot->rightSteer->Set(0);
			else if(rightSteeringEncoderValue < Target)
			{
				if(!myRobot->frontleftSteerLimit->Get()) myRobot->rightSteer->Set(0);
				else if(rightSteeringEncoderValue + STEERING_APPROACH_DISTANCE >= Target) myRobot->rightSteer->Set(speedincrement * Abs(Target - leftSteeringEncoderValue));
				else myRobot->rightSteer->Set(0.9);
			}
				
			else
			{
				if(!myRobot->backleftSteerLimit->Get()) myRobot->rightSteer->Set(0);
				else if(rightSteeringEncoderValue - STEERING_APPROACH_DISTANCE <= Target) myRobot->rightSteer->Set(-1 * speedincrement * Abs(Target - leftSteeringEncoderValue));
				else myRobot->rightSteer->Set(-0.9);
			}
		}
		
		if(Abs(myRobot->rightSteer->Get()) < 0.05f && Abs(myRobot->leftSteer->Get()) < 0.05f) return 1;
		else return 0;
	}
}; // End of SimpleTracker class

// Return absolute value of argument (distance from zero)
float Abs(float x)
{
	if (x < 0)
		x *= -1;
	return x;
}

// Entry point is FRC_UserProgram_StartupLibraryInit
START_ROBOT_CLASS(SimpleTracker)
;

/* To send text to driver station:
 * // Send formatted text to specified line and column
 DriverStationLCD::GetInstance()->Printf(DriverStationLCD::kUser_Line1, 1, "This is a test");
 // Update to make text appear
 DriverStationLCD::GetInstance()->UpdateLCD();
 */
