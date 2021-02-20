package frc.Autonomous;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

import frc.Mechanisms.*;
import frc.robot.*;

public class CatzAutonomous
{
    	/***************************************************************************
	 * PID Turn Constants
	 ***************************************************************************/
    static public double PID_TURN_THRESHOLD   = 0.5;

	/***************************************************************************
	 * PID_TURN_DELTAERROR_THRESHOLD_HI - Delta Error Values larger than this are
	 * considered invalid and will be ignored PID_TURN_DELTAERROR_THRESHOLD_LO -
	 * When drivetrain power drops below the PID_TURN_MIN_xxx_POWER, we will check
	 * to see if deltaError is below this threshold before setting power at
	 * PID_TURN_MIN_xxx_POWER.
	 ***************************************************************************/
	final static public double PID_TURN_DELTAERROR_THRESHOLD_HI = 4.0;
	final public static double PID_TURN_DELTAERROR_THRESHOLD_LO = 0.11;

	final static public double PID_TURN_FILTER_CONSTANT    = 0.7;
	      static public double PID_TURN_POWER_SCALE_FACTOR = 1.0;

	      static public double PID_TURN_KP = 0.08;
	      static public double PID_TURN_KI = 0.0;
	      static public double PID_TURN_KD = 0.012; // 0.0744

	final static public double PID_TURN_INTEGRAL_MAX =  1.0;
	final static public double PID_TURN_INTEGRAL_MIN = -1.0;

	final public static double PID_TURN_MIN_POS_POWER = 0.6; // 0.4 is min power to move robot when it is stopped
	final public static double PID_TURN_MIN_NEG_POWER = -PID_TURN_MIN_POS_POWER;

	
	/***************************************************************************
	 * PID Turn Variables
	 ***************************************************************************/
    public static CatzPID turnPid;

	static Timer functionTimer;
    static Timer pdTimer;
    
    final public static double DRIVE_MAX_POS_POWER =  1.0;
	final public static double DRIVE_MAX_NEG_POWER = -1.0;

    final static public double NAVX_RESET_WAIT_TIME = 0.2;

    static double pidTurnkP = PID_TURN_KP;
    static double pidTurnkI = PID_TURN_KI;
    static double pidTurnkD = PID_TURN_KD;

	static double currentError; 
	static double deltaError;
	static double derivative;
	static double deltaT;

	static double power;

	static double previousError;
	static double totalError;

	static double currentAngle;
	static double currentAngleAbs;
	static double targetAngle;
	static double targetAngleAbs;

	static double timeout;
	static double loopDelay = 0.015;

	static double previousDerivative = 0;

	static boolean tuningMode = false;
	static boolean debugMode = false;

    public static boolean checkBoxL;
    public static boolean checkBoxM;
    public static boolean checkBoxR;

    public boolean prev_boxL = false;
	public boolean prev_boxM = false;
	public boolean prev_boxR = false;

    public final double STOP_THRESHOLD_ANGLE = 2;
    public final double SLOW_THRESHOLD_ANGLE = 20;

    public boolean runningRadialTurn = false;
    public double angleGoal;
    public double angleToTurn;

    public double currentEncPosition;
    public double turnRateRadians;

    public double r1;
    public double r2;
    public double s1Dot;
    public double s2Dot;
    public double s1Conv;
    public double s2Conv;

    public Timer turnT;
    public Timer driveT1;
    public Timer driveTWait;
    public Timer shooterTimer;

    public final double WAIT_TIME_THRESHOLD = 5.0;
    public double totalTime = 0;

    public double lVelocity;
    public double rVelocity;

    public double leftInitialEncoderPos;
    public double rightInitialEncoderPos;

    public boolean runningDistanceDrive = false;
    public boolean driveBackwards = false;
    public boolean monitoringTime = false;

    public double distanceGoal;
    public double distanceMoved;
    public final double STOP_THRESHOLD_DIST = 1;
    public final double SLOW_THRESHOLD_DIST = 30;

    public final double ENCODER_COUNTS_PER_INCH_LT = 1014.5; //without weight: 1046.6
    public final double ENCODER_COUNTS_PER_INCH_RT = 964;    //without weight: 1025.7

    public final double TO_RADIANS = Math.PI/180;
    public final double TO_DEGREES = 180/Math.PI;

    public CatzAutonomous()
    {
        turnT        = new Timer();
        driveT1      = new Timer();
        driveTWait   = new Timer();
        shooterTimer = new Timer();
    }

    public void resetTotalTime()
    {
        totalTime = 0;
    }

    public boolean simulateShoot()
    {
        boolean status = false;

        if(shooterTimer.get() > 3)
        {
            status = true;
        }
        return status;
    }

    public void monitorTimer()
    {
        if(monitoringTime)
        {    
            if(driveTWait.get() > WAIT_TIME_THRESHOLD)
            {
                setDistanceGoal(103.5, 16000);
            }
        }
    }

    public boolean monitorEncoderPosition()
    {
        boolean status = false;

        if (runningDistanceDrive == true)
        {
            currentEncPosition = Robot.driveTrain.getIntegratedEncPosition("LT");
            distanceMoved = Math.abs(leftEncoderDistanceMoved(currentEncPosition));
           
            SmartDashboard.putNumber("Encoder Position", currentEncPosition);
            SmartDashboard.putNumber("Initial Encoder Distance", leftInitialEncoderPos);
            SmartDashboard.putNumber("Distance Moved", distanceMoved);
            SmartDashboard.putNumber("Delta Encoder", (currentEncPosition - leftInitialEncoderPos));
            SmartDashboard.putNumber("drive timer", driveT1.get());

            //System.out.println((currentEncPosition - leftInitialEncoderPos) + " = " + currentEncPosition + " - " + leftInitialEncoderPos);
            double distanceToGoal;
            if(distanceGoal > 0)
            {
                distanceToGoal = distanceGoal - distanceMoved;
            }
            else
            {
                distanceToGoal = distanceGoal + distanceMoved;
            }
                

            SmartDashboard.putNumber("Distance To Goal", distanceToGoal);

            SmartDashboard.putNumber("total time", totalTime);
            if (distanceToGoal < STOP_THRESHOLD_DIST)
            {
                Robot.driveTrain.setTargetVelocity(0);
                runningDistanceDrive = false;
                status = true;
            }
            else if (distanceToGoal < SLOW_THRESHOLD_DIST && distanceToGoal < distanceGoal*0.5)
            {
                if(Robot.driveTrain.getIntegratedEncVelocity("LT") > 3000)
                    Robot.driveTrain.setTargetVelocity(Robot.driveTrain.getIntegratedEncVelocity("LT")*0.98);
                
            }

        }

        return status;
    }

    public void setDistanceGoal(double inches, int speed)
    {
        if(!runningDistanceDrive)
        {   
            driveT1.reset();
            monitoringTime = false;
            runningDistanceDrive = true;
            leftInitialEncoderPos = Robot.driveTrain.drvTrainMtrCtrlLTFrnt.getSelectedSensorPosition(0);
            if (inches < 0)
            {
                driveT1.start();
                distanceGoal = -inches;
                Robot.driveTrain.setTargetVelocity(-speed);
            }
            else    
            {
                driveT1.start();
                distanceGoal = inches;
                Robot.driveTrain.setTargetVelocity(speed);
            }
        }
    }

    /***************************************************************************
	 * PIDturn()
	 * 
	 * timeoutSeconds: -1 : PIDturn() will calculate timeout based on degreesToTurn
	 * 0 : Max Timeout >0 : # of seconds before aborting
	 ***************************************************************************/
	public static void PIDturn(double degreesToTurn, double timeoutSeconds, double maxPower) {
		Robot.navx.reset();
		Timer.delay(NAVX_RESET_WAIT_TIME);

        turnPid = new CatzPID();
		pdTimer = new Timer();
		functionTimer = new Timer();
		functionTimer.reset();
        functionTimer.start();

		boolean done = false;

		setPidValues(degreesToTurn, turnPid);

		currentAngle  = Robot.navx.getAngle();
		targetAngle   = degreesToTurn + currentAngle;

		printDebugInit();
		printDebugHeader();

		pdTimer.reset();
		pdTimer.start();
		while (done == false) {
			currentAngle = Robot.navx.getAngle();

			deltaT = pdTimer.get();
			pdTimer.stop();
			pdTimer.reset();
			pdTimer.start();
            
            turnPid.setCommand((float)targetAngle);
            turnPid.setFeedback((float)currentAngle);

            if (functionTimer.get() > timeoutSeconds) {
                done = true;
            } else {
                turnPid.calc_pid((float)deltaT);
                /*******************************************************************
                 * Cmd robot to turn at new power level Note: Power will be positive if turning
                 * right and negative if turning left
                 *******************************************************************/
                //documentation states that the output value is the derivative of the input values 
                //so I just assumed I could multiply the delta time to integrate it.
                double outputPower = turnPid.getOutput() * deltaT; 
                Robot.driveTrain.setTargetPower(outputPower);
                printDebugData();
                Timer.delay(loopDelay);
            }
		}

		/**********************************************************************
		 * We're at targetAngle or timed out. Stop the robot and do final cleanup. -
		 * Print out last set of debug data (note that this may not be a complete set of
		 * data) - Stop timers
		 **********************************************************************/
	    currentAngle = Robot.navx.getAngle();
	    printDebugData();
		Robot.driveTrain.setTargetPower(0.0); // makes robot stop
	    currentAngle = Robot.navx.getAngle();
	    printDebugData();

		functionTimer.stop();
		pdTimer.stop();
	}



    /***************************************************************************
    *
    * setPidValues()
    * 
    ***************************************************************************/
    public static void setPidValues(double degreesToTurn) {
        double degreesToTurnAbs;

        degreesToTurnAbs = Math.abs(degreesToTurn);

        if (degreesToTurnAbs <= 25.0) {
            pidTurnkP = 0.090;
            pidTurnkD = 0.024;
        }
        if (degreesToTurnAbs <= 30.0) {
            pidTurnkP = 0.110;   //0.126 at 12.0 V;
            pidTurnkD = 0.026;
            PID_TURN_THRESHOLD = 0.75;
            loopDelay = 0.007;
        }
        else if (degreesToTurnAbs <= 35.0) {
            pidTurnkP = 0.090;
            pidTurnkD = 0.020;
        }
        else if (degreesToTurnAbs <= 40.0) {
            pidTurnkP = 0.086;
            pidTurnkD = 0.024;
        }
        else if (degreesToTurnAbs <= 45.0) {
            pidTurnkP = 0.090;
            pidTurnkD = 0.030;
            PID_TURN_THRESHOLD = 0.75;
            loopDelay = 0.007;
        }
        else if (degreesToTurnAbs <= 50.0) {
            pidTurnkP = 0.100;
            pidTurnkD = 0.028;
            PID_TURN_THRESHOLD = 0.75;
            loopDelay = 0.007;
        }
        else { //if degreesToTurnAbs > 50.0
            pidTurnkP = 0.080;  //PID_TURN_KP;
            pidTurnkD = 0.030;  //PID_TURN_KD;
            PID_TURN_THRESHOLD = 0.5;
            loopDelay = 0.010;
        }

    }   //End of setPidValues()

    public static void setPidValues(double degreesToTurn, CatzPID pid) {
        double degreesToTurnAbs;

        degreesToTurnAbs = Math.abs(degreesToTurn);

        if (degreesToTurnAbs <= 25.0) {
            //pidTurnkP = 0.090;
            pid.setPgain(0.090f);
            //pidTurnkD = 0.024;
            pid.setDgain(0.024f);
        }
        if (degreesToTurnAbs <= 30.0) {
            //pidTurnkP = 0.110;   //0.126 at 12.0 V;
            pid.setPgain(0.110f);
            //pidTurnkD = 0.026;
            pid.setDgain(0.026f);
            //PID_TURN_THRESHOLD = 0.75;
            pid.setDeadband(0.75f);
            loopDelay = 0.007;
        }
        else if (degreesToTurnAbs <= 35.0) {
            //pidTurnkP = 0.090;
            pid.setPgain(0.090f);
            //pidTurnkD = 0.020;
            pid.setDgain(0.020f);
        }
        else if (degreesToTurnAbs <= 40.0) {
            //pidTurnkP = 0.086;
            pid.setPgain(0.086f);
            //pidTurnkD = 0.024;
            pid.setDgain(0.024f);
        }
        else if (degreesToTurnAbs <= 45.0) {
            //pidTurnkP = 0.090;
            pid.setPgain(0.090f);
            //pidTurnkD = 0.030;
            pid.setDgain(0.030f);
            //PID_TURN_THRESHOLD = 0.75;
            pid.setDeadband(0.75f);
            loopDelay = 0.007;
        }
        else if (degreesToTurnAbs <= 50.0) {
            //pidTurnkP = 0.100;
            pid.setPgain(0.100f);
            //pidTurnkD = 0.028;
            pid.setDgain(0.028f);
            //PID_TURN_THRESHOLD = 0.75;
            pid.setDeadband(0.75f);
            loopDelay = 0.007;
        }
        else { //if degreesToTurnAbs > 50.0
            //pidTurnkP = 0.080;  //PID_TURN_KP;
            pid.setPgain(0.080f);
            //pidTurnkD = 0.030;  //PID_TURN_KD;
            pid.setDgain(0.030f);
            //PID_TURN_THRESHOLD = 0.5;
            pid.setDeadband(0.5f);
            loopDelay = 0.010;
        }

    }   //End of setPidValues()



    /***************************************************************************
    *
    * setPIDTurnDebugModeEnabled()
    * 
    ***************************************************************************/
	public static void setPIDTurnDebugModeEnabled(boolean enabled) {
		debugMode = enabled;
	}

	public static boolean isTuningModeEnabled() {
		return tuningMode;
	}

    /***************************************************************************
    *
    * printDebugInit()
    * 
    ***************************************************************************/
	public static void printDebugInit() {
		if (debugMode == true) {
            System.out.println("navX: *** PID Turn **********************************************************");
            System.out.printf("navX: kP, %.3f, kI, %.3f, kD, %.3f\n", pidTurnkP, pidTurnkI, pidTurnkD);
            System.out.printf("navX: currentAngle,   %.3f, targetAngle, %.3f\n", currentAngle, targetAngle);
            System.out.printf("navX: ErrThreshold,   %.3f\n", PID_TURN_THRESHOLD);
            System.out.printf("navX: DerivFiltConst, %.3f\n", PID_TURN_FILTER_CONSTANT);
            System.out.printf("navX: MinPosPwr,      %.3f\n", PID_TURN_MIN_POS_POWER);
            System.out.printf("navX: MinNegPwr,      %.3f\n", PID_TURN_MIN_NEG_POWER);
            System.out.printf("navX: delErrThreshHi, %.3f\n", PID_TURN_DELTAERROR_THRESHOLD_HI);
            System.out.printf("navX: delErrThreshLo, %.3f\n", PID_TURN_DELTAERROR_THRESHOLD_LO);
		}
	}


	/***************************************************************************
    *
    * printDebugHeader()
    * 
    ***************************************************************************/
    public static void printDebugHeader() {
        if (debugMode == true) {
            System.out.print("navX: time,deltaT,currAngle,currError,deltaError,deriv,power\n");
        }
    }


    /***************************************************************************
    *
    * printDebugData()
    * 
    ***************************************************************************/
    public static void printDebugData() {
        if (debugMode == true) {

            System.out.printf("navX: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", functionTimer.get(), deltaT,
                    currentAngle, currentError, deltaError, derivative, power);
        }
    }
    
    /***************************************************************************
    *
    * Access Methods
    * 
    ***************************************************************************/
    public static double getCurrTime()
    {
        return functionTimer.get();
    }

    public static double getDeltaT()
    {
        return pdTimer.get();
    }

    public static double getCurrAngle()
    {
        return Robot.navx.getAngle();
    }

    public static double getCurrError()
    {
        double CurrError = targetAngle - currentAngle;
        return CurrError;
    }
 
    public static double getDeltaError()
    {
        double dError = currentError - previousError;
        return dError;
    }

    public static double getCurrPower()
    {
        return power;
    }

 /*   public boolean monitorAngle()
    {
        boolean status = false;
        if (runningRadialTurn == true)
        {
            double currentAngle = Math.abs(Robot.navx.getAngle());
            SmartDashboard.putNumber("Current Angle", currentAngle);
            SmartDashboard.putNumber("turn Time", turnT.get());

            angleToTurn = angleGoal - currentAngle;

            SmartDashboard.putNumber("Angle to Turn", angleToTurn);
            SmartDashboard.putNumber("Angle to Turn Plot", angleToTurn);
            SmartDashboard.putNumber("Angle Goal", angleGoal);

            SmartDashboard.putNumber("left enc velocity", Robot.driveTrain.getIntegratedEncVelocity("LT"));

            if(angleToTurn < STOP_THRESHOLD_ANGLE)
            {
                Robot.driveTrain.drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Velocity, 0);
                Robot.driveTrain.drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Velocity, 0);  
                turnT.stop();
                status = true;
                runningRadialTurn = false;
            }
           else if (angleToTurn < SLOW_THRESHOLD_ANGLE && angleToTurn < (angleGoal * 0.5))
            {

                lVelocity = Robot.driveTrain.getIntegratedEncVelocity("LT");
                rVelocity = Robot.driveTrain.getIntegratedEncVelocity("RT");

                if(lVelocity > 5000 && rVelocity > 5000)
                {
                    Robot.driveTrain.drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Velocity, Robot.driveTrain.getIntegratedEncVelocity("LT")*0.99);
                    Robot.driveTrain.drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Velocity, Robot.driveTrain.getIntegratedEncVelocity("RT")*0.99);
                }

            }

        }

        return status;

    }
    
    /**
     * @param angle - The target angle to turn; A negative angle results in a left turn
     * @param speed - The base speed to apply to the drive train; Measured in counts/100ms
     * @param ratio - The ratio between the two sides of the drive train; Setting 0.5 will result in one side moving at half the speed of the other
     */
 /*   
    public void setAngleGoal(double angle, int speed, double ratio)
    {
        if(!runningRadialTurn)
        {


            angleGoal = angle;
            runningRadialTurn = true;
            Robot.navx.reset();
            turnT.start();

            if(angle < 0)
            {

                Robot.driveTrain.drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Velocity, speed * ratio);
                Robot.driveTrain.drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Velocity, -speed);

            }
            else
            {

                Robot.driveTrain.drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Velocity, speed);
                Robot.driveTrain.drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Velocity, -speed * ratio);

            }


            
        }
    }

    public void radialTurn(double radiusOfCurvature, double turnRateDegrees, double targetAngleDegrees)
    {
        turnT.reset();

        r1 = radiusOfCurvature;
        r2 = radiusOfCurvature + (7.0/3.0);

        turnRateRadians = turnRateDegrees*TO_RADIANS;

        s1Dot = r1 * turnRateRadians;
        s2Dot = r2 * turnRateRadians;

        s1Conv = s1Dot * ENCODER_COUNTS_PER_INCH_RT * 12 *(1.0/10.0);
        s2Conv = s2Dot * ENCODER_COUNTS_PER_INCH_LT * 12 *(1.0/10.0);

        SmartDashboard.putNumber("s1", s1Dot);
        SmartDashboard.putNumber("s2", s2Dot);
        SmartDashboard.putNumber("s1Conv", s1Conv);
        SmartDashboard.putNumber("s2Conv", s2Conv);
        SmartDashboard.putNumber("Target Angle", targetAngleDegrees);
        SmartDashboard.putNumber("Turn Rate Degrees", turnRateDegrees);

        double targetAngleRadians = targetAngleDegrees * TO_RADIANS;
        double timeOut = (targetAngleRadians)/turnRateRadians;
       
        turnT.start();

        SmartDashboard.putNumber("Time Out", timeOut);

        double deltaTime;
        double timeStart = turnT.get();

        while ((turnT.get() - timeStart) < 2)
        {
            Robot.driveTrain.drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Velocity, s2Conv);
            Robot.driveTrain.drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Velocity, -s1Conv * DRIVE_STRAIGHT_PID_TUNING_CONSTANT);

            Robot.driveTrain.drvTrainMtrCtrlLTBack.follow(Robot.driveTrain.drvTrainMtrCtrlLTFrnt);
            Robot.driveTrain.drvTrainMtrCtrlRTBack.follow(Robot.driveTrain.drvTrainMtrCtrlRTFrnt);

            deltaTime = turnT.get() - timeStart;
            SmartDashboard.putNumber("Delta time", deltaTime);
        }
        if(turnT.get() > timeOut)
        {
            Robot.driveTrain.drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Velocity, 0);
            Robot.driveTrain.drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Velocity, 0);
        }

    }
    */

    public double leftEncoderDistanceMoved(double encoderPosition)
    {  
        //System.out.println((getIntegratedEncPosition("LT") - leftInitialEncoderPos) + " = " + getIntegratedEncPosition("LT") + " - " + leftInitialEncoderPos);
        return (encoderPosition - leftInitialEncoderPos) / ENCODER_COUNTS_PER_INCH_LT;
    }

    public double rightEncoderDistanceMoved()
    {   
        rightInitialEncoderPos = Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getSelectedSensorPosition(0);
        return (Robot.driveTrain.getIntegratedEncPosition("RT") - rightInitialEncoderPos) * ENCODER_COUNTS_PER_INCH_RT;
    }

}