package frc.Autonomous;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.Mechanisms.*;
import frc.robot.*;
import com.kauailabs.navx.frc.AHRS;

public class CatzAutonomous
{

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

    public double currentEncCountRt;
    public double currentEncCountLt;
    public double turnRateRadians;

    public double r1;
    public double r2;
    public double s1Dot;
    public double s2Dot;
    public double s1Conv;
    public double s2Conv;

    public Timer turnT;
    public Timer driveTWait;
    public Timer shooterTimer;
    public Timer driveStraightTimer;

    public final double WAIT_TIME_THRESHOLD = 5.0;
    public double totalTime = 0;

    public double lVelocity;
    public double rVelocity;

    public int leftInitialEncoderCnt;
    public int rightInitialEncoderCnt;

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

    public final int DRIVE_STRAIGHT_TIMEOUT = 30;
    public boolean inDriveStraight = false;

    public final int MAX_VELOCITY_LIMIT = 3000;
    public final double VELOCITY_DECREASE_RATE = 0.98;

    public double targetVelocity = 0.0;

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
    public CatzPID turnPid;
    
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



    public CatzAutonomous()
    {
        turnT        = new Timer();
        driveTWait   = new Timer();
        shooterTimer = new Timer();
        driveStraightTimer = new Timer();
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

    

   


    /***********************************************************************
    * 
    * Convert velocity from ft/sec to counts/100msec
    *
    ************************************************************************/
    public double convertVelocity(double ftPerSec) 
    {
        double inPerSec = ftPerSec * (12/1); 
        double cntsPerSec = inPerSec * 1/Robot.driveTrain.encCountsToInches; 
        double cntsPer100Ms = cntsPerSec * 1/10;
        return cntsPer100Ms;
    }



    /*
     *Distance: input distance in inches that the robot has to go
     *maxSpeed: input speed in encoder counts per 100ms
     *timeout:  input timeout time in 
     */

    public void driveStraight(double distance, double maxSpeed, double timeout)
    {
        
        rightInitialEncoderCnt = Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getSelectedSensorPosition(0);

        distanceGoal = Math.abs(distance); //in inches

        if (distance < 0)
        {
            targetVelocity = -convertVelocity(maxSpeed);//cnts/100ms
        }
        else    
        {
            targetVelocity = convertVelocity(maxSpeed);
        }

        driveStraightLogConfig();
        //Robot.driveTrain.drvTrainMtrCtrlLTBack.set(targetVelocity);
        
        Robot.driveTrain.setTargetVelocity(targetVelocity);//targetVelocity);

        monitorEncoderPosition(rightInitialEncoderCnt, timeout);
    }
    

    public boolean monitorEncoderPosition(int initialEncoderCount, double timeoutTime)
    {
        CatzLog data;
        driveStraightTimer.reset();
        driveStraightTimer.start();

        boolean completed = false;
        boolean done      = false;
        double distanceRemaining;
        double currentVelocityRt  = 0.0;
        double currentVelocityLt  = 0.0;
        double deltaCounts;
        double currentTime = 0.0;
        double halfWay = distanceGoal * 0.5;
        //Robot.dataCollection.printInitialData();
        Robot.dataCollection.isFirst = true;
       while(done == false)
       {
            currentEncCountRt = Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getSelectedSensorPosition(0);//for left and right
            currentEncCountLt =Robot.driveTrain.drvTrainMtrCtrlLTFrnt.getSelectedSensorPosition(0);
            currentVelocityRt = Robot.driveTrain.getIntegratedEncVelocity("RT"); //for left and right
            currentVelocityLt = Robot.driveTrain.getIntegratedEncVelocity("LT");

            deltaCounts = currentEncCountRt - initialEncoderCount;
            distanceMoved = Math.abs((double) (deltaCounts * Robot.driveTrain.encCountsToInches) );
            
            distanceRemaining = distanceGoal - distanceMoved; //distance in inches (error)

            
            
            if (distanceRemaining < STOP_THRESHOLD_DIST)
            {
                targetVelocity = 0.0;
                Robot.driveTrain.setTargetVelocity(targetVelocity);
                completed = true;
                done = true;
            }
            else 
            {
                if (distanceRemaining < SLOW_THRESHOLD_DIST && distanceRemaining < halfWay) //TBD 
                {

                    
                    if(targetVelocity > MAX_VELOCITY_LIMIT)
                    { 
                        targetVelocity = targetVelocity * VELOCITY_DECREASE_RATE;
                        Robot.driveTrain.setTargetVelocity(targetVelocity);
                    }    
                    
                }
                currentTime = driveStraightTimer.get();
                if(currentTime > timeoutTime)
                {
                    targetVelocity = 0.0;
                    Robot.driveTrain.setTargetVelocity(targetVelocity);
                    completed = false;
                    done = true;
                }
            }
            if (Robot.dataCollection.isFirst == true)
            {
                //Robot.dataCollection.printInitialData();
                data = new CatzLog(Robot.driveTrain.PID_P,Robot.driveTrain.PID_I,Robot.driveTrain.PID_D,
                            Robot.driveTrain.PID_F,Robot.driveTrain.PID_IZ,Robot.driveTrain.encCountsToInches,
                            Robot.driveTrain.currentDrvTrainGear,distanceGoal,
                            -999.0,-999.0,-999.0,-999.0,-999.0,-999.0,-999.0,-999.0);
                Robot.dataCollection.logData.add(data);
                Robot.dataCollection.isFirst = false;
            }
            else 
            {
                data = new CatzLog(currentTime, targetVelocity, currentVelocityRt,
                                                                Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getClosedLoopError(0), 
                                                                currentEncCountRt, currentVelocityLt, Robot.driveTrain.drvTrainMtrCtrlLTFrnt.getClosedLoopError(0), currentEncCountLt, distanceRemaining, 
                                                                -999.0, -999.0, -999.0, -999.0, -999.0,-999.0, -999.0);
                Robot.dataCollection.logData.add(data);
                Robot.dataCollectionTimer.delay(0.04);
            }
           
                /*
                System.out.println(currentTime + "," +
                               targetVelocity + "," +
                               currentVelocity + "," +
                               Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getClosedLoopError(0) + "," +
                               currentEncCount + "," +
                               distanceRemaining );  */
                               
             
 
       }
        

        return completed;

        //SmartDashboard.putNumber("Encoder Position", currentEncCount);
        //SmartDashboard.putNumber("Initial Encoder Distance", leftInitialEncoderCnt);
        //SmartDashboard.putNumber("Distance Moved", distanceMoved);
        //SmartDashboard.putNumber("Delta Encoder", (currentEncCount - leftInitialEncoderCnt));
        //SmartDashboard.putNumber("Distance To Goal", distanceRemaining);
        //SmartDashboard.putNumber("total time", totalTime);
    }


    public void PIDturn(double degreesToTurn, double timeoutSeconds, double maxPower) {
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
                power = turnPid.getOutput(); //output is in rad/sec so we need to convert this
                Robot.driveTrain.setTargetPower(power);
                logDebugData();
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

    public static  double getCurrError()
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

    public void logDebugData()
    {
        CatzLog data;
        data = new CatzLog(functionTimer.get(), deltaT, 
        currentAngle, turnPid.getError(), power, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0,
        -999.0, -999.0, -999.0, -999.0, -999.0);
        Robot.dataCollection.logData.add(data);
    }

    public void driveStraightLogConfig()
    {
        System.out.println("T0: " + Robot.driveTrain.PID_P + "," 
                                  + Robot.driveTrain.PID_I + "," 
                                  + Robot.driveTrain.PID_D + "," 
                                  + Robot.driveTrain.PID_F + "," 
                                  + Robot.driveTrain.PID_IZ + "," 
                                  + Robot.driveTrain.encCountsToInches + "," 
                                  + Robot.driveTrain.currentDrvTrainGear + "," 
                                  + distanceGoal + ","
                                  + STOP_THRESHOLD_DIST
        );
        
    }
}