package frc.Autonomous;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.Mechanisms.*;
import frc.robot.*;

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

    

   


    /*******8*****88********8*******************88*****8*8**8****8***8*8*****
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
                Robot.dataCollection.printInitialData();
                Robot.dataCollection.isFirst = false;
            }
            else 
            {
                data = new CatzLog(currentTime, targetVelocity, currentVelocityRt,
                                                                Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getClosedLoopError(0), 
                                                                currentEncCountRt, currentVelocityLt, Robot.driveTrain.drvTrainMtrCtrlLTFrnt.getClosedLoopError(0), currentEncCountLt, distanceRemaining, 
                                                                -999.0, -999.0, -999.0, -999.0, -999.0,-999.0, -999.0);
                Robot.dataCollection.logData.add(data);
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