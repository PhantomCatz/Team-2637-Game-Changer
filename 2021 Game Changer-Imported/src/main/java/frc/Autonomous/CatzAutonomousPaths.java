package frc.Autonomous;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.Autonomous.CatzAutonomous;
import frc.robot.*;
import frc.Mechanisms.*;

public class CatzAutonomousPaths
{
    static CatzAutonomous auton = new CatzAutonomous();

    public boolean spunUp = false;
    public boolean doneShooting = false;
    public static boolean done;

    public final int DRIVE_DIST_HIGH_SPEED = 16000;
    public final int DRIVE_DIST_MED_SPEED = 10000;
    public final int DRIVE_DIST_LOW_SPEED = 7500;
    public final int TURN_SPEED = 12000;

    public static boolean goalMet;

    public Timer driveStraightT;

    public final static double STRAIGHT_BLACKTOP_POWER_PORT_DIST      = -78-1;
    public final static double STRAIGHT_BLACKTOP_PAST_START_LINE_DIST = 130;

    public final static int STRAIGHT_BLACKTOP_MED_SPEED  = 12000;
    public final static int STRAIGHT_BLACKTOP_HIGH_SPEED = 16000;

    public final static double SIDE_BLACKTOP_DRIVE_DIST = 24;
    
    public final static int SIDE_BLACKTOP_DRIVE_SPEED = 12000;

    public final static double DEFAULT_DIST = 60;

    public final static int DEFAULT_SPEED = 12000;
    
    public final static double STRAIGHT_BLUE_PAST_START_LINE_DIST = 0;
    public final static double STRAIGHT_RED_PAST_START_LINE_DIST  = 0;

    public final static int STRAIGHT_BLUE_MED_SPEED  = 12000;
    public final static int STRAIGHT_BLUE_HIGH_SPEED = 16000;

    public final static int STRAIGHT_RED_MED_SPEED  = 12000;
    public final static int STRAIGHT_RED_HIGH_SPEED = 16000;    

    public final static double STRAIGHT_BLUE_POWER_PORT_DIST= 0.0;
    public final static double STRAIGHT_RED_POWER_PORT_DIST = 0.0;

    public final static double SIDE_BLUE_DRIVE_DIST= 0.0;
    public final static double SIDE_RED_DRIVE_DIST = 0.0;

    public final static int STRAIGHT_BLUE_DRIVE_SPEED = 0;
    public final static int STRAIGHT_RED_DRIVE_SPEED  = 0;

    public final static int SIDE_BLUE_DRIVE_SPEED = 0;
    public final static int SIDE_RED_DRIVE_SPEED  = 0;

    

    public enum AUTO_STATE 
    {
        AS_INIT, AS_DRIVETO, AS_DRIVEBACK, AS_DONE, AS_CHECK_SHOOTER_DONE, AS_WAIT_FOR_SHOOTER_READY;
    }

    public enum AUTO_STATE2 
    {
        AS_INIT, AS_DRIVETO,  AS_DONE;
    }
    public static AUTO_STATE autoState = AUTO_STATE.AS_INIT;
    public static AUTO_STATE2 autoState2 = AUTO_STATE2.AS_INIT;

    public CatzAutonomousPaths() 
    {
        done = false;
        driveStraightT = new Timer();
    }

    /**
     * 
     * @param position - The path the robot will be taking (e.g. "STRAIGHT")
     */


    



   
    

}