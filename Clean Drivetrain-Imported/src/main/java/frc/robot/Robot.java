/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.kauailabs.navx.frc.AHRS;

import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;

import frc.Autonomous.CatzAutonomous;

//import frc.Mechanisms.CatzShooter;
//import frc.Mechanisms.CatzElevator;
import frc.Mechanisms.CatzIntake;
import frc.Mechanisms.CatzDriveTrain;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  //public static CatzShooter shooter;
  //public static CatzElevator elevator;
  public static CatzIntake intake;
  public static CatzDriveTrain driveTrain;

  public static CatzAutonomous auton;
  public static Timer autonomousTimer;

  public static DataCollection dataCollection;
  public static CatzLog        catzLog;
  public static Timer dataCollectionTimer;
  public ArrayList<CatzLog> dataArrayList;

  public static PowerDistributionPanel pdp;

  public static AHRS navx;
  
//-------------------------------------------Drive Controller------------------------------------------------------------- 
  public static XboxController xboxDrv;
  public static XboxController xboxAux;

  private final int XBOX_DRV_PORT = 0;
  private final int XBOX_AUX_PORT = 1;
  private final int DPAD_UP = 0;
  private final int DPAD_DN = 180;
  private final int DPAD_LT = 270;
  private final int DPAD_RT = 90;

//-------------------------------------------Elevator------------------------------------------------------------- 
  public final double ELEV_POWER = 0.5;
  public final double ELEV_FACTOR = 1.3;

//-------------------------------------------Path Chooser------------------------------------------------------------- 
  
  
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */

  @Override
  public void robotInit() 
  {
    xboxDrv = new XboxController(XBOX_DRV_PORT);
    xboxAux = new XboxController(XBOX_AUX_PORT);

    driveTrain = new CatzDriveTrain();
    intake = new CatzIntake();
    auton = new CatzAutonomous();
    /*shooter = new CatzShooter();
    elevator = new CatzElevator();*/
    dataCollection = new DataCollection();
  
    navx = new AHRS(Port.kMXP, (byte)200);
    navx.reset();
    
    pdp = new PowerDistributionPanel();
    
    autonomousTimer     = new Timer();
    dataCollectionTimer = new Timer();

    dataCollection.dataCollectionInit(dataArrayList);
    dataCollectionTimer.reset();
    dataCollectionTimer.start();
    dataCollection.setLogDataID(dataCollection.LOG_ID_DRV_STRAIGHT);
    //dataCollection.setLogDataID(dataCollection.LOG_ID_DRV_TURN);
    dataCollection.startDataCollection();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override

  public void robotPeriodic() 
  {
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() 
  {
    auton.driveStraight(120, 14, 100);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() 
  {


  }


  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() 
  {
    driveTrain.instantiateDifferentialDrive();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override

  public void teleopPeriodic() 
  {
    //-------------------------------------------Drivetrain------------------------------------------------------------- 
    driveTrain.arcadeDrive(xboxDrv.getY(Hand.kLeft), xboxDrv.getX(Hand.kRight));
  
    if(xboxDrv.getBumper(Hand.kLeft))
    {
      driveTrain.shiftToHighGear();
    }
    else if(xboxDrv.getBumper(Hand.kRight))
    {
      driveTrain.shiftToLowGear();
    }
    //-------------------------------------------Intake------------------------------------------------------------- 
    if(xboxDrv.getStickButtonPressed(Hand.kLeft))
    {
      intake.deployIntake(); 
    }
    else if(xboxDrv.getStickButtonPressed(Hand.kRight))
    {
      intake.stowIntake();
    }
    
    if(xboxDrv.getTriggerAxis(Hand.kLeft) > 0.2)
    {
      intake.intakeRollerIn();
    }
    else if(xboxDrv.getTriggerAxis(Hand.kRight) > 0.2)
    {
      intake.intakeRollerOut();
    }
    else
    {
      intake.intakeRollerOff();
    }
    /*//-----------------------shooter-----------------------
      if(xboxAux.getPOV() == DPAD_UP)
      {
        shooter.setTargetRPM(shooter.SHOOTER_TARGET_RPM_LO);
        //shooter.setTargetVelocity(.25);
      }
      else if(xboxAux.getPOV() == DPAD_LT)
      {
      shooter.setTargetRPM(shooter.SHOOTER_TARGET_RPM_MD);
      }
      else if(xboxAux.getPOV() == DPAD_DN)
      {
        shooter.setTargetRPM(shooter.SHOOTER_TARGET_RPM_HI);
      }
      else if(xboxAux.getBButton())
      {
        //indexer.setShooterIsRunning(true);
        shooter.shoot();
      } 
      else if(xboxAux.getStartButton())
      {
        shooter.shooterOff();
      }  

    //-----------------------Elevator-----------------------

      if(xboxDrv.getAButton() == true)
      {
        elevator.runElevatorA(ELE_POWER)V;
        elevator.runElevatorB(ELE_POWER * ELEV_FACTOR);
      }
      else
      {
        elevator.runElevatorA(0);
        elevator.runElevatorB(0);

      }*/
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() 
  {
    dataCollection.stopDataCollection();
    Robot.driveTrain.drvTrainMtrCtrlLTFrnt.setNeutralMode(NeutralMode.Coast); 
    Robot.driveTrain.drvTrainMtrCtrlLTBack.setNeutralMode(NeutralMode.Coast);
    Robot.driveTrain.drvTrainMtrCtrlRTFrnt.setNeutralMode(NeutralMode.Coast);
    Robot.driveTrain.drvTrainMtrCtrlRTBack.setNeutralMode(NeutralMode.Coast);

    try 
    {
      dataCollection.exportData(dataArrayList);
    } catch (Exception e) 
    {
      e.printStackTrace();
    }

  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() 
  {
  }

  /**
   * This function is called once when test mode is enabled.
   */
  @Override
  public void testInit() 
  {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() 
  {
  }
}