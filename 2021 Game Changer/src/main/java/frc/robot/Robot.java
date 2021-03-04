/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Mechanisms.CatzShooter;
import frc.Mechanisms.CatzElevator;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{


  public static CatzShooter shooter;
  public static CatzElevator elevator;
  
  public static XboxController xboxDrv;
  public static XboxController xboxAux;

  private final int XBOX_DRV_PORT = 0;
  private final int XBOX_AUX_PORT = 1;

  public final double ELE_POWER = 0.5;
  public final double ELE_FACTOR = 1.3;

  private final int DPAD_UP = 0;
  private final int DPAD_DN = 180;
  private final int DPAD_LT = 270;
  private final int DPAD_RT = 90;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */

  @Override
  public void robotInit() 
  {
    xboxDrv = new XboxController(XBOX_DRV_PORT);
    xboxAux = new XboxController(XBOX_AUX_PORT);

    shooter = new CatzShooter();
    elevator = new CatzElevator();
  
    

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

    SmartDashboard.putNumber("shaft velocity (RPM)", shooter.getFlywheelShaftVelocity());
    SmartDashboard.putNumber("shooterState", CatzShooter.shooterState);

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
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {
      //-----------------------shooter-----------------------
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
        elevator.runElevatorA(ELE_POWER);
        elevator.runElevatorB(ELE_POWER * ELE_FACTOR);
      }
      else
      {
        elevator.runElevatorA(0);
        elevator.runElevatorB(0);

      }
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() 
  {
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
