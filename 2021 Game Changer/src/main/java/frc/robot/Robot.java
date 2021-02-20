/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import java.util.ArrayList;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Autonomous.CatzAutonomousPaths;
import frc.Autonomous.CatzPathChooser;
import frc.Autonomous.CatzAutonomous;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.Mechanisms.CatzClimber;
import frc.Mechanisms.CatzDriveTrain;
import frc.Mechanisms.CatzIndexer;
import frc.Mechanisms.CatzIntake;
import frc.Mechanisms.CatzShooter;
import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
  /**
   *  Mechanisms
   */
  public static CatzDriveTrain driveTrain;
  public static CatzIntake     intake;
  public static CatzShooter    shooter;
  public static CatzIndexer    indexer;
  public static CatzAutonomous auton;


  public static DataCollection dataCollection;
  public static CatzLog        catzLog;

  public static AHRS navx;

  // Xbox Controllers
  private static XboxController xboxDrv;
  public  static XboxController xboxAux;

  private final int XBOX_DRV_PORT = 0;
  private final int XBOX_AUX_PORT = 1;

  public static PowerDistributionPanel pdp;

  public static Timer dataCollectionTimer;
  public static Timer autonomousTimer;

  public ArrayList<CatzLog> dataArrayList; 

  public boolean testFlag = false;

  public boolean check_boxL = false;
  public boolean check_boxM = false;
  public boolean check_boxR = false;

	public boolean prev_boxL = false;
	public boolean prev_boxM = false;
  public boolean prev_boxR = false;

  private final int DPAD_UP = 0;
  private final int DPAD_DN = 180;
  private final int DPAD_LT = 270;
  private final int DPAD_RT = 90;

  // Camera Settings

  @Override
  public void robotInit() 
  {
    xboxDrv = new XboxController(XBOX_DRV_PORT);
    xboxAux = new XboxController(XBOX_AUX_PORT);

    driveTrain = new CatzDriveTrain();
    intake     = new CatzIntake();
    auton      = new CatzAutonomous();   
    
    navx = new AHRS(Port.kMXP, (byte)200);
    navx.reset();
    
    pdp = new PowerDistributionPanel();

    dataArrayList = new ArrayList<CatzLog>();

    dataCollection = new DataCollection();

    dataCollectionTimer = new Timer();
    autonomousTimer     = new Timer();
    
    dataCollection.dataCollectionInit(dataArrayList);

    
    //create a path chooser
    SmartDashboard.putBoolean(CatzConstants.POSITION_SELECTORL, true);
    SmartDashboard.putBoolean(CatzConstants.POSITION_SELECTORM, true);
    SmartDashboard.putBoolean(CatzConstants.POSITION_SELECTORR, true);
      
    SmartDashboard.putBoolean(CatzConstants.POSITION_SELECTORL, false);
    SmartDashboard.putBoolean(CatzConstants.POSITION_SELECTORM, false);
    SmartDashboard.putBoolean(CatzConstants.POSITION_SELECTORR, false);
    
    SmartDashboard.putBoolean("Use default autonomous?", false);

    SmartDashboard.putBoolean(CatzConstants.TEAM_SELECTORB, true);
    SmartDashboard.putBoolean(CatzConstants.TEAM_SELECTORR, true);

    SmartDashboard.putBoolean(CatzConstants.TEAM_SELECTORB, false);
    SmartDashboard.putBoolean(CatzConstants.TEAM_SELECTORR, false);
  
    // Camera Configuration
  }

  @Override
  public void robotPeriodic() 
  {
    //path chooser safety code
    check_boxL = SmartDashboard.getBoolean(CatzConstants.POSITION_SELECTORL, false);
		check_boxM = SmartDashboard.getBoolean(CatzConstants.POSITION_SELECTORM, false);
		check_boxR = SmartDashboard.getBoolean(CatzConstants.POSITION_SELECTORR, false);

    if ((check_boxL != prev_boxL) && (check_boxL == true)) 
    {
			prev_boxL = check_boxL;
			prev_boxM = false;
			prev_boxR = false;
    }
     else if ((check_boxM != prev_boxM) && (check_boxM == true)) 
    {
			prev_boxL = false;
			prev_boxM = check_boxM;
			prev_boxR = false;
    } 
    else if ((check_boxR != prev_boxR) && (check_boxR == true)) 
    {
			prev_boxL = false;
			prev_boxM = false;
			prev_boxR = check_boxR;
    }
    
		// Update display
		SmartDashboard.putBoolean(CatzConstants.POSITION_SELECTORL, prev_boxL);
		SmartDashboard.putBoolean(CatzConstants.POSITION_SELECTORM, prev_boxM);
		SmartDashboard.putBoolean(CatzConstants.POSITION_SELECTORR, prev_boxR);

    SmartDashboard.putBoolean("Deployed", intake.getDeployedLimitSwitchState());
    SmartDashboard.putBoolean("Stowed",   intake.getStowedLimitSwitchState());


    SmartDashboard.putNumber("deploy mtr ctrl", intake.getDeployMotorPower());
    }

  @Override
  public void autonomousInit() 
  {
    driveTrain.shiftToHighGear();
    auton.driveStraight(100, 10, 1000);
    auton.PIDturn(1800, 5, 0.45);
    dataCollection.dataCollectionInit(dataArrayList);
    dataCollectionTimer.reset();
    dataCollectionTimer.start();
    dataCollection.setLogDataID(dataCollection.LOG_ID_DRV_STRAIGHT);
    //dataCollection.setLogDataID(dataCollection.LOG_ID_DRV_TURN);
    dataCollection.startDataCollection();
    
    //auton.run(0.5);
    
    //shooter.autonomousOn();
  }

  @Override
  public void autonomousPeriodic() 
  {
   //// driveTrain.drvTrainLtBack.set(0.3);///use this to test
    //driveTrain.drvTrainMtrCtrlLTBack.set(0.3);
    //driveTrain.drvTrainLT.set(0.3);
    //auton.driveStraight(30.0, 0.5, 10);

  }

  @Override
  public void teleopInit() 
  {

    driveTrain.instantiateDifferentialDrive();

    dataCollection.dataCollectionInit(dataArrayList);
    dataCollectionTimer.reset();
    dataCollectionTimer.start();
    dataCollection.setLogDataID(dataCollection.LOG_ID_SHOOTER);
    dataCollection.startDataCollection();

    //driveTrain.shiftToLowGear();
  }

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

    

    

    //-----------------------------------------------INTAKE---------------------------------------------------
    if(xboxDrv.getStickButtonPressed(Hand.kLeft) && intake.getDeployedLimitSwitchState() == false)
    {
      intake.deployIntake();
    }
    else if(xboxDrv.getStickButtonPressed(Hand.kRight) && intake.getStowedLimitSwitchState() == false)
    {
      intake.stowIntake();
    }
    /*else if(xboxDrv.getAButton() == true)
    {
      intake.applyBallCompression();
    } 
    /*else
    {
      intake.stopDeploying();
    }*/

    if(xboxDrv.getTriggerAxis(Hand.kLeft) > 0.2)
    {
      intake.intakeRollerIn();
      intake.applyBallCompression();
    }
    else if(xboxDrv.getTriggerAxis(Hand.kRight) > 0.2)
    {
      intake.intakeRollerOut();
    }
    else
    {
      intake.intakeRollerOff();
    }

    if(xboxDrv.getXButton())
    {
      intake.changeMode(5);
    }
    else if(xboxDrv.getBButton())
    {
      intake.changeMode(0);
    }

    //--------------------------------------------SHOOTER-------------------------------------------------
    if(xboxAux.getPOV() == DPAD_UP)
    {
      //shooter.setTargetRPM(shooter.SHOOTER_TARGET_RPM_LO);
      //shooter.setTargetVelocity(.25);
    }
    else if(xboxAux.getPOV() == DPAD_LT)
    {
     //shooter.setTargetRPM(shooter.SHOOTER_TARGET_RPM_MD);
    }
    else if(xboxAux.getPOV() == DPAD_DN)
    {
      //shooter.setTargetRPM(shooter.SHOOTER_TARGET_RPM_HI);
    }
    else if(xboxAux.getBButton())
    {
      //indexer.setShooterIsRunning(true);
      //shooter.shoot();
    } 
    else if(xboxAux.getStartButton())
    {
      //shooter.shooterOff();
      //indexer.indexerStop();
    }

   //----------------------------------------------INDEXER----------------------------------------
   if(xboxAux.getBackButton())
   {
    //indexer.indexerReversedOn();
   } 
   else
   {
     //indexer.indexerReversedOff();
   }

  

  //--------------------------------------------------TESTING----------------------------------------------
   /*if(xboxAux.getAButton()) //TBD is A and B used on aux for different purpose
   {
     shooter.logTestData = true;
   }
   if(xboxAux.getBButton())
   {
     shooter.logTestData = false;
   }*/
  }

  public void testPeriodic() 
  {
    
    
  }
  
  @Override
  public void disabledInit() 
  {
    dataCollection.stopDataCollection();
    
    /*for (int i = 0; i <dataArrayList.size();i++)
    {
       System.out.println(dataArrayList.get(i));
    }*/

    try 
    {
      dataCollection.exportData(dataArrayList);
    } catch (Exception e) 
    {
      e.printStackTrace();
    } 
  }

}