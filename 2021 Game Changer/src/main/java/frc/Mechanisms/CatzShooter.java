package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Robot;

public class CatzShooter
{
    public WPI_TalonFX shtrMCA;
    public WPI_TalonFX shtrMCB;

    private final int SHTR_MC_ID_A = 10; 
    private final int SHTR_MC_ID_B = 11; 

    final double COUNTS_PER_REVOLUTION      = 2048.0;
    final double SEC_TO_MIN                 = 60.0;
    final double ENCODER_SAMPLE_RATE_MSEC   = 100.0;
    final double ENCODER_SAMPLE_PERIOD_MSEC = (1.0 / ENCODER_SAMPLE_RATE_MSEC);
    final double MSEC_TO_SEC                = 1000.0;
    final double FLYWHEEL_GEAR_REDUCTION    = 3.0;

    final double CONV_QUAD_VELOCITY_TO_RPM = ( ((ENCODER_SAMPLE_PERIOD_MSEC * MSEC_TO_SEC * SEC_TO_MIN) / COUNTS_PER_REVOLUTION)); //converts velocity to RPM

    public static final int SHOOTER_STATE_OFF                 = 0;
    public static final int SHOOTER_STATE_RAMPING             = 1;
    public static final int SHOOTER_STATE_SET_SPEED           = 2;
    public static final int SHOOTER_STATE_READY               = 3;
    public static final int SHOOTER_STATE_START_SHOOTING      = 4;
    public static final int SHOOTER_STATE_WAIT_FOR_SHOOT_DONE = 5;
    
    public final double SHOOTER_RPM_START_OFFSET =  250.0;
    public final double SHOOTER_TARGET_RPM_LO    = (4700.0 - SHOOTER_RPM_START_OFFSET); //4400
    public final double SHOOTER_TARGET_RPM_MD    = (5000.0 - SHOOTER_RPM_START_OFFSET);
    public final double SHOOTER_TARGET_RPM_HI    = (6000.0 - SHOOTER_RPM_START_OFFSET);

    final double SHOOTER_BANG_BANG_MAX_RPM_OFFSET = 5.0; 
    final double SHOOTER_BANG_BANG_MIN_RPM_OFFSET = 5.0;

    final double SHOOTER_RAMP_RPM_OFFSET = 200.0;

    final double SHOOTER_OFF_POWER   =  0.0;
    final double SHOOTER_RAMP_POWER  = -1.0;
    final double SHOOTER_SHOOT_POWER = -1.0;

    final int NUM_OF_DATA_SAMPLES_TO_AVERAGE = 5;

    final double SHOOTER_THREAD_PERIOD           = 0.040;
    final long   SHOOTER_THREAD_PERIOD_MS        = 40;
    final double SHOOTER_RAMP_TIMEOUT_SEC        = 4.000;  //TBD-TEST put back to 4.0
    final double INDEXER_SHOOT_TIME_SEC          = 1.60;
    final double SHOOTER_AVG_VEL_SAMPLE_TIME_SEC = 0.100;

    public double targetRPM          = 0.0;
    public double targetRPMThreshold = 0.0;
    public double shooterPower       = 0.0;
    public double minPower           = 0.0;
    public double maxPower           = 0.0;

    public static int shooterState = SHOOTER_STATE_OFF;

    private int indexerShootStateCountLimit  = 0;
    private int rampStateCountLimit          = 0;
    private int indexerShootStateCount       = 0;
    private int rampStateCount               = 0;
    private int samplingVelocityCount        = 0;
    private int samplingVelocityCountLimit   = 0;

    private boolean shooterIsReady = false;
    public  boolean shooterIsDone  = true;
    double avgVelocity             = 0.0;

    private Thread shooterThread;

    public boolean inAutonomous;

    public CatzShooter()
    {
        //initialize motor controllers
        shtrMCA = new WPI_TalonFX(SHTR_MC_ID_A);
        shtrMCB = new WPI_TalonFX(SHTR_MC_ID_B);

        shtrMCA.configFactoryDefault();
        shtrMCB.configFactoryDefault();

        //shtrMCB.follow(shtrMCA);

        shtrMCA.setNeutralMode(NeutralMode.Coast);
        shtrMCB.setNeutralMode(NeutralMode.Coast);

        //limits how long the shooter runs so it doesn't go too long (limiter)
        indexerShootStateCountLimit = (int)Math.round( (INDEXER_SHOOT_TIME_SEC / SHOOTER_THREAD_PERIOD) + 0.5 ); 
        //equation which limits how long the ramp up goes (don't want it to go too much/fast)
        rampStateCountLimit         = (int)Math.round( (SHOOTER_RAMP_TIMEOUT_SEC / SHOOTER_THREAD_PERIOD) + 0.5);
        //equation which determines the time between each sample (flywheel velocity)
        samplingVelocityCountLimit  = (int)Math.round( (SHOOTER_AVG_VEL_SAMPLE_TIME_SEC / SHOOTER_THREAD_PERIOD) + 0.5);
        
        setShooterVelocity();
        shooterOff();
    }

    public void setShooterVelocity() //will make shooter run and etc
    {

        shooterThread = new Thread(() -> //start of thread
        {
            double flywheelShaftVelocity    = -1.0;
            double minRPM                   = 0.0;
            double maxRPM                   = 0.0;
            double shootTime                = 0.0;
            double[] velocityData           = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            double sumOfVelocityData        = 0.0;
            int velocityDataIndex           = 0;
            boolean readyToCalculateAverage = false;
            boolean rumbleSet               = false;

            while(true)
            {
                //shootTime = Robot.dataCollectionTimer.get();
                flywheelShaftVelocity = getFlywheelShaftVelocity();

                switch (shooterState)
                {
                    case SHOOTER_STATE_OFF: //when there is no targetRPM (basically when no button is pressed) will be shooter most of the time
                        shooterPower = SHOOTER_OFF_POWER;

                        if(targetRPM > 0.0)
                        {
                            indexerShootStateCount  = 0;
                            rampStateCount          = 0;
                            samplingVelocityCount   = 0;
                            sumOfVelocityData       = 0.0;
                            velocityDataIndex       = 0;
                            readyToCalculateAverage = false;
                            shooterState            = SHOOTER_STATE_RAMPING;
                            targetRPMThreshold      = targetRPM - SHOOTER_RAMP_RPM_OFFSET;
                            minRPM                  = targetRPM - SHOOTER_BANG_BANG_MIN_RPM_OFFSET;
                            maxRPM                  = targetRPM + SHOOTER_BANG_BANG_MAX_RPM_OFFSET;
                            shooterPower            = SHOOTER_RAMP_POWER;
                            rumbleSet               = false;
            			    shooterIsDone           = false;

                            getBangBangPower();
                            shtrMCA.set(shooterPower);
                            shtrMCB.set(-shooterPower);

                            for(int i = 0; i < NUM_OF_DATA_SAMPLES_TO_AVERAGE; i++ )
                            {
                                velocityData[i] = 0.0;
                            }

                            System.out.println("T1: " + shootTime + " : " + flywheelShaftVelocity + " Power: " + shooterPower);
                        }

                    break;

                    case SHOOTER_STATE_RAMPING: // once targetRPM is given, velocity ramps up as fast as possible to reach targetRPM
                        //Robot.indexer.setShooterRamping(true);
                        if(flywheelShaftVelocity > targetRPMThreshold)
                        {
                            shooterState = SHOOTER_STATE_SET_SPEED;
                            shooterPower = maxPower;
                            shtrMCA.set(shooterPower);
                            shtrMCB.set(-shooterPower);
                            System.out.println("T2: " + shootTime + " : " + flywheelShaftVelocity + " Power: " + shooterPower );

                        }
                        rampStateCount++;
                        if(rampStateCount > rampStateCountLimit)
                        {
                            shooterOff();
                        }
                        break;

                    case SHOOTER_STATE_SET_SPEED: // making bang bang work. adds up RPM (prerequisite for bang bang, checks average of )
                        samplingVelocityCount++;
                        if(samplingVelocityCount > samplingVelocityCountLimit)
                        {
                            samplingVelocityCount = 0;
                            velocityData[velocityDataIndex++ ] = flywheelShaftVelocity;
                            System.out.print("S");
                            //Robot.indexer.setShooterRamping(false);
                            if(velocityDataIndex == NUM_OF_DATA_SAMPLES_TO_AVERAGE)
                            {
                                velocityDataIndex = 0;
                                readyToCalculateAverage = true;
                            }

                            if(readyToCalculateAverage == true)
                            {
                                
                                for(int i = 0; i < NUM_OF_DATA_SAMPLES_TO_AVERAGE; i++ )
                                {  
                                    sumOfVelocityData = sumOfVelocityData + velocityData[i];
                                }
                        
                                avgVelocity = sumOfVelocityData / NUM_OF_DATA_SAMPLES_TO_AVERAGE;
                                sumOfVelocityData = 0.0;
                                System.out.println("AD: " + avgVelocity);
                            }

                            if(avgVelocity > minRPM && avgVelocity < maxRPM)
                            {
                                shooterState = SHOOTER_STATE_READY;
                            }
                        }

                        bangBang(minRPM, maxRPM, flywheelShaftVelocity);

                        System.out.println("T3: " + shootTime + " : " + flywheelShaftVelocity + " Power: " + shooterPower);

                    break;

                    case SHOOTER_STATE_READY:// makes the controller vibrate so that aux driver knows to shoot
                        shooterIsReady = true;
                        System.out.println("SSR");
                        bangBang(minRPM, maxRPM, flywheelShaftVelocity);   

                        if(inAutonomous == true)
                        { 
                        }
                        else 
                        {
                            if(rumbleSet == false)
                            {
                                Robot.xboxAux.setRumble(RumbleType.kLeftRumble, 0.5);
                                rumbleSet = true;
                            } 
                        }
                    break;

                    case SHOOTER_STATE_START_SHOOTING: 
                        shooterPower = SHOOTER_SHOOT_POWER;
                        shtrMCA.set(shooterPower);   
                        shtrMCB.set(-shooterPower); 
                        System.out.println("TS1: " + shootTime + " : " + flywheelShaftVelocity + " Power: " + shooterPower);

                        if(flywheelShaftVelocity > targetRPM + SHOOTER_RPM_START_OFFSET)
                        {
                            //Robot.indexer.indexerStart(); 
                            shooterState = SHOOTER_STATE_WAIT_FOR_SHOOT_DONE;
                            System.out.println("TS2: " + shootTime + " : " + flywheelShaftVelocity + " Power: " + shooterPower);
                        }
                    break;

                    case SHOOTER_STATE_WAIT_FOR_SHOOT_DONE: //will count for a certain amount of time until it switches the shooter off and sets state to OFF
                        System.out.println("TS3: " + shootTime + " : " + flywheelShaftVelocity + " Power: " + shooterPower);
                        indexerShootStateCount++;
                        if(indexerShootStateCount > indexerShootStateCountLimit)
                        {
                            shooterOff();
                            //Robot.indexer.indexerStop(); 
                            //Robot.indexer.setShooterIsRunning(false);
                            System.out.println("TS4: " + shootTime + " : " + flywheelShaftVelocity + " Power: " + shooterPower);
                        }
                
                    break;

                    default:  //default code when there is nothing going on 
                        shooterOff();
                    break;
            }        
            Timer.delay(SHOOTER_THREAD_PERIOD);
        }
    }); //end of thread
        shooterThread.start();
    }

    public void getBangBangPower() //determines max and min power based on the velocity chosen
    {//10000) + 0.04
       double power =  (targetRPM / 6380.0); //+0.05    
       minPower = -(power - 0.05);
       maxPower = -(power + 0.05);
    }

    public void bangBang(double minRPM, double maxRPM, double flywheelShaftVelocity) // bangbang method
    {
        if (flywheelShaftVelocity > maxRPM)
        {
            shooterPower = minPower; 
        }
        else if(flywheelShaftVelocity < minRPM) 
        {
            shooterPower = maxPower;
        }
        shtrMCA.set(shooterPower);
        shtrMCB.set(-shooterPower);
    }

    public double getFlywheelShaftPosition() //encoder counts
    {
        return shtrMCA.getSensorCollection().getIntegratedSensorPosition();
    }

    public double getFlywheelShaftVelocity() //RPM
    {
        return (Math.abs((double) shtrMCA.getSensorCollection().getIntegratedSensorVelocity() * CONV_QUAD_VELOCITY_TO_RPM)); 
    }

    public void setTargetRPM(double velocity) // Sets the RPM (determined by the button pressed on controller)
    {
        targetRPM = velocity;
    }

    public void shoot() // when a button is pressed on controller, it will cause the indexer to move and launch balls. sets state to ready
    {
        if(shooterState == SHOOTER_STATE_READY)
        {
            indexerShootStateCount = 0;
            shooterState = SHOOTER_STATE_START_SHOOTING;
        }
    }

    public void shooterOff() // turns shooter off , sets the shooter state to off
    {
        targetRPM      = 0.0;
        shooterIsReady = false;
	    shooterIsDone  = true;
        shooterState   = SHOOTER_STATE_OFF;
        shooterPower   = SHOOTER_OFF_POWER;
        shtrMCA.set(shooterPower);
        shtrMCB.set(-shooterPower);
        //Robot.indexer.setShooterIsRunning(false);
        Robot.xboxAux.setRumble(RumbleType.kLeftRumble, 0);
    
    }
}