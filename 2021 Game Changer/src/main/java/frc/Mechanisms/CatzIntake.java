package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.CatzLog;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

public class CatzIntake
{
    public WPI_TalonSRX intakeRollerMC;
    public CANSparkMax intakeDeployMC;

    public CANEncoder intakeDeployEncoder;

    private final int INTAKE_ROLLER_MC_CAN_ID = 31;
    private final int INTAKE_DEPLOY_MC_CAN_ID = 30; 

    private CANDigitalInput intakeDeployedLimitSwitch;
    private CANDigitalInput intakeStowedLimitSwitch;

    private int deployPowerCountLimit = 0;
    private int stowPowerCountLimit   = 0;
    private int timeCounter = 0;

    private final double MTR_POWER_ROLLER =  0.6;

    private final double COMPRESSION_POWER = 0.3;

    private final double INTAKE_MOTOR_POWER_START_DEPLOY =  0.25;
    private final double INTAKE_MOTOR_POWER_END_DEPLOY   =  0.0;
    private final double INTAKE_MOTOR_POWER_START_STOW   = -0.40;
    private final double INTAKE_MOTOR_POWER_END_STOW     = -0.25;

    private Thread intakeThread;

    private final int INTAKE_MODE_NULL                = 0;
    private final int INTAKE_MODE_DEPLOY_START        = 1;
    private final int INTAKE_MODE_DEPLOY_REDUCE_POWER = 2;
    private final int INTAKE_MODE_STOW_START          = 3;
    private final int INTAKE_MODE_STOW_REDUCE_POWER   = 4;
    private final int INTAKE_MODE_WAIT_FOR_HARD_STOP  = 5;
    private final int INTAKE_MODE_STOW_HOLD           = 6;

    private int intakeMode = INTAKE_MODE_NULL;

    boolean intakeDeployed = false;

    final double INTAKE_THREAD_WAITING_TIME       = 0.020;
    final double DEPLOY_REDUCE_POWER_TIME_OUT_SEC = 0.400;
    final double STOW_REDUCE_POWER_TIME_OUT_SEC   = 0.450;

    final double intakeStowMotorGearRatio = 1.0/49.0;

    private double currentIntakeStowPosition;
    private double lastIntakeStowPosition;
    private double targetIntakeStowPosition;
    private double currentIntakeStowPower;
    private int intakeCheckHardstopCount = 0;

    private final int INTAKE_MAX_HARD_STOP_COUNT          = 7;
    private final double INTAKE_STOW_HOLD_THRESHOLD       = 15.0;
    private final double INTAKE_CHECK_HARD_STOP_THRESHOLD = 1.0;

    private double intakeStowHoldkP = 0.0001;

    private boolean dataCollectionState = false; //true for on, flase for off. Turns on or off datacollection for this class
    private CatzLog data;

    private Timer intakeTimer;

    public CatzIntake()
    {
        intakeRollerMC = new WPI_TalonSRX(INTAKE_ROLLER_MC_CAN_ID);
        intakeDeployMC = new CANSparkMax(INTAKE_DEPLOY_MC_CAN_ID, MotorType.kBrushless);
        
        intakeDeployEncoder = new CANEncoder(intakeDeployMC);

        intakeDeployedLimitSwitch = intakeDeployMC.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
        intakeStowedLimitSwitch   = intakeDeployMC.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

        //Reset configuration
        intakeRollerMC.configFactoryDefault();
        intakeDeployMC.restoreFactoryDefaults();

        //Set roller MC to coast intakeMode
        intakeRollerMC.setNeutralMode(NeutralMode.Coast);

        //Set deploy MC to brake intakeMode
        intakeDeployMC.setIdleMode(IdleMode.kBrake);

        deployPowerCountLimit = (int) Math.round((DEPLOY_REDUCE_POWER_TIME_OUT_SEC / INTAKE_THREAD_WAITING_TIME) + 0.5);
        stowPowerCountLimit   = (int) Math.round((STOW_REDUCE_POWER_TIME_OUT_SEC   / INTAKE_THREAD_WAITING_TIME) + 0.5);
        SmartDashboard.putNumber("stow count limit", stowPowerCountLimit);
        SmartDashboard.putNumber("deploy count limit", deployPowerCountLimit);

        intakeTimer = new Timer();
    }

    // ---------------------------------------------ROLLER---------------------------------------------
    public void intakeRollerIn()
    {
        intakeRollerMC.set(ControlMode.PercentOutput, -MTR_POWER_ROLLER);
    }
    public void intakeRollerOut()
    {
        intakeRollerMC.set(ControlMode.PercentOutput, MTR_POWER_ROLLER);
    }
    public void intakeRollerOff()
    {
        intakeRollerMC.set(ControlMode.PercentOutput, 0.0);
    }

    // ---------------------------------------------DEPLOY/STOW---------------------------------------------
    public void intakeControl()
    {
        intakeThread = new Thread(() ->
        { 
            while(true)
            {
                switch(intakeMode)
                {
                    case INTAKE_MODE_DEPLOY_START:
                        intakeDeployMC.set(INTAKE_MOTOR_POWER_START_DEPLOY);
                        intakeMode = INTAKE_MODE_DEPLOY_REDUCE_POWER;
                        timeCounter++;
                    break;

                    case INTAKE_MODE_DEPLOY_REDUCE_POWER:
                        if(timeCounter > deployPowerCountLimit)
                        {
                            intakeDeployMC.set(INTAKE_MOTOR_POWER_END_DEPLOY);
                            intakeMode = INTAKE_MODE_NULL;
                            intakeDeployed = true;
                        }
                        timeCounter++;
                    break;

                    case INTAKE_MODE_STOW_START:
                        intakeDeployEncoder.setPosition(0);
                        intakeDeployMC.set(INTAKE_MOTOR_POWER_START_STOW);
                        intakeMode = INTAKE_MODE_STOW_REDUCE_POWER;

                        timeCounter++;
                    break;

                    case INTAKE_MODE_STOW_REDUCE_POWER:
                        if(timeCounter > stowPowerCountLimit)
                        {
                            intakeDeployMC.set(INTAKE_MOTOR_POWER_END_STOW);
                            intakeCheckHardstopCount = 0;
                            intakeMode = INTAKE_MODE_WAIT_FOR_HARD_STOP;
                            intakeDeployed = false;
                        }
                        timeCounter++;
                    break;

                    case INTAKE_MODE_WAIT_FOR_HARD_STOP:
                        currentIntakeStowPosition = getIntakeDeployPositionDegrees();
                        double difference = lastIntakeStowPosition - currentIntakeStowPosition;
                        if(Math.abs(difference) <= INTAKE_CHECK_HARD_STOP_THRESHOLD)
                        {
                            intakeCheckHardstopCount ++;
                            if(intakeCheckHardstopCount >= INTAKE_MAX_HARD_STOP_COUNT)
                            {
                                targetIntakeStowPosition = currentIntakeStowPosition;
                                currentIntakeStowPower = INTAKE_MOTOR_POWER_END_STOW;
                                intakeTimer.reset();
                                intakeTimer.start();
                                intakeMode = INTAKE_MODE_STOW_HOLD;
                            }
                        }

                        lastIntakeStowPosition = currentIntakeStowPosition;
                    break;

                    case INTAKE_MODE_STOW_HOLD:
                        currentIntakeStowPosition = getIntakeDeployPositionDegrees();
                        double error = currentIntakeStowPosition - targetIntakeStowPosition;
                        double currentTime = intakeTimer.get();
                        double power = 0.0;
                        
                        if(Math.abs(error) >= INTAKE_STOW_HOLD_THRESHOLD)
                        {
                            power = clamp(currentIntakeStowPower + (error * intakeStowHoldkP), -1.0, INTAKE_MOTOR_POWER_END_STOW);
                            currentIntakeStowPower = power;
                            intakeDeployMC.set(power);
                        }else{
                            intakeDeployMC.set(0.0);
                        }

                        if(dataCollectionState)
                        {
                            System.out.println("current: " + currentIntakeStowPosition + " target: " + targetIntakeStowPosition + " error: " + error + " power: " + power);
                            data = new CatzLog(currentTime, targetIntakeStowPosition, currentIntakeStowPosition, error, power, 
                                                -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0);
                            Robot.dataCollection.logData.add(data);
                        }
                    break;

                    default:
                        intakeDeployMC.set(0.0);
                    break;
                }
                Timer.delay(INTAKE_THREAD_WAITING_TIME); //put at end
                
            }   
        });
        intakeThread.start();
    }
    public void deployIntake()
    {
        timeCounter = 0;
        intakeMode = INTAKE_MODE_DEPLOY_START;
    }
    public void stowIntake()
    {
        timeCounter = 0;
        intakeMode = INTAKE_MODE_STOW_START;
    }

    public double getIntakeDeployPositionDegrees(){
        return Math.abs(intakeDeployEncoder.getPosition() * intakeStowMotorGearRatio * 360.0);
    }

    public double clamp(double value, double min, double max){
        if (value < min)
            return min;
        if (value > max)
            return max;
        return value;
    }
}