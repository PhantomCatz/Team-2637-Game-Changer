package frc.Mechanisms;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class CatzIntake
{
    public WPI_TalonSRX intakeRollerMC;
    public CANSparkMax intakeDeployMC;

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
    private final double INTAKE_MOTOR_POWER_START_STOW   = -0.25;
    private final double INTAKE_MOTOR_POWER_END_STOW     = -0.25;

    private Thread intakeThread;

    private final int INTAKE_MODE_NULL                = 0;
    private final int INTAKE_MODE_DEPLOY_START        = 1;
    private final int INTAKE_MODE_DEPLOY_REDUCE_POWER = 2;
    private final int INTAKE_MODE_STOW_START          = 3;
    private final int INTAKE_MODE_STOW_REDUCE_POWER   = 4;

    private int intakeMode = INTAKE_MODE_NULL;

    boolean intakeDeployed = false;

    final double INTAKE_THREAD_WAITING_TIME       = 0.050;
    final double DEPLOY_REDUCE_POWER_TIME_OUT_SEC = 0.400;
    final double STOW_REDUCE_POWER_TIME_OUT_SEC   = 0.350;

    public CatzIntake()
    {
        intakeRollerMC = new WPI_TalonSRX(INTAKE_ROLLER_MC_CAN_ID);
        intakeDeployMC = new CANSparkMax(INTAKE_DEPLOY_MC_CAN_ID, MotorType.kBrushless);
        
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
                        intakeDeployMC.set(INTAKE_MOTOR_POWER_START_STOW);
                        intakeMode = INTAKE_MODE_STOW_REDUCE_POWER;

                        timeCounter++;
                    break;

                    case INTAKE_MODE_STOW_REDUCE_POWER:
                        if(timeCounter > stowPowerCountLimit)
                        {
                            intakeDeployMC.set(INTAKE_MOTOR_POWER_END_STOW);
                            intakeMode = INTAKE_MODE_NULL;
                            intakeDeployed = false;
                        }
                        timeCounter++;
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
    public void stopDeploying()
    {
        intakeDeployMC.set(0);
    }

    public void applyBallCompression()
    {
        intakeDeployMC.set(COMPRESSION_POWER);
    }

    public double getDeployMotorPower()
    {
        return intakeDeployMC.get();
    }

    // ---------------------------------------------Intake Limit Switches---------------------------------------------   
    public boolean getDeployedLimitSwitchState()
    {
        return intakeDeployedLimitSwitch.get();
    }
    public boolean getStowedLimitSwitchState()
    {
        return intakeStowedLimitSwitch.get();
    }
}