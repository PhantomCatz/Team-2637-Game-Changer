package frc.Mechanism;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class CatzDrivetrain
{
    public WPI_TalonFX drvTrainMtrCtrlLTFrnt;
    public WPI_TalonFX drvTrainMtrCtrlLTBack;
    public WPI_TalonFX drvTrainMtrCtrlRTFrnt;
    public WPI_TalonFX drvTrainMtrCtrlRTBack;

    public final int DRVTRAIN_MC_LT_FRNT_CAN_ID = 1;
    public final int DRVTRAIN_MC_LT_BACK_CAN_ID = 2;
    public final int DRVTRAIN_MC_RT_FRNT_CAN_ID = 4;
    public final int DRVTRAIN_MC_RT_BACK_CAN_ID = 3;

    private SpeedControllerGroup drvTrainLT;
    private SpeedControllerGroup drvTrainRT;
        
    private DifferentialDrive drvTrainDifferentialDrive;

    private DoubleSolenoid gearShifter;

    private final int GEAR_SHIFTER_SOLENOID_PORT_A_PC = 0;
    private final int GEAR_SHIFTER_SOLENOID_PORT_B_PC = 1;  
    
    private boolean isDrvTrainInHighGear = true;

    private SupplyCurrentLimitConfiguration drvTrainCurrentLimit;

    private boolean enableCurrentLimit     = true; 
    private int currentLimitAmps           = 60;
    private int currentLimitTriggerAmps    = 80;
    private int currentLimitTimeoutSeconds = 5;

        
    public CatzDriveTrain()
    {
        drvTrainMtrCtrlLTFrnt = new WPI_TalonFX(DRVTRAIN_MC_LT_FRNT_CAN_ID);
        drvTrainMtrCtrlLTBack = new WPI_TalonFX(DRVTRAIN_MC_LT_BACK_CAN_ID)
        
        drvTrainMtrCtrlRTFrnt = new WPI_TalonFX(DRVTRAIN_MC_RT_FRNT_CAN_ID);
        drvTrainMtrCtrlRTBack = new WPI_TalonFX(DRVTRAIN_MC_RT_BACK_CAN_ID);

        //Set current limit
        drvTrainCurrentLimit = new SupplyCurrentLimitConfiguration(enableCurrentLimit, currentLimitAmps, currentLimitTriggerAmps, currentLimitTimeoutSeconds);

        drvTrainMtrCtrlLTFrnt.configSupplyCurrentLimit(drvTrainCurrentLimit);
        drvTrainMtrCtrlLTBack.configSupplyCurrentLimit(drvTrainCurrentLimit);
        drvTrainMtrCtrlRTFrnt.configSupplyCurrentLimit(drvTrainCurrentLimit);
        drvTrainMtrCtrlRTBack.configSupplyCurrentLimit(drvTrainCurrentLimit);
        
        drvTrainLT = new SpeedControllerGroup(drvTrainMtrCtrlLTFrnt, drvTrainMtrCtrlLTBack);
        drvTrainLT = new SpeedControllerGroup(drvTrainMtrCtrlRTFrnt, drvTrainMtrCtrlRTBack);
   
        drvTrainDifferentialDrive = new DifferentialDrive(drvTrainLT, drvTrainRT);

        gearShifter = new DoubleSolenoid(GEAR_SHIFTER_SOLENOID_PORT_A_PC, GEAR_SHIFTER_SOLENOID_PORT_B_PC);    
    }

    public void arcadeDrive(double power, double rotation)
    {
        drvTrainDifferentialDrive.arcadeDrive(power, rotation);
    }

    public void shiftToHighGear()
    {
        gearShifter.set(Value.kForward);
        isDrvTrainInHighGear = true;
    }

    public void shiftToLowGear()
    {
        gearShifter.set(Value.kReverse);
        isDrvTrainInHighGear = false;
    }
}