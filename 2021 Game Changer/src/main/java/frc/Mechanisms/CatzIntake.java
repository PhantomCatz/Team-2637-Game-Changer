package frc.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class CatzIntake
{
    public CANSparkMax intakeDeployMC;

    private final int INTAKE_DEPLOY_MC_CAN_ID = 30; 

    public CatzIntake()
    {
        intakeDeployMC = new CANSparkMax(INTAKE_DEPLOY_MC_CAN_ID, MotorType.kBrushless);
    }

    public void deployIntake()
    {
        intakeDeployMC.set(1.0);
    }

    public void retractIntake()
    {
        intakeDeployMC.set(-1.0);
    }
}