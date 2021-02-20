package frc.Mechanism;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class CatzIntake
{
    public CANSparkMax deployMC;

    private final int DEPLOY_MC_ID = 30; 

    public CatzIntake()
    {
        deployMC = new CANSparkMax(DEPLOY_MC_ID, MotorType.kBrushless);
    }

    public void deployIntake()
    {
        deployMC.set(1.0);
    }

    public void retractIntake()
    {
        deployMC.set(-1.0);
    }
}