package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class CatzElevator
{
    public WPI_TalonSRX elvtrMCA;
    public WPI_TalonSRX elvtrMCB;

    private final int ELVTR_MC_ID_A = 20; 
    private final int ELVTR_MC_ID_B = 3; 

    public CatzElevator()
    {
        elvtrMCA = new WPI_TalonSRX(ELVTR_MC_ID_A);
        elvtrMCB = new WPI_TalonSRX(ELVTR_MC_ID_B);
    }

    public void runElevatorA(double power)
    {
        elvtrMCA.set(-power);
    }

    public void runElevatorB(double power)
    {
        elvtrMCB.set(power);
    }
}