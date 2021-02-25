package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class CatzShooter
{
    public WPI_TalonFX shtrMCA;
    public WPI_TalonFX shtrMCB;

    private final int SHTR_MC_ID_A = 10; 
    private final int SHTR_MC_ID_B = 11; 

    public CatzShooter()
    {
        //initialize motor controllers
        shtrMCA = new WPI_TalonFX(SHTR_MC_ID_A);
        shtrMCB = new WPI_TalonFX(SHTR_MC_ID_B);
    }
}