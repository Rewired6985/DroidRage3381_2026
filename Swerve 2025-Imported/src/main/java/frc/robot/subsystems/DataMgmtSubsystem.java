package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DataMgmtSubsystem extends SubsystemBase
{

    public int driveMode = Constants.kd_AUTO;


    public boolean input3;
    public boolean input5;

    public DataMgmtSubsystem()
    {

    }

    public boolean input3() 
    {
        return input3;
    }

    public boolean input5()
    {
        return input5;
    }

}
