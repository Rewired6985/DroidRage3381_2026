package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.eAuto;
import frc.robot.Constants.eAutoGoal;

public class DrivetrainSubsystem extends SubsystemBase
{

    public double m_Xswerve = 0;
    public double m_Yswerve = 0;
    public double m_Rswerve = 0;

    public boolean inPosition       = false;
    public boolean depotCollected   = true;
    public boolean outpostCollected = true;
    public boolean goalAccomplished = false;

    public eAutoGoal goal = eAutoGoal.NOMOVE;

    public eAuto state = eAuto.REST;

    public DrivetrainSubsystem()
    {
        
    }

    public void autoHandler()
    {
        switch (state)
        {
            case REST:
            {
                if (goalAccomplished == false) state = eAuto.TOCENTER;
                break;
            }
            case TODEPOT:
            {
                if (inPosition) state = eAuto.COLLECTDEPOT;
                break;
            }
            case TOOUTPOST:
            {
                if (inPosition) state = eAuto.COLLECTOUTPOST;
                break;
            }
            case TOCENTER:
            {
                if (inPosition)
                {
                    switch (goal)
                    {
                        case DEPOT:   state = eAuto.TODEPOT;   break;
                        case OUTPOST: state = eAuto.TOOUTPOST; break;
                        case DEPOT_THEN_OUTPOST:
                        { 
                            if (depotCollected)      state = eAuto.TOOUTPOST;
                            else                     state = eAuto.TODEPOT;
                            break;
                        }
                        case OUTPOST_THEN_DEPOT:
                        {
                            if (outpostCollected) state = eAuto.TODEPOT;
                            else                  state = eAuto.TOOUTPOST;
                            break;
                        }
                        default: state = eAuto.REST; break;
                    }

                }
                break;
            }
            case COLLECTDEPOT:
            {
                if (depotCollected)
                {
                    if (goal == eAutoGoal.DEPOT_THEN_OUTPOST) state = eAuto.TOCENTER;
                    else                                           state = eAuto.REST;
                }
                break;
            }
            case COLLECTOUTPOST:
            {
                if (outpostCollected)
                {
                    if (goal == eAutoGoal.OUTPOST_THEN_DEPOT) state = eAuto.TOCENTER;
                    else                                           state = eAuto.REST;
                }
                break;
            }
        }
    }
    
    
    public void updateParams(Constants.eAutoGoal argGoal)
    {
        goal = argGoal;
    }

}
