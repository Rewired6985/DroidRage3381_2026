package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.eAuto;
import frc.robot.Constants.eAutoGoal;
import frc.robot.Constants.eInitPose;

public class DrivetrainSubsystem extends SubsystemBase
{

    public double m_Xswerve = 0;
    public double m_Yswerve = 0;
    public double m_Rswerve = 0;

    public eInitPose initPose = eInitPose.MIDDLE;

    
    public DrivetrainSubsystem()
    {
        
    }


    public boolean inPosition       = false;
    public boolean depotCollected   = false;
    public boolean outpostCollected = false;
    public boolean goalAccomplished = false;

    public eAutoGoal goal = eAutoGoal.NOMOVE;

    public eAuto state = eAuto.REST;
    

    public void autoHandler()
    {
        switch (state)
        {
            case REST:
            {
                if (goalAccomplished == false) 
                {
                    switch (goal)
                    {
                        case DEPOT: //do the rollover thing :D
                        case DEPOT_THEN_OUTPOST:
                        {
                            if (initPose == eInitPose.LEFT) state = eAuto.TODEPOT;
                            else                            state = eAuto.TOCENTER;
                            break;
                        }
                        case OUTPOST: //do the rollover thing :D
                        case OUTPOST_THEN_DEPOT:
                        {
                            if (initPose == eInitPose.RIGHT) state = eAuto.TOOUTPOST;
                            else                             state = eAuto.TOCENTER;
                            break;
                        }
                        case NOMOVE: break;
                    }
                }
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
                            if (depotCollected) state = eAuto.TOOUTPOST;
                            else                state = eAuto.TODEPOT;
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
                    else                                      state = eAuto.REST;
                }
                break;
            }
            case COLLECTOUTPOST:
            {
                if (outpostCollected)
                {
                    if (goal == eAutoGoal.OUTPOST_THEN_DEPOT) state = eAuto.TOCENTER;
                    else                                      state = eAuto.REST;
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
