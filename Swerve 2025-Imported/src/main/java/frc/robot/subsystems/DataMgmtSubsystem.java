package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DataMgmtSubsystem extends SubsystemBase
{

    public Constants.eMode Mode = Constants.eMode.AUTO;

    public class aimStruct
    {
        public Constants.eAim state = Constants.eAim.ZERO;

        public boolean hasTarget  = false;
        public boolean isTracking = false;
        public boolean inDefense  = false;
        public boolean inEndgame  = false;
    }

    public class intakeStruct
    {
        public Constants.eIntake state = Constants.eIntake.STOP;

        public boolean inPosition = false;
        public boolean retractNow = false;
        public boolean speedZero  = false;
        public boolean deployNow  = false;
    }

    public class inputStruct
    {
        public boolean brake;
        public boolean reset;
        public boolean switchIntake;
    }

    
    public aimStruct    aim    = new aimStruct();
    public intakeStruct intake = new intakeStruct();
    public inputStruct  inputs = new inputStruct();

    public DataMgmtSubsystem()
    {

    }

    public void ShotHandler()
    {

        switch (aim.state)
        {
            case ZERO: 
            {
                if (aim.hasTarget) aim.state = Constants.eAim.HUNT;
                break;
            }
            case HUNT:
            {
                if (aim.isTracking) aim.state = Constants.eAim.STANDBY;
                break;
            }
            case FIRE: 
            {
                if (aim.hasTarget = false) aim.state = Constants.eAim.STANDBY;
                break;
            }
            case STANDBY: 
            {
                if (aim.hasTarget) aim.state = Constants.eAim.FIRE;
                if (aim.inDefense) aim.state = Constants.eAim.ZERO;
                if (aim.inEndgame) aim.state = Constants.eAim.ZERO;
                break;
            }
        }

    }

    public void IntakeHandler()
    {
        switch (intake.state)
        {
            case DEPLOY:
            {
                if (intake.inPosition) intake.state = Constants.eIntake.RUN;
                break;
            }
            case RUN:
            {
                if (intake.retractNow) intake.state = Constants.eIntake.STOP;
                break;
            }
            case STOP:
            {
                if (intake.speedZero) intake.state = Constants.eIntake.RETRACT;
                break;
            }
            case RETRACT:
            {
                if (intake.deployNow) intake.state = Constants.eIntake.DEPLOY;
                break;
            }
        }
    }

    public boolean brake() 
    {
        return inputs.brake;
    }

    public boolean reset()
    {
        return inputs.reset;
    }

    public boolean switchIntake()
    {
        return inputs.switchIntake;
    }

}
