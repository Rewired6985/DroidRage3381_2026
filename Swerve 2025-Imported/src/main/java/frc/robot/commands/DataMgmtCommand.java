package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DataMgmtSubsystem;

public class DataMgmtCommand extends Command
{

    private final DataMgmtSubsystem ms_this;
    private Joystick m_joystick;
    private boolean usingJoystick;

    public DataMgmtCommand(DataMgmtSubsystem subsystem)
    {
        ms_this = subsystem;
        usingJoystick = false;
        addRequirements(subsystem);
    }

    public DataMgmtCommand(DataMgmtSubsystem subsystem, Joystick joystick)
    {
        ms_this = subsystem;
        m_joystick = joystick;
        usingJoystick = true;
        addRequirements(subsystem);
    }

    @Override 
    public void initialize()
    {
        
    }
   
    @Override
    public void execute()
    {
        //calls state handlers
        ms_this.IntakeHandler();
        ms_this.ShotHandler();

        switch (ms_this.Mode)
        {
            case TELEOP:
            {
                if (usingJoystick)
                {
                    ms_this.inputs.switchIntake = true;
                }
                break;
            }
            case AUTO:
            {
                break;
            }
        }



    }

    @Override
    public void end(boolean interrupted)
    {

    }

    @Override 
    public boolean isFinished()
    {
        return false;
    }
}
