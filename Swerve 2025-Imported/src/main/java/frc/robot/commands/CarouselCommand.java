package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.CarouselSubsystem;
import frc.robot.subsystems.DataMgmtSubsystem;

public class CarouselCommand extends Command
{

    private final CarouselSubsystem ms_this;
    private final DataMgmtSubsystem ms_data;
    private CommandXboxController m_controller;
    private Joystick m_joystick;
    private boolean usingJoystick;
    
    public CarouselCommand(CarouselSubsystem subsystem, DataMgmtSubsystem state_subsystem, CommandXboxController controller)
    {
        ms_this       = subsystem;
        ms_data       = state_subsystem;
        m_controller  = controller;
        usingJoystick = false;
        addRequirements(subsystem);
    }
        
    public CarouselCommand(CarouselSubsystem subsystem, DataMgmtSubsystem state_subsystem, Joystick joystick)
    {
        ms_this       = subsystem;
        ms_data       = state_subsystem;
        m_joystick    = joystick;
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
        switch (ms_data.driveMode)
        {
            case Constants.kd_TELEOP:
            {
                if (usingJoystick)
                {
                    if (m_joystick.getRawButton(2))
                    {
                        ms_this.setPower(0.5);
                    }
                    else
                    {
                        ms_this.setPower(0);
                    }
                }
                else
                {
                    ms_this.setPower(m_controller.getRightTriggerAxis());
                }
                break;
            }
            case Constants.kd_AUTO:
            {
                break;
            }
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        ms_this.setPower(0);
    }
    
    @Override
    public boolean isFinished()
    {
        return false;
    }



}
