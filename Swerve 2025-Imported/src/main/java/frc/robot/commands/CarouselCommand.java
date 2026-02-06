package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CarouselSubsystem;
import frc.robot.subsystems.DataMgmtSubsystem;

public class CarouselCommand extends Command
{

    private final CarouselSubsystem ms_this;
    private final DataMgmtSubsystem ms_data;
    private CommandXboxController m_gamepad;
    
    public CarouselCommand (CarouselSubsystem subsystem, DataMgmtSubsystem state_subsystem, CommandXboxController gamepad)
    {
        ms_this = subsystem;
        ms_data = state_subsystem;
        m_gamepad = gamepad;
        addRequirements(subsystem);
    }

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {
        ms_this.setPower(m_gamepad.getRightTriggerAxis());
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
