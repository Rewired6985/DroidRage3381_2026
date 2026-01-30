package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CarouselSubsystem;
import frc.robot.subsystems.DataMgmtSubsystem;

public class CarouselCommand extends Command
{

    private final CarouselSubsystem ms_this;
    private final DataMgmtSubsystem ms_data;
    private double m_power;
    
    public CarouselCommand (CarouselSubsystem subsystem, DataMgmtSubsystem state_subsystem, double power)
    {
        ms_this = subsystem;
        ms_data = state_subsystem;
        m_power = power;
        addRequirements(subsystem);
    
    }

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {
        ms_this.setPower(m_power);
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
