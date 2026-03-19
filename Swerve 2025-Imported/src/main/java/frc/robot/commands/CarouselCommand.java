package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CarouselSubsystem;

public class CarouselCommand extends Command
{

    private final CarouselSubsystem ms_this;
    private double m_speed;
    
    public CarouselCommand(CarouselSubsystem subsystem, double speed)
    {
        ms_this = subsystem;
        m_speed = speed;
        addRequirements(subsystem);
    }
    

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {
        ms_this.set(m_speed);
    }

    @Override
    public void end(boolean interrupted)
    {
        ms_this.set(0);
    }
    
    @Override
    public boolean isFinished()
    {
        return false;
    }



}
