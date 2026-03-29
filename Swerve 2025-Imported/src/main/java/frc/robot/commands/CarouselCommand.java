package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CarouselSubsystem;

public class CarouselCommand extends Command
{

    private final CarouselSubsystem ms_this;
    
    public CarouselCommand(CarouselSubsystem subsystem)
    {
        ms_this = subsystem;
        addRequirements(subsystem);
    }
    

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {
        SmartDashboard.putNumber("car speed", ms_this.getVelocity());
        ms_this.setSpeed(60);
    }

    @Override
    public void end(boolean interrupted)
    {
        ms_this.setSpeed(0);
    }
    
    @Override
    public boolean isFinished()
    {
        return false;
    }



}
