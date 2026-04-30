package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CarouselSubsystem;
import frc.robot.subsystems.DataMgmtSubsystem;

public class CarouselCommand extends Command
{

    private final CarouselSubsystem ms_this;
    private final DataMgmtSubsystem ms_data;
    
    public CarouselCommand(CarouselSubsystem subsystem, DataMgmtSubsystem data_subsystem)
    {
        ms_this = subsystem;
        ms_data = data_subsystem;
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
        ms_this.setSpeed(ms_data.carouselSpeed);
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
