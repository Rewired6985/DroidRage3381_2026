package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodShooterSubsystem;
import frc.robot.subsystems.DataMgmtSubsystem;

public class HoodShooterCommand extends Command
{

    private final DataMgmtSubsystem ms_data;
    private final HoodShooterSubsystem ms_this;
    private double m_power;

    public HoodShooterCommand(HoodShooterSubsystem subsystem, DataMgmtSubsystem data_subsystem, double power)
    {
        ms_this  = subsystem;
        ms_data = data_subsystem;
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
