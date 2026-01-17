package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DoubleShooterSubsystem;
import frc.robot.subsystems.StateMachineSubsystem;

public class DoubleShooterCommand extends Command
{

    private final StateMachineSubsystem  ms_subsystem;
    private final DoubleShooterSubsystem m_subsystem;
    private double m_power;

    public DoubleShooterCommand(DoubleShooterSubsystem subsystem, StateMachineSubsystem state_subsystem, double power)
    {
        m_subsystem  = subsystem;
        ms_subsystem = state_subsystem;
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
        m_subsystem.setPower(m_power);
    }

    @Override
    public void end(boolean interrupted)
    {
        m_subsystem.setPower(0);
    }
    
    @Override
    public boolean isFinished()
    {
        return false;
    }


}
