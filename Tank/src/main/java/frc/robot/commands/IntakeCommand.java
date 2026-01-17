package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StateMachineSubsystem;

public class IntakeCommand extends Command
{

    private IntakeSubsystem m_subsystem;
    private StateMachineSubsystem ms_subsystem;
    private double m_power;

    public IntakeCommand(IntakeSubsystem subsystem, StateMachineSubsystem state_subsystem, double power)
    {
        m_subsystem = subsystem;
        ms_subsystem = state_subsystem;
        m_power = power;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){}

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
