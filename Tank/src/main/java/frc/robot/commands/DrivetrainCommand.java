package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.StateMachineSubsystem;

public class DrivetrainCommand extends Command 
{
    private final DrivetrainSubsystem m_subsystem;
    private final StateMachineSubsystem ms_subsystem;
    private final CommandXboxController m_gamepad;
    
    public double LeftPower;
    public double RightPower;

    public double gamepadX;
    public double gamepadY;

    public DrivetrainCommand(DrivetrainSubsystem subsystem, StateMachineSubsystem state_subsystem, CommandXboxController gamepad)
    {
        m_subsystem  = subsystem;
        ms_subsystem = state_subsystem;
        m_gamepad    = gamepad;
        addRequirements(subsystem);
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {

        gamepadX = m_gamepad.getLeftX() * 0.95;
        gamepadY = m_gamepad.getLeftY() * 0.95;

        LeftPower = gamepadY + gamepadX;
        RightPower= gamepadY - gamepadX;

        if      (LeftPower >  1)  LeftPower  =  1;
        else if (LeftPower < -1)  LeftPower  = -1;

        if      (RightPower >  1) RightPower =  1;
        else if (RightPower < -1) RightPower = -1;

        m_subsystem.setPower(LeftPower, RightPower);
    }

    @Override
    public void end(boolean interrupted)
    {
        m_subsystem.setPower(0,0);
    }
    
    @Override
    public boolean isFinished()
    {
        return false;
    }

}
