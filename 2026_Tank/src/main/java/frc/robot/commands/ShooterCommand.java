package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command
{
    private Shooter ms_this;
    private CommandXboxController m_controller;

     public ShooterCommand(Shooter subsystem, CommandXboxController controller)
    {
        ms_this = subsystem;
        m_controller = controller;
    }

     @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
        
    }
}
