package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intakedeploy;
import frc.robot.subsystems.Intake;

public class FeederCommand extends Command {

    private Feeder ms_this;
    private CommandXboxController m_controller;

    public FeederCommand(Shooter subsystem, CommandXboxController controller)
    {
        ms_this = subsystem;
        m_controller = controller;
    }
    
}
