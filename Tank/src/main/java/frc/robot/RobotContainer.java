package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.HoodShooterCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodShooterSubsystem;
import frc.robot.subsystems.StateMachineSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeCommand;

public class RobotContainer 
{

  private final DrivetrainSubsystem   m_DrivetrainSubsystem   = new DrivetrainSubsystem();
  private final StateMachineSubsystem m_StateMachineSubsystem = new StateMachineSubsystem(); 
  private final HoodShooterSubsystem m_HoodShooterSubsystem = new HoodShooterSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem           = new IntakeSubsystem();

  private final CommandXboxController m_DriverGamepad = new CommandXboxController(0);
  private final CommandXboxController m_OperatorGamepad = new CommandXboxController(1);

  public RobotContainer() 
  {
    configureBindings();
  }

  private void configureBindings() 
  {
    m_DrivetrainSubsystem.setDefaultCommand(new DrivetrainCommand(m_DrivetrainSubsystem, m_StateMachineSubsystem, m_DriverGamepad));

    m_OperatorGamepad.a().onTrue(new HoodShooterCommand(m_HoodShooterSubsystem, m_StateMachineSubsystem, 0.95));
    m_OperatorGamepad.b().onTrue(new HoodShooterCommand(m_HoodShooterSubsystem, m_StateMachineSubsystem, 0));
    m_OperatorGamepad.y().onTrue(new IntakeCommand(m_IntakeSubsystem, m_StateMachineSubsystem,0.95));
    m_OperatorGamepad.x().onTrue(new IntakeCommand(m_IntakeSubsystem, m_StateMachineSubsystem,0));

  }

  public Command getAutonomousCommand() 
  {
    return null;
  }

}
