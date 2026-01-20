package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.HoodShooterCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodShooterSubsystem;
import frc.robot.subsystems.DataMgmtSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeCommand;

public class RobotContainer 
{

  private final DrivetrainSubsystem   ms_drive   = new DrivetrainSubsystem();
  private final DataMgmtSubsystem ms_data = new DataMgmtSubsystem(); 
  private final HoodShooterSubsystem ms_hood = new HoodShooterSubsystem();
  private final IntakeSubsystem ms_intake           = new IntakeSubsystem();

  private final CommandXboxController m_DriverGamepad = new CommandXboxController(0);
  private final CommandXboxController m_OperatorGamepad = new CommandXboxController(1);

  public RobotContainer() 
  {
    configureBindings();
  }

  private void configureBindings() 
  {
    ms_drive.setDefaultCommand(new DrivetrainCommand(ms_drive, ms_data, m_DriverGamepad));

    m_OperatorGamepad.a().onTrue(new HoodShooterCommand(ms_hood, ms_data, 0.95));
    m_OperatorGamepad.b().onTrue(new HoodShooterCommand(ms_hood, ms_data, 0));
    m_OperatorGamepad.y().onTrue(new IntakeCommand(ms_intake, ms_data,0.95));
    m_OperatorGamepad.x().onTrue(new IntakeCommand(ms_intake, ms_data,0));

  }

  public Command getAutonomousCommand() 
  {
    return null;
  }

}
