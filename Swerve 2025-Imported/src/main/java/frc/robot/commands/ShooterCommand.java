package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.DataMgmtSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command
{

    private final DataMgmtSubsystem ms_data;
    private final ShooterSubsystem ms_this;
    private Joystick m_joystick;
    private CommandXboxController m_controller;
    private boolean usingJoystick;

    private double power = 0;
    private int accCounter;
    private int dclCounter;

    public ShooterCommand(ShooterSubsystem subsystem, DataMgmtSubsystem data_subsystem, Joystick joystick)
    {
        ms_this  = subsystem;
        ms_data = data_subsystem;
        m_joystick = joystick;
        usingJoystick = true;
        addRequirements(subsystem);
    }


     public ShooterCommand(ShooterSubsystem subsystem, DataMgmtSubsystem data_subsystem, CommandXboxController controller)
    {
        ms_this  = subsystem;
        ms_data = data_subsystem;
        m_controller = controller;
        usingJoystick = false;
        addRequirements(subsystem);
    }

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {

        double usePower;

        switch (ms_data.driveMode)
        {
            case Constants.kd_TELEOP:
            {
                if (usingJoystick)
                {

                    if (m_joystick.getRawButton(4)) dclCounter = dclCounter + 1;
                    else dclCounter = 0;

                    if (m_joystick.getRawButton(6)) accCounter = accCounter + 1;
                    else accCounter = 0;

                    if (accCounter == 1) power = power + 0.05;
                    if (dclCounter == 1) power = power - 0.05;

                    if (m_joystick.getRawButton(1)) usePower = power;
                    else                                   usePower = 0;

                    SmartDashboard.putNumber("power", power);

                    ms_this.setPower(usePower);

                }
                break;
            }
            case Constants.kd_AUTO:
            {
                break;
            }
        }
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
