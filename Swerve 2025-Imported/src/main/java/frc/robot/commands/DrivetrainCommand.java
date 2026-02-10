package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.DataMgmtSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainCommand extends Command
{

    private final DrivetrainSubsystem ms_this;
    private final DataMgmtSubsystem   ms_data;
    
    private Joystick m_joystick;
    private CommandXboxController m_controller;
    private boolean usingJoystick;

    public DrivetrainCommand(DrivetrainSubsystem subsystem, DataMgmtSubsystem data_subsystem, Joystick joystick)
    {
        ms_this = subsystem;
        ms_data = data_subsystem;
        m_joystick = joystick;
        usingJoystick = true;
        addRequirements(subsystem);
    }

    public DrivetrainCommand(DrivetrainSubsystem subsystem, DataMgmtSubsystem data_subsystem, CommandXboxController controller)
    {
        ms_this = subsystem;
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

        double inputX = 0;
        double inputY = 0;
        double inputR = 0;
        double inputT = 0;

        switch (ms_data.driveMode)
        {
            case Constants.kd_TELEOP:
            {
                if (usingJoystick) 
                {
                    inputX = -m_joystick.getY() * 0.9;
                    inputY = -m_joystick.getX() * 0.9;
                    inputR = -m_joystick.getZ() * 0.9;
                    inputT = ((m_joystick.getThrottle() + 1) * 0.4) + 0.2;

                    ms_this.m_Xswerve = addDeadZone(inputX, 0.02) * inputT;
                    ms_this.m_Yswerve = addDeadZone(inputY, 0.1)  * inputT;
                    ms_this.m_Rswerve = addDeadZone(inputR, 0.4)  * inputT;

                    ms_data.input3 = m_joystick.getRawButton(3);
                    ms_data.input5 = m_joystick.getRawButton(5);

                }
                else
                {
                    inputX = -m_controller.getRightY() * 0.7;
                    inputY = -m_controller.getRightX() * 0.7;
                    inputR = -m_controller.getLeftX()  * 0.7;

                    ms_this.m_Xswerve = addDeadZone(inputX, 0);
                    ms_this.m_Yswerve = addDeadZone(inputY, 0);
                    ms_this.m_Rswerve = addDeadZone(inputR, 0);
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

    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    private double addDeadZone(double input, double limit)
    {
        double output;

        if      (input < -limit) output = (input + limit)/(1-limit);
        else if (input >  limit) output = (input - limit)/(1-limit);
        else output = 0;

        return output;
    }


}
