package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DataMgmtSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command
{

    private final DataMgmtSubsystem ms_data;
    private final ShooterSubsystem ms_this;
    private Joystick m_joystick;
    private CommandXboxController m_controller;
    private boolean usingJoystick;

    private double speed = 0;
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

        double useSpeed = 0;

        
        switch (ms_data.Mode)
        {
            case TELEOP:
            {
                if (usingJoystick)
                {

                    //makes it so that every button press increments/decrements shooter speed
                    if (m_joystick.getRawButton(4)) dclCounter = dclCounter + 1;
                    else dclCounter = 0;

                    if (m_joystick.getRawButton(6)) accCounter = accCounter + 1;
                    else accCounter = 0;

                    if (accCounter == 1) speed = speed + 0.05;
                    if (dclCounter == 1) speed = speed - 0.05;
                    
                    if (speed > 1) speed = 1;
                    if (speed < 0) speed = 0;
                    
                    if (m_joystick.getRawButton(1)) useSpeed = speed;
                    else                                   useSpeed = 0;

                }
                break;
            }
            case AUTO:
            {
                break;
            }
        }

        switch (ms_data.aim.state)
        {
            case ZERO: 
            {
                break;
            }
            case HUNT:
            {
                break;
            }
            case FIRE: 
            {
                break;
            }
            case STANDBY: 
            {
                break;
            }
        }

        
        SmartDashboard.putNumber("speed", speed);
        ms_this.setShooterSpeed(useSpeed);

    }

    @Override
    public void end(boolean interrupted)
    {
        ms_this.setShooterSpeed(0);
    }
    
    @Override
    public boolean isFinished()
    {
        return false;
    }

}
