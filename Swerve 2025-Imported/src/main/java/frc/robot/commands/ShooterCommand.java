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

    private double targetVelocity = 0;

    private double shooterSpeed = 0;
    private int accCounter = 0;
    private int dclCounter = 0;

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
        
        double turretSpeed = 0;
        double useSpeed = 0;
       
        boolean accelerate = false;
        boolean decelerate = false;
        boolean trigger = false;
        boolean swivel_left = false;
        boolean swivel_right = false;

        ms_data.aim.fuelVelocity = ((ms_data.aim.distance)/
                            (Math.cos(ms_data.aim.angle) * 
                           Math.sqrt((2*(Constants.SHOOTERHEIGHT - ms_data.aim.target[2] + 
                                        (ms_data.aim.distance * Math.tan(ms_data.aim.angle))))/
                                     (Constants.GRAVITY))));

        targetVelocity = ms_data.aim.fuelVelocity * 57.29578;

        
        // SmartDashboard.putNumber("target velocity", targetVelocity);

        switch (ms_data.Mode)
        {
            case TELEOP:
            {
                if (usingJoystick)
                {
                    accelerate = m_joystick.getRawButton(5);
                    decelerate = m_joystick.getRawButton(10);
                    trigger = m_joystick.getRawButton(1);
                    swivel_left = m_joystick.getRawButton(6);
                    swivel_right = m_joystick.getRawButton(9);
                }
                else 
                {
                    accelerate = m_controller.rightBumper().getAsBoolean();
                    decelerate = m_controller.leftBumper().getAsBoolean();
                    trigger = m_controller.b().getAsBoolean();
                    swivel_left = m_controller.povLeft().getAsBoolean();
                    swivel_right = m_controller.povRight().getAsBoolean();
                }

                //makes it so that every button press increments/decrements shooter shooterSpeed
                if (decelerate) dclCounter = dclCounter + 1;
                else dclCounter = 0;

                if (accelerate) accCounter = accCounter + 1;
                else accCounter = 0;

                if (accCounter == 1) shooterSpeed = shooterSpeed + 0.05;
                if (dclCounter == 1) shooterSpeed = shooterSpeed - 0.05;
                
                if (shooterSpeed > 0.95) shooterSpeed = 0.95;
                if (shooterSpeed < 0) shooterSpeed = 0;
                
                if (trigger) useSpeed = shooterSpeed;
                else         useSpeed = 0;

                if      (swivel_left)  turretSpeed = 0.05;
                else if (swivel_right) turretSpeed = -0.05;
                else                   turretSpeed = 0;
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

        
        // SmartDashboard.putNumber("shooter speed",   shooterSpeed);
        // SmartDashboard.putNumber("turret position", ms_this.getTurretPosition());
        ms_this.setShooterSpeed(useSpeed);
        ms_this.setTurretSpeed(turretSpeed);
        
        // SmartDashboard.putNumber("TurretPosition", ms_this.getTurretPosition());
        // SmartDashboard.putNumber("ShooterSpeed", useSpeed);

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
