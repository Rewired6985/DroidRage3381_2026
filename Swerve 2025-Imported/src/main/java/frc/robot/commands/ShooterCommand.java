package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DataMgmtSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import frc.robot.PIDFController;

public class ShooterCommand extends Command
{

    private final DataMgmtSubsystem ms_data;
    private final ShooterSubsystem ms_this;
    private Joystick m_joystick;
    private CommandXboxController m_controller;
    private boolean m_usingJoystick;

    private double m_targetVelocity = 0;
    private double m_turretTarget = 0;
   

    private PIDFController m_turretPID = new PIDFController(0.02,0,0,0);

    public ShooterCommand(ShooterSubsystem subsystem, DataMgmtSubsystem data_subsystem, Joystick joystick)
    {
        ms_this  = subsystem;
        ms_data = data_subsystem;
        m_joystick = joystick;
        m_usingJoystick = true;
        addRequirements(subsystem);
    }


     public ShooterCommand(ShooterSubsystem subsystem, DataMgmtSubsystem data_subsystem, CommandXboxController controller)
    {
        ms_this  = subsystem;
        ms_data = data_subsystem;
        m_controller = controller;
        m_usingJoystick = false;
        addRequirements(subsystem);
    }

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {
        
        // double m_turretSpeed = 0;
        // double timestamp = Timer.getFPGATimestamp();

        double turretPosition = ms_data.rolloverHelper(ms_this.getTurretPosition());
        
        double yaw = ms_data.position.getYaw();
        double targetX = ms_data.aim.target[0];
        double targetY = ms_data.aim.target[1];

        double differenceX = ms_data.aim.turretX - targetX;
        double differenceY = ms_data.aim.turretY - targetY;
        double targetAngle = ms_data.rolloverHelper((Math.atan(differenceY/differenceX) * Constants.DEGREES_PER_RADIANS));

        if      ((differenceX < 0) && (differenceY > 0)) targetAngle = targetAngle - 180;
        else if ((differenceX < 0) && (differenceY < 0)) targetAngle = targetAngle + 180;

        targetAngle *= -1;
        
        SmartDashboard.putNumber("target angle", targetAngle);

        m_turretTarget = ms_data.rolloverHelper(yaw + targetAngle);

        SmartDashboard.putNumber("turret target", m_turretTarget);
        SmartDashboard.putNumber("yaw", yaw);
       

        // ms_data.aim.fuelVelocity = ((ms_data.aim.distance)/
        //                     (Math.cos(ms_data.aim.angle) * 
        //                    Math.sqrt((2*(Constants.SHOOTERHEIGHT - ms_data.aim.target[2] + 
        //                                 (ms_data.aim.distance * Math.tan(ms_data.aim.angle))))/
        //                              (Constants.GRAVITY))));
        // SmartDashboard.putNumber("fuel velocity", ms_data.aim.fuelVelocity);


        m_turretPID.m_Error = ms_data.errorHelper(turretPosition, m_turretTarget);
        
        switch (ms_data.Mode)
        {
            case TELEOP:
            {
                if (m_usingJoystick)
                {
                    ms_data.aim.shoot = m_joystick.getRawButton(1);
                    ms_data.aim.stop  = m_joystick.getRawButton(2);
                }
                else
                {
                    ms_data.aim.shoot = m_controller.rightBumper().getAsBoolean();
                    ms_data.aim.stop  = m_controller.leftBumper().getAsBoolean();
                }
                break;
            }
            case AUTO:
            {
                if (ms_data.aim.tracking) ms_data.aim.shoot = true;
                break;
            }
        }

        double currentVelocity = 0;

        switch (ms_data.aim.state)
        {
            case ZERO: 
            {
                if (m_targetVelocity > 0) m_targetVelocity = m_targetVelocity - 100;
                else                      m_targetVelocity = 0;

                break;
            }
            case TRANSITION:
            {
                
                currentVelocity = -ms_this.getVelocity();
                m_targetVelocity = 4000;
                // m_targetVelocity = 0.750 /*(ms_data.aim.fuelVelocity * 57.29578) * COMPENSATION*/;
                // if (distance < 14) m_targetVelocity = 4000;
                // else               m_targetVelocity = 170 * distance + 1600;
                break;
            }
            case FIRE:
            {
                
                currentVelocity = -ms_this.getVelocity();
                m_targetVelocity = 4000;
                // m_targetVelocity = 0.750 /*(ms_data.aim.fuelVelocity * 57.29578) * COMPENSATION*/;
                // if (distance < 14) m_targetVelocity = 4000;
                // else               m_targetVelocity = (170 * distance) + 1600;
                break;
            }
        }
        
        ms_data.aim.tracking = (Math.abs(m_targetVelocity - currentVelocity) < 50) /*&&  (Math.abs(m_turretPID.m_Error) < 5)*/;

        // m_turretSpeed = m_turretPID.CalcPID(timestamp);


        SmartDashboard.putNumber("shooter current", currentVelocity);
        SmartDashboard.putNumber("shooter target", m_targetVelocity);

        ms_this.setShooterVelocity(m_targetVelocity);
        // ms_this.setTurretSpeed(-m_turretSpeed);

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