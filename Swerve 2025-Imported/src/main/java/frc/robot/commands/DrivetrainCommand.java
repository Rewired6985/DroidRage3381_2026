package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

        ms_data.aim.updateVelocity();
        double positionX = ms_data.position.getEstX();
        double positionY = ms_data.position.getEstY();
        // double heading   = ms_data.position.getHeading().getValueAsDouble();
        double velocityX = ms_data.aim.velocityX;
        double velocityY = ms_data.aim.velocityY;

        double distanceX = (positionX - ms_data.aim.target[0]);
        double distanceY = (positionY - ms_data.aim.target[1]);
        double vectorI   = (ms_data.aim.flightTime * velocityX);
        double vectorJ   = (ms_data.aim.flightTime * velocityY);

        
        SmartDashboard.putNumber("distanceX", distanceX);
        SmartDashboard.putNumber("distanceY", distanceY);

        SmartDashboard.putBoolean("hasTarget?", ms_data.aim.hasTarget);
        SmartDashboard.putNumber("targetX", ms_data.aim.target[0]);
        SmartDashboard.putNumber("targetY", ms_data.aim.target[1]);
        SmartDashboard.putNumber("targetZ", ms_data.aim.target[2]);

        SmartDashboard.putNumber("distance", ms_data.aim.distance);



        boolean allyRed = ms_data.AllianceIsRed;
        boolean inAllianceZone = false;
        

        double inputX = 0;
        double inputY = 0;
        double inputR = 0;
        double inputT = 0;

        if (allyRed)
        {
            if      (positionX > 40.880) 
            {
                ms_data.aim.target = ms_data.aim.REDHUB;
                inAllianceZone = true;
            }
            else if (positionX > 15.13)
            {
                if (positionY > 13.139) ms_data.aim.target = ms_data.aim.LEFTRED;
                else                    ms_data.aim.target = ms_data.aim.RIGHTRED;
            }
            else
            {
                if (positionY > 13.139) ms_data.aim.target = ms_data.aim.LEFTNEUTRAL; 
                else                    ms_data.aim.target = ms_data.aim.RIGHTNEUTRAL;
            }
        }
        else
        {
            if (positionX > 39.047)
            {
                if (positionY > 13.139) ms_data.aim.target = ms_data.aim.LEFTNEUTRAL;
                else                    ms_data.aim.target = ms_data.aim.RIGHTNEUTRAL;
            }
            else if (positionX > 15.13)
            {
                if (positionY > 13.139) ms_data.aim.target = ms_data.aim.LEFTBLUE;
                else                    ms_data.aim.target = ms_data.aim.RIGHTBLUE;
            }
            else if (positionX < 13.297)         
            {
                ms_data.aim.target = ms_data.aim.BLUEHUB;
                inAllianceZone = true;
            }
        }

        if ((inAllianceZone == false) && 
            (((positionX < 16.963) && (positionX > 13.297)) || 
             ((positionX < 40.880) && (positionX > 37.214)) || 
             ((positionY < 15.627) && (positionY > 10.759))))
        {
            ms_data.aim.hasTarget = false;
        }
        else
        {
            ms_data.aim.hasTarget = true;
        }



        switch (ms_data.Mode)
        {

            case TELEOP:
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

                    ms_data.inputs.brake = m_joystick.getRawButton(3);
                    ms_data.inputs.resetGyro = m_joystick.getRawButton(5);

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
                                         
        ms_data.aim.distance = Math.sqrt((distanceX - vectorI) * (distanceX - vectorI) + 
                                         (distanceY - vectorJ) * (distanceY - vectorJ));

        if (ms_data.aim.fuelVelocity == 0) ms_data.aim.flightTime = 0;
        else  ms_data.aim.flightTime = (ms_data.aim.distance)/(ms_data.aim.fuelVelocity * Math.cos(ms_data.aim.angle));

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
