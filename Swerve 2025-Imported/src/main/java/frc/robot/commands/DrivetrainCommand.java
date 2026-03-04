package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.PIDFController;
import frc.robot.subsystems.DataMgmtSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainCommand extends Command
{

    private final DrivetrainSubsystem ms_this;
    private final DataMgmtSubsystem   ms_data;

    
    private Joystick m_joystick;
    private CommandXboxController m_controller;
    private boolean usingJoystick;

    
    private static double[] BYBLUEDEPOT   = {3.542, 20.812, 180};
    private static double[] BLUEDEPOT     = {1.292, 20.812, 180};
    private static double[] BLUEOUTPOST   = {1.292,  3.427, 0};
    private static double[] BLUECENTER    = {6.609, 13.237, 0};

    private static double[] BYREDDEPOT   = {52.018,  6.953,  180};
    private static double[] REDDEPOT     = {54.268,  6.953,  180};
    private static double[] REDOUTPOST   = {54.268, 24.2892, 0};
    private static double[] REDCENTER    = {47.657, 13.238,  0};

    private static double acceptanceRange = 0.5;

    private PIDFController m_Xpid = new PIDFController(0.1,0,0,0);
    private PIDFController m_Ypid = new PIDFController(0.1,0,0,0);
    private PIDFController m_Rpid = new PIDFController(0.1,0,0,0);

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
        // double time = Timer.getFPGATimestamp();
        // m_Xpid.m_PIDlasttime_s = time;
        // m_Ypid.m_PIDlasttime_s = time;
        // m_Rpid.m_PIDlasttime_s = time;
    }

    @Override
    public void execute()
    {

        ms_data.aim.updateVelocity();
        double positionX = ms_data.position.getEstX();
        double positionY = ms_data.position.getEstY();
        double positionR   = ms_data.position.getHeading().getValueAsDouble();
        double velocityX = ms_data.aim.velocityX;
        double velocityY = ms_data.aim.velocityY;

        double distanceX = (positionX - ms_data.aim.target[0]);
        double distanceY = (positionY - ms_data.aim.target[1]);
        double vectorI   = (ms_data.aim.flightTime * velocityX);
        double vectorJ   = (ms_data.aim.flightTime * velocityY);

        double driveTargetX = 0;
        double driveTargetY = 0;
        double driveTargetR = 0;

        double systemTime = Timer.getFPGATimestamp();

        
        // SmartDashboard.putNumber("distanceX", distanceX);
        // SmartDashboard.putNumber("distanceY", distanceY);

        // SmartDashboard.putBoolean("hasTarget?", ms_data.aim.hasTarget);
        // SmartDashboard.putNumber("targetX", ms_data.aim.target[0]);
        // SmartDashboard.putNumber("targetY", ms_data.aim.target[1]);
        // SmartDashboard.putNumber("targetZ", ms_data.aim.target[2]);

        // SmartDashboard.putNumber("distance", ms_data.aim.distance);


        boolean allyRed = ms_data.AllianceIsRed;
        boolean inAllianceZone = false;
        

        double driverX = 0;
        double driverY = 0;
        double driverR = 0;
        double driverT = 1;

        if (allyRed)
        {
            if      (positionX > 40.880) 
            {
                ms_data.aim.target = Constants.shot_REDHUB;
                inAllianceZone = true;
            }
            else if (positionX > 15.13)
            {
                if (positionY > 13.139) ms_data.aim.target = Constants.shot_LEFTRED;
                else                    ms_data.aim.target = Constants.shot_RIGHTRED;
            }
            else
            {
                if (positionY > 13.139) ms_data.aim.target = Constants.shot_LEFTNEUTRAL; 
                else                    ms_data.aim.target = Constants.shot_RIGHTNEUTRAL;
            }
        }
        else
        {
            if (positionX > 39.047)
            {
                if (positionY > 13.139) ms_data.aim.target = Constants.shot_LEFTNEUTRAL;
                else                    ms_data.aim.target = Constants.shot_RIGHTNEUTRAL;
            }
            else if (positionX > 15.13)
            {
                if (positionY > 13.139) ms_data.aim.target = Constants.shot_LEFTBLUE;
                else                    ms_data.aim.target = Constants.shot_RIGHTBLUE;
            }
            else if (positionX < 13.297)         
            {
                ms_data.aim.target = Constants.shot_BLUEHUB;
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
                    driverX = -m_joystick.getY() * 0.9;
                    driverY = -m_joystick.getX() * 0.9;
                    driverR = -m_joystick.getZ() * 0.9;
                    driverT = ((-m_joystick.getThrottle() + 1) * 0.4) + 0.2;

                    ms_data.inputs.brake = m_joystick.getRawButton(4);
                    ms_data.inputs.resetGyro = m_joystick.getRawButton(13);

                }
                else
                {
                    driverX = -m_controller.getRightY() * 0.7;
                    driverY = -m_controller.getRightX() * 0.7;
                    driverR = -m_controller.getLeftX()  * 0.7;
                }

                break;
            }
            case AUTO:
            {
                ms_this.autoHandler();

                switch (ms_this.state)
                {
                    case REST: 
                    {
                        driveTargetX = positionX; 
                        driveTargetY = positionY; 
                        driveTargetR = positionR;
                        break;
                    }
                    case TODEPOT:
                    {
                        if (allyRed) { driveTargetX = BYREDDEPOT[0];  driveTargetY = BYREDDEPOT[1];  driveTargetR = BYREDDEPOT[2];}
                        else         { driveTargetX = BYBLUEDEPOT[0]; driveTargetY = BYBLUEDEPOT[1]; driveTargetR = BYBLUEDEPOT[2];}
                        break;
                    }
                    case TOCENTER:
                    {
                        if (allyRed) { driveTargetX = REDCENTER[0];  driveTargetY = REDCENTER[1];  driveTargetR = REDCENTER[2];}
                        else         { driveTargetX = BLUECENTER[0]; driveTargetY = BLUECENTER[1]; driveTargetR = BLUECENTER[2];}
                        break;
                    }
                    case TOOUTPOST:
                    {
                        if (allyRed) { driveTargetX = REDOUTPOST[0];  driveTargetY = REDOUTPOST[1];  driveTargetR = REDOUTPOST[2];}
                        else         { driveTargetX = BLUEOUTPOST[0]; driveTargetY = BLUEOUTPOST[1]; driveTargetR = BLUEOUTPOST[2];}
                        break;
                    }
                    case COLLECTDEPOT:
                    {
                        if (allyRed) { driveTargetX = REDDEPOT[0];  driveTargetY = REDDEPOT[1];  driveTargetR = REDDEPOT[2];}
                        else         { driveTargetX = BLUEDEPOT[0]; driveTargetY = BLUEDEPOT[1]; driveTargetR = BLUEDEPOT[2];}
                        break;
                    }
                    case COLLECTOUTPOST:
                    {
                        if (allyRed) { driveTargetX = REDOUTPOST[0];  driveTargetY = REDOUTPOST[1];  driveTargetR = REDOUTPOST[2];}
                        else         { driveTargetX = BLUEOUTPOST[0]; driveTargetY = BLUEOUTPOST[1]; driveTargetR = BLUEOUTPOST[2];}
                        break;
                    }
                }

                if ((positionX > (driveTargetX - acceptanceRange) && 
                     positionX < (driveTargetX + acceptanceRange)) && 
                    (positionY > (driveTargetY - acceptanceRange) && 
                     positionY < (driveTargetY + acceptanceRange))) ms_this.inPosition = true;
                else                                                ms_this.inPosition = false;

                if (ms_this.inPosition)
                {

                    
                    if (ms_this.state == Constants.eAuto.COLLECTDEPOT) ms_this.depotCollected = true;

                    if (ms_this.state == Constants.eAuto.COLLECTOUTPOST) ms_this.outpostCollected = true;

                    switch (ms_this.goal)
                    {
                        case DEPOT:   
                        {
                            if (ms_this.depotCollected)   ms_this.goalAccomplished = true; break;
                        }
                        case OUTPOST: 
                        {
                            if (ms_this.outpostCollected) ms_this.goalAccomplished = true; break;
                        }
                        case DEPOT_THEN_OUTPOST: 
                        {
                            if (ms_this.outpostCollected && ms_this.depotCollected) ms_this.goalAccomplished = true; 
                            break;
                        }
                        case OUTPOST_THEN_DEPOT:
                        {
                            if (ms_this.outpostCollected && ms_this.depotCollected) ms_this.goalAccomplished = true; 
                            break;
                        }
                        case NOMOVE: ms_this.goalAccomplished = true; break;
                    }
                }

                SmartDashboard.putNumber("targetX", driveTargetX);
                SmartDashboard.putNumber("targetY", driveTargetY);
                SmartDashboard.putString("state",ms_this.state.toString());

                SmartDashboard.putBoolean("did outpost", ms_this.outpostCollected);
                SmartDashboard.putBoolean("did depot",   ms_this.depotCollected);
                SmartDashboard.putBoolean("did goal",    ms_this.goalAccomplished);


                m_Xpid.m_Error = (driveTargetX - positionX);
                m_Ypid.m_Error = (driveTargetY - positionY);
                m_Rpid.m_Error = (driveTargetR - positionR);

                driverX = m_Xpid.CalcPID(systemTime);
                driverY = m_Ypid.CalcPID(systemTime);
                driverR = m_Rpid.CalcPID(systemTime);


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

        
        ms_this.m_Xswerve = driverX  * driverT;
        ms_this.m_Yswerve = driverY  * driverT;
        ms_this.m_Rswerve = driverR  * driverT;

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

    private double addDeadZone(double driver, double limit)
    {
        double output;

        if      (driver < -limit) output = (driver + limit)/(1-limit);
        else if (driver >  limit) output = (driver - limit)/(1-limit);
        else output = 0;

        return output;
    }


}
