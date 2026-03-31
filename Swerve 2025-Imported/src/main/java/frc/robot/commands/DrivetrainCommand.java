package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
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

    private BooleanSupplier m_inputAim;

    
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

    public DrivetrainCommand(DrivetrainSubsystem subsystem, DataMgmtSubsystem data_subsystem, Joystick joystick, CommandXboxController apac)
    {
        ms_this = subsystem;
        ms_data = data_subsystem;
        m_inputAim = apac.leftBumper();
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
        double drivetrainX = ms_data.position.getEstX();
        double drivetrainY = ms_data.position.getEstY();
        double drivetrainR = ms_data.position.getYaw();
        double velocityX   = ms_data.aim.velocityX;
        double velocityY   = ms_data.aim.velocityY;

        double[] shotTarget = {0, 0, 0};

        ms_data.aim.turretX = drivetrainX + ((Constants.TURRETX * Math.cos(drivetrainR)) - (Constants.TURRETY * Math.sin(drivetrainR)));
        ms_data.aim.turretY = drivetrainY + ((Constants.TURRETX * Math.sin(drivetrainR)) + (Constants.TURRETY * Math.cos(drivetrainR)));

        double driveTargetX = 0;
        double driveTargetY = 0;
        double driveTargetR = 0;

        double systemTime = Timer.getFPGATimestamp();

        // SmartDashboard.putNumber("distance", ms_data.aim.distance);



        boolean allyRed = ms_data.AllianceIsRed;
        

        double driverX = 0;
        double driverY = 0;
        double driverR = 0;
        double driverT = 1;

        if (allyRed)
        {
            if      (drivetrainX > 40.880) 
            {
                shotTarget = Constants.shot_REDHUB;
            }
            else if (drivetrainX > 15.13)
            {
                if (drivetrainY > 13.139) shotTarget = Constants.shot_LEFTRED;
                else                      shotTarget = Constants.shot_RIGHTRED;
            }
            else
            {
                if (drivetrainY > 13.139) shotTarget = Constants.shot_LEFTNEUTRAL; 
                else                      shotTarget = Constants.shot_RIGHTNEUTRAL;
            }

        }
        else
        {
            if (drivetrainX > 39.047)
            {
                if (drivetrainY > 13.139) shotTarget = Constants.shot_LEFTNEUTRAL;
                else                      shotTarget = Constants.shot_RIGHTNEUTRAL;
            }
            else if (drivetrainX > 15.13)
            {
                if (drivetrainY > 13.139) shotTarget = Constants.shot_LEFTBLUE;
                else                      shotTarget = Constants.shot_RIGHTBLUE;
            }
            else if (drivetrainX < 13.297)         
            {
                shotTarget = Constants.shot_BLUEHUB;
            }
        }

        double vectorI   = (ms_data.aim.flightTime * velocityX);
        double vectorJ   = (ms_data.aim.flightTime * velocityY);

        
        double distanceX = (ms_data.aim.turretX - ms_data.aim.target[0]);
        double distanceY = (ms_data.aim.turretY - ms_data.aim.target[1]);

        ms_data.aim.target[0] = shotTarget[0] - vectorI;
        ms_data.aim.target[1] = shotTarget[1] - vectorJ;
        ms_data.aim.target[2] = shotTarget[2];

        switch (ms_data.Mode)
        {

            case TELEOP:
            {
                if (usingJoystick) 
                {

                    int pov = m_joystick.getPOV();
                    double setR = 0;
                    double targetAngle = 999.99;

                    if (m_inputAim.getAsBoolean())
                    {

                        double targetX = ms_data.aim.target[0];
                        double targetY = ms_data.aim.target[1];

                        double differenceX = ms_data.aim.turretX - targetX;
                        double differenceY = ms_data.aim.turretY - targetY;
                        
                        targetAngle = ms_data.rolloverHelper((Math.atan(differenceY/differenceX) * Constants.DEGREES_PER_RADIANS));

                        if      ((differenceX < 0) && (differenceY > 0)) targetAngle = targetAngle - 180;
                        else if ((differenceX < 0) && (differenceY < 0)) targetAngle = targetAngle + 180;

                    }
                    else if (pov == 0)   targetAngle =   0;
                    else if (pov == 90)  targetAngle = -90;
                    else if (pov == 180) targetAngle = 180;
                    else if (pov == 270) targetAngle =  90;

                    if (targetAngle == 999.99) setR = -m_joystick.getZ() * 0.9;
                    else
                    {
                        m_Rpid.m_Error = ms_data.errorHelper(drivetrainR, targetAngle); 
                        setR = m_Rpid.CalcPID(systemTime);
                    }

                    driverX = -m_joystick.getY() * 0.9;
                    driverY = -m_joystick.getX() * 0.9;
                    driverR = setR;


                    driverT = ((-m_joystick.getThrottle() + 1) * 0.4) + 0.2;

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
                        driveTargetX = drivetrainX; 
                        driveTargetY = drivetrainY; 
                        driveTargetR = drivetrainR;
                        ms_data.inRest = true;
                        break;
                    }
                    case TODEPOT:
                    {
                        if (allyRed) { driveTargetX = BYREDDEPOT[0];  driveTargetY = BYREDDEPOT[1];  driveTargetR = BYREDDEPOT[2];}
                        else         { driveTargetX = BYBLUEDEPOT[0]; driveTargetY = BYBLUEDEPOT[1]; driveTargetR = BYBLUEDEPOT[2];}
                        ms_data.inRest = false;
                        break;
                    }
                    case TOCENTER:
                    {
                        if (allyRed) { driveTargetX = REDCENTER[0];  driveTargetY = REDCENTER[1];  driveTargetR = REDCENTER[2];}
                        else         { driveTargetX = BLUECENTER[0]; driveTargetY = BLUECENTER[1]; driveTargetR = BLUECENTER[2];}
                        ms_data.inRest = false;
                        break;
                    }
                    case TOOUTPOST:
                    {
                        if (allyRed) { driveTargetX = REDOUTPOST[0];  driveTargetY = REDOUTPOST[1];  driveTargetR = REDOUTPOST[2];}
                        else         { driveTargetX = BLUEOUTPOST[0]; driveTargetY = BLUEOUTPOST[1]; driveTargetR = BLUEOUTPOST[2];}
                        ms_data.inRest = false;
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

                if ((drivetrainX > (driveTargetX - acceptanceRange) && 
                     drivetrainX < (driveTargetX + acceptanceRange)) && 
                    (drivetrainY > (driveTargetY - acceptanceRange) && 
                     drivetrainY < (driveTargetY + acceptanceRange))) ms_this.inPosition = true;
                else                                                  ms_this.inPosition = false;

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

                // SmartDashboard.putNumber("targetX", driveTargetX);
                // SmartDashboard.putNumber("targetY", driveTargetY);
                // SmartDashboard.putString("state",ms_this.state.toString());

                // SmartDashboard.putBoolean("did outpost", ms_this.outpostCollected);
                // SmartDashboard.putBoolean("did depot",   ms_this.depotCollected);
                // SmartDashboard.putBoolean("did goal",    ms_this.goalAccomplished);


                m_Xpid.m_Error = (driveTargetX - drivetrainX);
                m_Ypid.m_Error = (driveTargetY - drivetrainY);
                m_Rpid.m_Error = ms_data.errorHelper(driveTargetR, drivetrainR);

                driverX = m_Xpid.CalcPID(systemTime);
                driverY = m_Ypid.CalcPID(systemTime);
                driverR = -m_Rpid.CalcPID(systemTime);


                break;
            }
        }

        switch (ms_data.aim.state)
        {
            case ZERO: 
            {
                break;
            }
            case TRANSITION:
            {
                break;
            }
            case FIRE: 
            {
                break;
            }
        }
                                         
        ms_data.aim.distance = Math.sqrt((distanceX) * (distanceX) + 
                                         (distanceY) * (distanceY));

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


}
