package frc.robot.commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DataMgmtSubsystem;

public class DataMgmtCommand extends Command
{

    private final DataMgmtSubsystem ms_this;
    private Joystick m_joystick;
    private boolean usingJoystick;

    public DataMgmtCommand(DataMgmtSubsystem subsystem)
    {
        ms_this = subsystem;
        usingJoystick = false;
        addRequirements(subsystem);
    }

    public DataMgmtCommand(DataMgmtSubsystem subsystem, Joystick joystick)
    {
        ms_this = subsystem;
        m_joystick = joystick;
        usingJoystick = true;
        addRequirements(subsystem);
    }

    @Override 
    public void initialize()
    {
        
    }
   
    @Override
    public void execute()
    {
        //calls state handlers
        ms_this.IntakeHandler();
        ms_this.ShotHandler();

        boolean pabloResultValid = ms_this.position.getPabloValidity();
        boolean baploResultValid = ms_this.position.getBaploValidity();

        switch (ms_this.Mode)
        {
            case TELEOP:
            {
                if (usingJoystick)
                {
                    ms_this.inputs.switchIntake = true;
                }
                break;
            }
            case AUTO:
            {
                break;
            }
        }

        if (ms_this.inSim)
        {
            ms_this.position.updateVisionSim();
            ms_this.position.runPositionSim();
        }


        
        if(pabloResultValid)
        {
            Optional<EstimatedRobotPose>  pablo_est = ms_this.position.pablo_estimator.estimateCoprocMultiTagPose(ms_this.position.getPabloResult());
            if (pablo_est.isEmpty())      pablo_est = ms_this.position.pablo_estimator.estimateLowestAmbiguityPose(ms_this.position.p_result);
            ms_this.position.addMeasurement(pablo_est);
        } 

        if (baploResultValid)
        {
            Optional<EstimatedRobotPose>  baplo_est = ms_this.position.baplo_estimator.estimateCoprocMultiTagPose(ms_this.position.getBaploResult());
            if (baplo_est.isEmpty())      baplo_est = ms_this.position.baplo_estimator.estimateLowestAmbiguityPose(ms_this.position.b_result);
            ms_this.position.addMeasurement(baplo_est);
        }

        ms_this.position.updateEstimator();


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
