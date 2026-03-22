package frc.robot.commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DataMgmtSubsystem;

public class DataMgmtCommand extends Command
{

    private final DataMgmtSubsystem ms_this;
    private Joystick m_joystick;
    private CommandXboxController m_controller;
    private CommandXboxController m_apac;
    private boolean usingJoystick;

    private int counter = 0;

    public DataMgmtCommand(DataMgmtSubsystem subsystem, CommandXboxController controller)
    {
        ms_this = subsystem;
        m_controller = controller;
        usingJoystick = false;
        addRequirements(subsystem);
    }

    public DataMgmtCommand(DataMgmtSubsystem subsystem, CommandXboxController apac, Joystick joystick)
    {
        ms_this = subsystem;
        m_joystick = joystick;
        m_apac = apac;
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

        ms_this.ShotHandler();


        boolean pabloResultValid = ms_this.position.getPabloValidity();
        boolean baploResultValid = ms_this.position.getBaploValidity();


        switch (ms_this.Mode)
        {
            case TELEOP:
            {
                if (usingJoystick)
                {
                    //if (m_joystick.getRawButton(3)) 
                    ms_this.inputs.intake = m_joystick.getRawButton(3);
                    ms_this.aim.tracking = m_apac.b().getAsBoolean();
                }
                else
                {
                    if (m_controller.x().getAsBoolean()) ms_this.inputs.intake = true;
                    ms_this.aim.tracking = m_controller.povLeft().getAsBoolean();
                }
                break;
            }
            case AUTO:
            {
                // ms_this.inputs.intake = true;
                if (counter == 0)
                {
                    counter++;
                    ms_this.inputs.resetGyro = true;
                }
                ms_this.aim.tracking = (Timer.getMatchTime() < 10);
                SmartDashboard.putNumber("timestamp", Timer.getMatchTime());
                break;
            }
        }

        switch (ms_this.aim.state)
        {
            case ZERO:
            {
                ms_this.inputs.carousel = false;
                break;
            }
            case TRANSITION:
            {
                ms_this.inputs.carousel = false;
                break;
            }
            case FIRE:
            {
                ms_this.inputs.carousel = true;
                break;
            }
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

        SmartDashboard.putString("aim state", ms_this.aim.state.toString());

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
