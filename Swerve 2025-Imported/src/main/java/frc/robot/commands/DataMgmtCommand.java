package frc.robot.commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DataMgmtSubsystem;

public class DataMgmtCommand extends Command
{

    private final DataMgmtSubsystem ms_this;
    private Joystick m_joystick;
    private CommandXboxController m_controller;
    private boolean usingJoystick;

    public DataMgmtCommand(DataMgmtSubsystem subsystem, CommandXboxController controller)
    {
        ms_this = subsystem;
        m_controller = controller;
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

        ms_this.ShotHandler();


        boolean pabloResultValid = ms_this.position.getPabloValidity();
        boolean baploResultValid = ms_this.position.getBaploValidity();


        switch (ms_this.Mode)
        {
            case TELEOP:
            {
                if (usingJoystick)
                {
                    if (m_joystick.getRawButton(3)) ms_this.inputs.callIntake = true;
                }
                else
                {
                    if (m_controller.x().getAsBoolean()) ms_this.inputs.callIntake = true;
                }
                break;
            }
            case AUTO:
            {
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
