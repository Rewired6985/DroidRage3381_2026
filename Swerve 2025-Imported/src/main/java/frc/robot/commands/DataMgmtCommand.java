package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DataMgmtSubsystem;

public class DataMgmtCommand extends Command
{

    private final DataMgmtSubsystem ms_this;
    private Joystick m_joystick;
    private CommandXboxController m_controller;
    private CommandXboxController m_apac;
    private boolean usingJoystick;

    private BooleanSupplier m_b;
    private BooleanSupplier m_x;

    private int counter = 0;

    // public DataMgmtCommand(DataMgmtSubsystem subsystem, CommandXboxController controller)
    // {
    //     ms_this = subsystem;
    //     m_controller = controller;
    //     usingJoystick = false;
    //     addRequirements(subsystem);
    // }

    public DataMgmtCommand(DataMgmtSubsystem subsystem, CommandXboxController apac, Joystick joystick)
    {
        ms_this = subsystem;
        m_joystick = joystick;
        m_apac = apac;
        m_b = m_apac.b();
        m_x = m_apac.x();
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


        switch (ms_this.Mode)
        {
            case TELEOP:
            {
                if (usingJoystick)
                {
                    //if (m_joystick.getRawButton(3)) 
                    ms_this.inputs.intake = m_joystick.getRawButton(3);
                    ms_this.aim.tracking = m_b.getAsBoolean();
                }
                else
                {
                    if (m_controller.x().getAsBoolean()) ms_this.inputs.intake = true;
                    ms_this.aim.tracking = m_controller.povLeft().getAsBoolean();

                }

                ms_this.resetFlag = m_x.getAsBoolean();

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


        SmartDashboard.putString("aim state", ms_this.aim.state.toString());


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
