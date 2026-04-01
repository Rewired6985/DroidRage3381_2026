package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PIDFController;
import frc.robot.subsystems.DataMgmtSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command
{

    private PIDFController m_pid = new PIDFController(0.1, 0, 0, 0);

    private IntakeSubsystem ms_this;
    private DataMgmtSubsystem ms_data;
    private double intakeSpeed = 0.95;
    private double deploySpeed = 0;

    private double m_speed = 0;

    private double targetPosition;

    private double targetRange = 5;
    private double retractTarget = 0;
    private double deployTarget = 29.2;

    private boolean m_autoMode = true;

    public IntakeCommand(IntakeSubsystem subsystem, DataMgmtSubsystem data_subsystem, double speed)
    {
        ms_this = subsystem;
        ms_data = data_subsystem;
        m_speed = speed;
        m_autoMode = false;
        addRequirements(subsystem);
    }

    public IntakeCommand(IntakeSubsystem subsystem, DataMgmtSubsystem data_subsystem)
    {
        ms_this    = subsystem;
        ms_data    = data_subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize()
    {

        if (m_autoMode)
        {
            if (ms_this.inDeployMode)
            {
                ms_this.inDeployMode = false;
                targetPosition = retractTarget;
                deploySpeed = 0.5;
                intakeSpeed = 0;
            }
            else
            {
                ms_this.inDeployMode = true;
                deploySpeed = -0.5;
                targetPosition = deployTarget;
            }
        }
    }

    @Override
    public void execute()
    {

        double currentTime_s = Timer.getFPGATimestamp();
        double currentPosition = ms_this.getEncoder();
        double m_target = m_pid.CalcFFWDPosition(0, targetPosition, currentTime_s * 1000)[1];

        m_pid.m_Error = (m_target - currentPosition);
        deploySpeed   = m_pid.CalcPID(currentTime_s);


        if ((currentPosition > (targetPosition - targetRange)) && 
            (currentPosition < (targetPosition + targetRange))) 
        {
            if (ms_this.inDeployMode) intakeSpeed = 0.95;
            ms_data.inputs.intake = false;
        }

        if (m_autoMode)
        {
            ms_this.setIntakeSpeed(intakeSpeed);
            ms_this.setDeploySpeed(deploySpeed);
        }
        else ms_this.setIntakeSpeed(m_speed);

    }

    @Override
    public void end(boolean interrupted)
    {
        ms_this.setIntakeSpeed(0);
        ms_this.setDeploySpeed(0);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
    
}
