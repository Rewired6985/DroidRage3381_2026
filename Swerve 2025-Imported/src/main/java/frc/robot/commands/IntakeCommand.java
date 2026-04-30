package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.eIntakeMode;
import frc.robot.PIDFController;
import frc.robot.subsystems.DataMgmtSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command
{

    private PIDFController m_pid = new PIDFController(0.04, 0, 0, 0);

    private eIntakeMode m_mode;
    

    private static double TARGET_RANGE = 2;
    private static double RETRACT_TARGET = 0;
    private static double DEPLOY_TARGET = 29.2;
    
    private static double[] INTAKESPEED   = { 0.75,  0};
    private static double[] OUTTAKESPEED  = {-0.95, -0.6};
    private static double[] ZERO          = { 0.00,  0.0};
    private static double   DEPLOYSPEED  =  0.05;
    private static double   RETRACTSPEED = -0.05;
     

    private IntakeSubsystem ms_this;
    private DataMgmtSubsystem ms_data;

    private double[] m_intakeSpeed = INTAKESPEED;
    private double   m_deploySpeed   = 0;

    private double[] m_manualSpeed  = {0,0};

    private double m_targetPosition;

    public IntakeCommand(IntakeSubsystem subsystem, DataMgmtSubsystem data_subsystem, eIntakeMode mode)
    {

        m_mode = mode;

        switch (mode)
        {
            case AUTOMATIC:
            {
                m_pid.m_LimitMax = 0.7;
                m_pid.m_LimitMin = -1;
                break;
            }
            case INTAKE:  { m_manualSpeed    = INTAKESPEED;  break; }
            case OUTTAKE: { m_manualSpeed    = OUTTAKESPEED; break; }
            case RETRACT: { m_manualSpeed[0] = RETRACTSPEED; break; }
            case DEPLOY:  { m_manualSpeed[0] = DEPLOYSPEED;  break; }
        }

        ms_this    = subsystem;
        ms_data    = data_subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize()
    {

        if (m_mode == eIntakeMode.AUTOMATIC)
        {
            if (ms_this.inDeployMode)
            {
                ms_this.inDeployMode = false;
                m_targetPosition = RETRACT_TARGET;
                ms_this.setIntakeSpeed(ZERO);
                
            }
            else
            {
                ms_this.inDeployMode = true;
                m_targetPosition = DEPLOY_TARGET;
                
            }
            
        }
    }

    @Override
    public void execute()
    {
      
        double currentPosition = ms_this.getEncoder();
    
        double currentTime_s = Timer.getFPGATimestamp();
        // double m_target = m_pid.CalcFFWDPosition(0, m_targetPosition, currentTime_s * 1000)[1];

        m_pid.m_Error = (m_targetPosition - currentPosition);
        m_deploySpeed   = m_pid.CalcPID(currentTime_s);

        if ((currentPosition > (m_targetPosition - TARGET_RANGE)) && 
            (currentPosition < (m_targetPosition + TARGET_RANGE))) 
        {
            ms_data.inputs.intake = false;
        }

        switch (m_mode)
        {
            case AUTOMATIC: 
            { 
                ms_this.setDeploySpeed(m_deploySpeed); 

                if (currentPosition > 7) ms_this.setIntakeSpeed(INTAKESPEED);
                else                     ms_this.setIntakeSpeed(ZERO);

                break; 
            }
            case DEPLOY:
            case RETRACT:   
            { 
                ms_this.setDeploySpeed(m_manualSpeed[0]); 
                break; 
            }
            case INTAKE:
            case OUTTAKE:   
            {
                ms_this.setIntakeSpeed(m_manualSpeed);
                break; 
            }
        }

    }

    @Override
    public void end(boolean interrupted)
    {
        if (m_mode == eIntakeMode.INTAKE || m_mode == eIntakeMode.OUTTAKE) ms_this.setIntakeSpeed(ZERO);
        ms_this.setDeploySpeed(0);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
    
}
