package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DataMgmtSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command
{

    private IntakeSubsystem ms_this;
    private DataMgmtSubsystem ms_data;
    private double intakeSpeed;
    private double deploySpeed;

    private double targetPosition;
    private double targetRange = 5;

    public IntakeCommand(IntakeSubsystem subsystem, DataMgmtSubsystem data_subsystem)
    {
        ms_this = subsystem;
        ms_data = data_subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute()
    {

        double currentPosition = ms_this.getEncoder();

        switch (ms_data.intake.state)
        {
            case DEPLOY:
            {
                if ((currentPosition > (targetPosition - targetRange)) && 
                    (currentPosition < (targetPosition + targetRange))) 
                {
                    ms_data.intake.inPosition = true;
                }
                break;
            }
            case RUN:
            {
                ms_data.intake.inPosition = false;
                break;
            }
            case STOP:
            {
                ms_data.intake.inPosition = false;
                break;
            }
            case RETRACT:
            {
                break;
            }
        }

        ms_this.setIntakeSpeed(intakeSpeed);
        ms_this.setDeploySpeed(deploySpeed);
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
