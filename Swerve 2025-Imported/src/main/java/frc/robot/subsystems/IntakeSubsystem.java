package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase
{

    private SparkMax motor = new SparkMax(8, MotorType.kBrushless);
    public boolean inDeployMode = false;

    public IntakeSubsystem()
    {

    }

    public void setIntakeSpeed(double set_speed)
    {
        motor.set(set_speed);
    }

    public void setDeploySpeed(double set_speed)
    {
        
    }

    public double getEncoder()
    {
        return 0.4;
    }

    
}
