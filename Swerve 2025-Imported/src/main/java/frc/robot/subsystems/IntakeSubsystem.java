package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase
{

    private TalonFX in_motor  = new TalonFX(30);
    private TalonFX out_motor = new TalonFX(31);
    public boolean inDeployMode = false;

    public IntakeSubsystem()
    {

    }

    public void setIntakeSpeed(double set_speed)
    {
        in_motor.set(set_speed);
        out_motor.set(set_speed);
    }

    public void setDeploySpeed(double set_speed)
    {
        
    }

    public double getEncoder()
    {
        return 0;
    }

    
}
