package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase
{

    private SparkMax motor = new SparkMax(8, MotorType.kBrushless);

    public IntakeSubsystem()
    {

    }

    public void setPower(double set_power)
    {
        motor.set(set_power);
    }


    
}
