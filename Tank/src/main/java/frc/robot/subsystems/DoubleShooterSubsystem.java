package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DoubleShooterSubsystem extends SubsystemBase
{
    public DoubleShooterSubsystem(){}

    final SparkMax left_motor  = new SparkMax(6, MotorType.kBrushless);
    final SparkMax right_motor = new SparkMax(7, MotorType.kBrushless);

    public void setPower(double set_power)
    {
        right_motor.set(set_power);
        left_motor .set(set_power);
    }

}