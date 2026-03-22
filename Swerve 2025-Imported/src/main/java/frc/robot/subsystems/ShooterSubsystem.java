package frc.robot.subsystems;

import edu.wpi.first.units.Units;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase
{

    final TalonFX right_motor = new TalonFX(12);
    final TalonFX left_motor  = new TalonFX(13);
    final SparkMax turret_motor = new SparkMax(41, MotorType.kBrushless);
    final RelativeEncoder turret_encoder;

    public ShooterSubsystem()
    {
        turret_encoder = turret_motor.getEncoder();
    }

    public void setShooterSpeed(double set_speed)
    {
        right_motor.set(set_speed);
        left_motor .set(set_speed);
    }

    public double getVelocity()
    { 
        return right_motor.getVelocity().getValue().in(Units.Rotations.per(Units.Minute));
    }

    public void setTurretSpeed(double set_speed)
    {
        turret_motor.set(set_speed);
    }

    public double getTurretPosition()
    {
        double value = (turret_encoder.getPosition() * 0.06857142857) - 180;
        return value;
    }


}