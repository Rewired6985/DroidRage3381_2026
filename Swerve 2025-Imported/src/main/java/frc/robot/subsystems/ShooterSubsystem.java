package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase
{

    final TalonFX right_motor = new TalonFX(41);
    final TalonFX left_motor  = new TalonFX(42);
    final SparkMax turret_motor = new SparkMax(41, MotorType.kBrushless);
    final RelativeEncoder turret_encoder;

    public ShooterSubsystem()
    {
        turret_encoder = turret_motor.getEncoder();
    }

    public void setShooterSpeed(double set_speed)
    {
        right_motor.set(-set_speed);
        left_motor .set( set_speed);
    }

    public void setTurretSpeed(double set_speed)
    {
        turret_motor.set(set_speed);
    }

    public double getTurretPosition()
    {
        return turret_encoder.getPosition();
    }


}