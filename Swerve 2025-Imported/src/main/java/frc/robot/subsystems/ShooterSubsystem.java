package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase
{

    final TalonFX right_motor = new TalonFX(8);
    final TalonFX left_motor  = new TalonFX(9);
    final Servo hood_servo = new Servo(3);
    final SparkMax turret_motor = new SparkMax(4, MotorType.kBrushless);
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

    public void setAngle(double set_angle)
    {
        hood_servo.setAngle(set_angle);
    }

    public double getTurretPosition()
    {
        return turret_encoder.getPosition();
    }


}