package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CarouselSubsystem extends SubsystemBase
{

    final TalonFX carousel_motor = new TalonFX(14);
    final SparkMax gate_motor = new SparkMax(41  , MotorType.kBrushless);
    final RelativeEncoder gate_encoder;

    public CarouselSubsystem()
    {
        gate_encoder = gate_motor.getEncoder();
    }

    public void setCarouselSpeed(double speed)
    {
        carousel_motor.set(speed);
    }

}
