package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CarouselSubsystem extends SubsystemBase
{

    final TalonFX carousel_motor = new TalonFX(14);
    final TalonFX gate_motor = new TalonFX(30);

    public CarouselSubsystem()
    {
        
    }

    public void setCarouselSpeed(double speed)
    {
        carousel_motor.set(speed);
    }

    public void setGateSpeed(double speed)
    {
        gate_motor.set(speed);
    }

    public double getGatePosition()
    {
        return gate_motor.getPosition().getValueAsDouble();
    }

}
