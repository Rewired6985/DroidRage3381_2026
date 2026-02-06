package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CarouselSubsystem extends SubsystemBase
{

    public CarouselSubsystem(){}

    final TalonFX motor = new TalonFX(14);

    public void setPower(double power)
    {
        motor.set(power);
    }

}
