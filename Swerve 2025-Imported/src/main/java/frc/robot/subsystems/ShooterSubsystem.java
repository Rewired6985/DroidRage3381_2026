package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase
{
    public ShooterSubsystem(){}

    final TalonFX right_motor = new TalonFX(8);
    final TalonFX left_motor  = new TalonFX(9);

    public void setPower(double set_power)
    {
        right_motor.set(-set_power);
        left_motor .set(set_power);
    }

}