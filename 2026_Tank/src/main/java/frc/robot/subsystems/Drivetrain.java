package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

public class Drivetrain extends SubsystemBase
{

    private TalonFX FrontLeft  = new TalonFX(1);
    private TalonFX FrontRight = new TalonFX(2);
    private TalonFX BackLeft   = new TalonFX(3);
    private TalonFX BackRight  = new TalonFX(4);

    public void setLeftDrive(double set_speed)
    {
        FrontLeft .set(set_speed);
        BackLeft  .set(set_speed);
    }

    public void setRightDrive(double set_speed)
    {
        FrontRight .set(set_speed);
        BackRight  .set(set_speed);
    }
}
