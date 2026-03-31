package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase
{

    private TalonFX in_motor    = new TalonFX(11);
    private TalonFX out_motor   = new TalonFX(10);
    private TalonFX left_motor  = new TalonFX(16);
    private TalonFX right_motor = new TalonFX(15);
    public boolean inDeployMode = false;

    public IntakeSubsystem()
    {

    }

    public void setIntakeSpeed(double set_speed)
    {
        in_motor.set(set_speed);
        out_motor.set(set_speed);
    }

    public void setDeploySpeed(double set_speed)
    {
        // right_motor.set(set_speed);
        // left_motor.set(-set_speed);
    }

    public double getEncoder()
    {
        double value = right_motor.getRotorPosition().getValueAsDouble();
        return value;
    }

    
}
