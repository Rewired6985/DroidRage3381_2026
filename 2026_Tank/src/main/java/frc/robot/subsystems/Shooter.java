package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
    private TalonFX LeftShooter  = new TalonFX(5);
    private TalonFX RightShooter = new TalonFX(6);
    private TalonFX TopShooter   = new TalonFX(7);

    public void setSpeed(double set_speed)
    {
        LeftShooter  .set(set_speed);
        RightShooter .set(set_speed);
        TopShooter   .set(set_speed);
    }
    
}



