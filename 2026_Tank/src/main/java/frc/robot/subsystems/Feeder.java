package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase{

    private TalonFX Feeder1  = new TalonFX(8);
    private TalonFX Feeder2 = new TalonFX(9);

    public void setFeeder(double set_speed)
    {
        Feeder1  .set(set_speed);
        Feeder2  .set(set_speed);
    }


}
