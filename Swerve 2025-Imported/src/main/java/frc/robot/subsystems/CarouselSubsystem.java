package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CarouselSubsystem extends SubsystemBase
{

    final TalonFX m_motor = new TalonFX(43);
    
    final VelocityVoltage m_velocity = new VelocityVoltage(0);

    private static double IO_RATIO = 0.021818182;


    public CarouselSubsystem()
    {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = 0.0;
        slot0Configs.kP = 0.0;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.0;
        m_motor.getConfigurator().apply(slot0Configs, 0.050);
    }

    public void setSpeed(double speed_rpm)
    {
        double speed_rps = (speed_rpm/60) * IO_RATIO;
        m_velocity.Slot = 0;
        m_motor.setControl (m_velocity.withVelocity(speed_rps));
    }

    public void set(double speed)
    {
        SmartDashboard.putNumber("carousel speed", speed);
        m_motor.set(speed);
    }

}
