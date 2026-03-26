package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CarouselSubsystem extends SubsystemBase
{

    final TalonFX m_motor = new TalonFX(14);
    
    final VelocityVoltage m_velocity = new VelocityVoltage(0);

    private static double IO_RATIO = 0.021818182;

    private final TalonFXConfiguration config = new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(50)
        .withStatorCurrentLimit(80)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimitEnable(true));


    public CarouselSubsystem()
    {
        // var slot0Configs = new Slot0Configs();
        // slot0Configs.kV = 0.0;
        // slot0Configs.kP = 0.0;
        // slot0Configs.kI = 0.0;
        // slot0Configs.kD = 0.0;
        // m_motor.getConfigurator().apply(slot0Configs, 0.050);
        m_motor.getConfigurator().apply(config);
    }

    public void setSpeed(double speed_rpm)
    {
        double speed_rps = (speed_rpm/60) * IO_RATIO;
        m_velocity.Slot = 0;
        m_motor.setControl (m_velocity.withVelocity(speed_rps));
    }

    public void set(double speed)
    {
        m_motor.set(speed);
    }

}
