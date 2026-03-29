package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CarouselSubsystem extends SubsystemBase
{

    final TalonFX m_motor = new TalonFX(14);
    
    final VelocityVoltage m_velocity = new VelocityVoltage(0);

    private static double IO_RATIO = 45.833333;

    private final TalonFXConfiguration config = new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(50)
        .withStatorCurrentLimit(80)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimitEnable(true));


    public CarouselSubsystem()
    {
        m_velocity.Slot = 0;

        config.Slot0.kS = 0.1;
        config.Slot0.kV = 0.12;
        config.Slot0.kP = 0.11;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        m_motor.getConfigurator().apply(config);
    }

    public void setSpeed(double speed_rpm)
    {
        double speed_rps = (speed_rpm/60) * IO_RATIO;
        SmartDashboard.putNumber("carousel speed", speed_rpm);
        m_velocity.Slot = 0;
        m_motor.setControl (m_velocity.withVelocity(speed_rps));
    }

    public double getVelocity()
    { 
        return m_motor.getVelocity().getValue().in(Units.Rotations.per(Units.Minute))/IO_RATIO;
    }

    public void set(double speed)
    {
        m_motor.set(speed);
    }

}
