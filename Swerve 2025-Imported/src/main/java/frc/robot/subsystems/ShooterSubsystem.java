package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase
{

    private final TalonFXConfiguration rightConfig = new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(50)
        .withStatorCurrentLimit(80)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimitEnable(true));

    private final TalonFXConfiguration leftConfig = new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(50)
        .withStatorCurrentLimit(80)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimitEnable(true));

        

    final VelocityVoltage m_velocity = new VelocityVoltage(0);

    final TalonFX right_motor = new TalonFX(12);
    final TalonFX left_motor  = new TalonFX(13);
    final SparkMax turret_motor = new SparkMax(41, MotorType.kBrushless);
    final RelativeEncoder turret_encoder;


    public ShooterSubsystem()
    {
        m_velocity.Slot = 0;

        rightConfig.Slot0.kS = 0.1;
        rightConfig.Slot0.kV = 0.11;
        rightConfig.Slot0.kP = 0.11;
        rightConfig.Slot0.kI = 0.01;
        rightConfig.Slot0.kD = 0;

        leftConfig.Slot0.kS = 0.1;
        leftConfig.Slot0.kV = 0.11;
        leftConfig.Slot0.kP = 0.11;
        leftConfig.Slot0.kI = 0.01;
        leftConfig.Slot0.kD = 0;

        right_motor.getConfigurator().apply(rightConfig);
        left_motor.getConfigurator().apply(leftConfig);
        turret_encoder = turret_motor.getEncoder();
        turret_encoder.setPosition(180);
    }

    public void setShooterSpeed(double set_speed)
    {
        right_motor.set(-set_speed);
        left_motor .set(-set_speed);
    }

    public void setShooterVelocity(double speed_rpm)
    {
        double speed_rps = -(speed_rpm/60);
        right_motor.setControl(m_velocity.withVelocity(speed_rps));
        left_motor.setControl( m_velocity.withVelocity(speed_rps));
    }

    public void zeroEncoder()
    {
        turret_encoder.setPosition(180);
    }

    public double getVelocity()
    { 
        return right_motor.getVelocity().getValue().in(Units.Rotations.per(Units.Minute));
    }

    public void setTurretSpeed(double set_speed)
    {
        SmartDashboard.putNumber("turret power", set_speed);
        turret_motor.set(-set_speed);
    }

    public double getTurretPosition()
    {
        double value = turret_encoder.getPosition() * 2.65;
        return value;
    }


}