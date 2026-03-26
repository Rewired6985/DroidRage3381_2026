package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

        

    final TalonFX right_motor = new TalonFX(12);
    final TalonFX left_motor  = new TalonFX(13);
    final SparkMax turret_motor = new SparkMax(41, MotorType.kBrushless);
    final RelativeEncoder turret_encoder;

    public ShooterSubsystem()
    {
        right_motor.getConfigurator().apply(rightConfig);
        left_motor.getConfigurator().apply(leftConfig);
        turret_encoder = turret_motor.getEncoder();
        turret_encoder.setPosition(0);
    }

    public void setShooterSpeed(double set_speed)
    {
        right_motor.set(-set_speed);
        left_motor .set(-set_speed);
    }

    public void zeroEncoder()
    {
        turret_encoder.setPosition(0);
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
        double value = turret_encoder.getPosition() * -2.66666667; //* 0.06857142857;
        return value;
    }


}