package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase
{

    private TalonFX in_motor    = new TalonFX(11);
    private TalonFX out_motor   = new TalonFX(10);
    private TalonFX left_motor  = new TalonFX(15);
    private TalonFX right_motor = new TalonFX(16);
   
    public boolean inDeployMode = false;

    public double[] storedIntake = {0,0};
    public double storedPivot = 0;

    private double offset = 0;

            private final TalonFXConfiguration inConfig = new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(50)
    .withStatorCurrentLimit(80)
    .withSupplyCurrentLimitEnable(true)
    .withStatorCurrentLimitEnable(true));

            private final TalonFXConfiguration outConfig = new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(50)
    .withStatorCurrentLimit(80)
    .withSupplyCurrentLimitEnable(true)
    .withStatorCurrentLimitEnable(true));

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

    public IntakeSubsystem()
    {
        
        leftConfig.MotorOutput.NeutralMode  = NeutralModeValue.Brake;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        in_motor.getConfigurator().apply(inConfig);
        out_motor.getConfigurator().apply(outConfig);
        left_motor.getConfigurator().apply(leftConfig);
        right_motor.getConfigurator().apply(rightConfig);

        offset = getEncoder();
    }

    
    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("intake position", getEncoder());
    }


    public void setIntakeSpeed(double[] set_speed)
    {
         in_motor .set(-set_speed[1]);
         out_motor.set(-set_speed[0]);
         
    }

    public void setDeploySpeed(double set_speed)
    {
        right_motor.set(set_speed);
        left_motor .set(-set_speed);
    }

    public double getEncoder()
    {
        double value = right_motor.getRotorPosition().getValueAsDouble() - offset;
        return value;
    }

    public void updateOffset()
    {
        offset = getEncoder();
    }

    
}
