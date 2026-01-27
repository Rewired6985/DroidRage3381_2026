package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase
{

    public DrivetrainSubsystem(){}

    final CANBus driveBus = new CANBus("drivetrain");

    final TalonFX LeftMotor1  = new TalonFX(1, driveBus);
    final TalonFX LeftMotor2  = new TalonFX(2, driveBus);
    final TalonFX RightMotor1 = new TalonFX(3, driveBus);
    final TalonFX RightMotor2 = new TalonFX(4, driveBus);

    public void setPower(double leftPower, double rightPower)
    {
        LeftMotor1 .set(leftPower * -1);
        LeftMotor2 .set(leftPower * -1);
        RightMotor1.set(rightPower);
        RightMotor2.set(rightPower);
    }



}
