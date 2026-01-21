package frc.robot.commands;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DataMgmtSubsystem;

public class DrivetrainCommand extends Command 
{
    private final DrivetrainSubsystem ms_this;
    private final DataMgmtSubsystem ms_data;
    private final CommandXboxController m_gamepad;
    
    private double LeftPower;
    private double RightPower;

    private double gamepadX;
    private double gamepadY;

    private PhotonTrackedTarget p_target;
    private PhotonTrackedTarget b_target;



    public DrivetrainCommand(DrivetrainSubsystem subsystem, DataMgmtSubsystem data_subsystem, CommandXboxController gamepad)
    {
        ms_this  = subsystem;
        ms_data = data_subsystem;
        m_gamepad    = gamepad;
        addRequirements(subsystem);
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {

        boolean pabloResultValid = ms_data.getPabloValidity();
        boolean baploResultValid = ms_data.getBaploValidity();

        if (pabloResultValid) p_target = ms_data.getPabloTarget();
        if (baploResultValid) b_target = ms_data.getBaploTarget();

        SmartDashboard.putBoolean("PabloValid?", pabloResultValid);
        SmartDashboard.putBoolean("BaploValid?", baploResultValid);

        if (pabloResultValid) 
        {
            SmartDashboard.putNumber("PabloID", p_target.fiducialId);
            SmartDashboard.putNumber("PabloYaw", p_target.yaw);
        }
        if (baploResultValid)
        {
            SmartDashboard.putNumber("BaploID", b_target.fiducialId);
            SmartDashboard.putNumber("BaploYaw", b_target.yaw);
        }

        gamepadX = m_gamepad.getLeftX() * 0.95;
        gamepadY = m_gamepad.getLeftY() * 0.95;

        LeftPower = gamepadY + gamepadX;
        RightPower= gamepadY - gamepadX;

        if      (LeftPower >  1)  LeftPower  =  1;
        else if (LeftPower < -1)  LeftPower  = -1;

        if      (RightPower >  1) RightPower =  1;
        else if (RightPower < -1) RightPower = -1;

        ms_this.setPower(LeftPower, RightPower);
    }

    @Override
    public void end(boolean interrupted)
    {
        ms_this.setPower(0,0);
    }
    
    @Override
    public boolean isFinished()
    {
        return false;
    }

}
