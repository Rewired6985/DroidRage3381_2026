package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainCommand extends Command
{
    private Drivetrain ms_this;
    private CommandXboxController m_controller;

    public DrivetrainCommand(Drivetrain subsystem, CommandXboxController controller)
    {
        ms_this = subsystem;
        m_controller = controller;
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {

        double rightX = m_controller.getRightX();
        double leftY  = m_controller.getLeftY();

        double rightHalf  = leftY - rightX * 0.5;
        double leftHalf   = leftY + rightX * 0.5;

        if      (rightHalf >   0.9)  rightHalf =  0.9;
        else if (rightHalf <  -0.9) rightHalf  = -0.9;
        
        if      (leftHalf  >   0.9)  leftHalf  =  0.9;     
        else if (leftHalf  <  -0.9) leftHalf   = -0.9;
     
        ms_this.setRightDrive(rightHalf);
        ms_this.setLeftDrive(leftHalf);
    }

}
