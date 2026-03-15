package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CarouselSubsystem;
import frc.robot.subsystems.DataMgmtSubsystem;

public class CarouselCommand extends Command
{

    private final CarouselSubsystem ms_this;
    private final DataMgmtSubsystem ms_data;
    private CommandXboxController m_controller;
    private Joystick m_joystick;
    private boolean usingJoystick;
    
    public CarouselCommand(CarouselSubsystem subsystem, DataMgmtSubsystem state_subsystem, CommandXboxController controller)
    {
        ms_this       = subsystem;
        ms_data       = state_subsystem;
        m_controller  = controller;
        usingJoystick = false;
        addRequirements(subsystem);
    }
        
    public CarouselCommand(CarouselSubsystem subsystem, DataMgmtSubsystem state_subsystem, Joystick joystick)
    {
        ms_this       = subsystem;
        ms_data       = state_subsystem;
        m_joystick    = joystick;
        usingJoystick = true;
        addRequirements(subsystem);
    }

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {

        

        // SmartDashboard.putNumber("GatePosition",ms_this.getGatePosition());
    }

    @Override
    public void end(boolean interrupted)
    {
        ms_this.setCarouselSpeed(0);
    }
    
    @Override
    public boolean isFinished()
    {
        return false;
    }



}
