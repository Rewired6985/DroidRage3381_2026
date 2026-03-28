// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

	private Command m_autonomousCommand;

	private final RobotContainer m_robotContainer;
	private boolean inSim = false;


	public Robot() 
	{
		m_robotContainer = new RobotContainer();
   		// CameraServer.startAutomaticCapture();
	}




	@Override 
	public void robotPeriodic() 
	{
		CommandScheduler.getInstance().run();
	}


	public void disabledInit() 
	{

	}

	@Override
	public void disabledPeriodic() 
	{
		m_robotContainer.updateChooserValues();
	}

	@Override
	public void disabledExit() 
	{

	}

	@Override
	public void autonomousInit() 
	{

		m_autonomousCommand = m_robotContainer.getAutonomousCommand();
		m_robotContainer.setDriveMode(Constants.eMode.AUTO);
		m_robotContainer.setSimState(inSim);

        if (m_autonomousCommand != null) 
        {
            m_autonomousCommand.schedule();
        }
	}

	@Override
	public void autonomousPeriodic() 
	{

	}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() 
	{
		if (m_autonomousCommand != null) 
	{
	    m_autonomousCommand.cancel();
	}
	    m_robotContainer.setDriveMode(Constants.eMode.TELEOP);
		m_robotContainer.setSimState(inSim);
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	@Override
	public void testInit() 
	{
	    CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}


	@Override
	public void simulationPeriodic() 
	{
	    inSim = true;
	    m_robotContainer.doSimStuff();
	    // m_robotContainer.updateLogger();
	}

}
