// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot 
{

	private Command m_autonomousCommand;

	private final RobotContainer m_robotContainer;
	
	
	private boolean inSim = false;

	private DoubleLogEntry XLog;
    private DoubleLogEntry YLog;


	public Robot() 
	{
		m_robotContainer = new RobotContainer();
		
		
        DataLogManager.start();
        DataLog log = DataLogManager.getLog();

        XLog = new DoubleLogEntry(log, "x");
        YLog = new DoubleLogEntry(log, "y");
   		// CameraServer.startAutomaticCapture();
	}




	@Override 
	public void robotPeriodic() 
	{
		CommandScheduler.getInstance().run();
		SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
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

	private double number = 0;

	@Override
	public void teleopPeriodic() 
	{
		
		number++;
		XLog.append(number);

	    // double X = m_robotContainer.getLogValues()[0];
		// double Y = m_robotContainer.getLogValues()[1];

		// if (X != Double.NaN) XLog.append(X);
		// if (Y != Double.NaN) YLog.append(Y);
	}

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
	}

}
