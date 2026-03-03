// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.CarouselCommand;
import frc.robot.commands.DataMgmtCommand;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CarouselSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DataMgmtSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

@SuppressWarnings("unused")
public class RobotContainer {


    private final DataMgmtSubsystem ms_data          = new DataMgmtSubsystem();
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final DrivetrainSubsystem ms_drivetrain  = new DrivetrainSubsystem();
    private final IntakeSubsystem ms_intake          = new IntakeSubsystem();
    private final CarouselSubsystem ms_carousel      = new CarouselSubsystem();
    private final ShooterSubsystem ms_shooter        = new ShooterSubsystem();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        
    public AprilTagFieldLayout field_2026  = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    private DoubleLogEntry XLog;
    private DoubleLogEntry YLog;


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    //private final SwerveRequest.RobotCentric rdrive = new SwerveRequest.RobotCentric();

    private final Telemetry  logger         = new Telemetry(MaxSpeed);

    SendableChooser<Command> m_chooser        = new SendableChooser<>();
    SendableChooser<Constants.eInitPose> m_poseChooser    = new SendableChooser<>();
    SendableChooser<Constants.eAutoGoal> m_goalChooser   = new SendableChooser<>();

    public boolean returnFalse()
    {
        return false;
    }


    private final CommandXboxController controller  = new CommandXboxController(0);
    private final CommandXboxController controller2 = new CommandXboxController(1);
    private final Joystick              joystick    = new Joystick(5);
    // private final CommandXboxController a_pac1 = new CommandXboxController(2);
    // private final CommandXboxController a_pac2 = new CommandXboxController(3);


    private void configureBindings()
    {
        Trigger brakeTrigger = new Trigger(ms_data::brake);
        Trigger resetField   = new Trigger(ms_data::resetGyro);
        Trigger callIntake = new Trigger(ms_data::callIntake);

        ms_drivetrain.setDefaultCommand(new DrivetrainCommand(ms_drivetrain, ms_data, joystick));
        ms_carousel.setDefaultCommand(new CarouselCommand(ms_carousel, ms_data, joystick));
        ms_shooter.setDefaultCommand(new ShooterCommand(ms_shooter, ms_data, joystick));

        ms_data.setDefaultCommand(new DataMgmtCommand(ms_data, joystick));
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(

        drivetrain.applyRequest(() ->
        drive.withVelocityX(ms_drivetrain.m_Xswerve * MaxSpeed)
             .withVelocityY(ms_drivetrain.m_Yswerve * MaxSpeed)
             .withRotationalRate(ms_drivetrain.m_Rswerve * MaxAngularRate))
        );

        controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        brakeTrigger  .whileTrue(drivetrain.applyRequest(() -> brake));
        controller.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))
        ));

        callIntake.whileTrue(new IntakeCommand(ms_intake, ms_data));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on y press
        controller.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        resetField.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    public RobotContainer()
    {

        DataLogManager.start();
        DataLog log = DataLogManager.getLog();

        XLog = new DoubleLogEntry(log, "x");
        YLog = new DoubleLogEntry(log, "y");

        final int choose_left   = 1;
        final int choose_middle = 2;
        final int choose_right  = 3;

        

        configureBindings();
        
        ms_data.setAlliance();
        ms_data.position.setupVisionSim();


        m_poseChooser.setDefaultOption("middle",Constants.eInitPose.MIDDLE);
        m_poseChooser.addOption("left"         ,Constants.eInitPose.LEFT);
        m_poseChooser.addOption("middle"       ,Constants.eInitPose.MIDDLE);
        m_poseChooser.addOption("right"        ,Constants.eInitPose.RIGHT);
        SmartDashboard.putData("position",     m_poseChooser);

        m_goalChooser.setDefaultOption("no move"   ,Constants.eAutoGoal.NOMOVE);
        m_goalChooser.addOption("no move"          ,Constants.eAutoGoal.NOMOVE);
        m_goalChooser.addOption("depot only"       ,Constants.eAutoGoal.DEPOT);
        m_goalChooser.addOption("outpost only"     ,Constants.eAutoGoal.OUTPOST);
        m_goalChooser.addOption("depot, outpost"   ,Constants.eAutoGoal.DEPOT_THEN_OUTPOST);
        m_goalChooser.addOption("outpost, depot"   ,Constants.eAutoGoal.OUTPOST_THEN_DEPOT);
        SmartDashboard.putData("goal"            ,m_goalChooser);

    }

    public void updateChooserValues()
    {
        ms_data.position.updateInitPosition(m_poseChooser.getSelected());
        ms_drivetrain.updateParams(m_goalChooser.getSelected());
        SmartDashboard.putString("currentGoal", ms_drivetrain.goal.toString());
    }
            
    public Command getAutonomousCommand() 
    {
        return m_chooser.getSelected();
    }

    public void updateLogger()
    {
        XLog.append(drivetrain.getState().Pose.getX());
        YLog.append(drivetrain.getState().Pose.getY());
    }

    public void setDriveMode(Constants.eMode mode)
    {
        ms_data.Mode = mode;
    }

    public void setSimState(boolean in_sim)
    {
        ms_data.inSim = in_sim;
    }

    public void updateDrivetrain()
    {
        ms_data.position.updateDrivetrain(drivetrain);
    }

    public void doSimStuff()
    {
        ms_data.position.updateVisionSim();
        ms_data.position.runPositionSim();
    }
    
}