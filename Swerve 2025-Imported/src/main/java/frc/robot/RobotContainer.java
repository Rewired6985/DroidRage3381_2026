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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.PositionChooserCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CarouselSubsystem;
// import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DataMgmtSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

@SuppressWarnings("unused")
public class RobotContainer {


    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final DataMgmtSubsystem ms_data          = new DataMgmtSubsystem();
    private final DrivetrainSubsystem ms_drivetrain  = new DrivetrainSubsystem();
    private final IntakeSubsystem ms_intake          = new IntakeSubsystem();
    private final CarouselSubsystem ms_carousel      = new CarouselSubsystem();
    // private final ShooterSubsystem ms_shooter        = new ShooterSubsystem();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private double m_lastSimTime = Utils.getCurrentTimeSeconds();

    private VisionSystemSim visionSim = new VisionSystemSim("test sim");
        
    
    public AprilTagFieldLayout field_2026  = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    private DoubleLogEntry XLog;
    private DoubleLogEntry YLog;

    //pablo configs                          (X,Y,Z,R,P,Y)
    public double[] pablo_pos              = {0.1,-0.1,0.1,0,-15,15};
    public PhotonCamera pablo              = new PhotonCamera("DR3381_pablo");
    public Translation3d pablo_translation = new Translation3d(pablo_pos[0],pablo_pos[1],pablo_pos[2]);
    public Rotation3d    pablo_rotation    = new Rotation3d(Math.toRadians(pablo_pos[3]),Math.toRadians(pablo_pos[4]),Math.toRadians(pablo_pos[5]));
    public Transform3d   pablo_transform   = new Transform3d(pablo_translation, pablo_rotation);
    public SimCameraProperties pabloProp   = new SimCameraProperties();
    public PhotonCameraSim sim_pablo       = new PhotonCameraSim(pablo, pabloProp);
    public PhotonPoseEstimator pablo_estimator   = new PhotonPoseEstimator(field_2026, pablo_transform);
    
    //baplo configs                           (X,Y,Z,R,P,Y)
    public double[] baplo_pos              = {0.1,0.1,0.1,0,-15,-15};
    public PhotonCamera baplo              = new PhotonCamera("DR3381_baplo");
    public Translation3d baplo_translation = new Translation3d(baplo_pos[0],baplo_pos[1],baplo_pos[2]);
    public Rotation3d    baplo_rotation    = new Rotation3d(Math.toRadians(baplo_pos[3]),Math.toRadians(baplo_pos[4]),Math.toRadians(baplo_pos[5]));
    public Transform3d   baplo_transform   = new Transform3d(baplo_translation, baplo_rotation);
    public SimCameraProperties baploProp   = new SimCameraProperties();
    public PhotonCameraSim sim_baplo       = new PhotonCameraSim(baplo, baploProp);
    public PhotonPoseEstimator baplo_estimator   = new PhotonPoseEstimator(field_2026, baplo_transform);

    public Pose2d pose;
        
    public PhotonPipelineResult p_result;
    public PhotonPipelineResult b_result;

    SwerveModulePosition[] positions = 
    {
    drivetrain.getModules()[0].getCachedPosition(),
    drivetrain.getModules()[1].getCachedPosition(),
    drivetrain.getModules()[2].getCachedPosition(),
    drivetrain.getModules()[3].getCachedPosition()
    };

    Pigeon2 pigeon = drivetrain.getPigeon2();
    

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator
    (
        drivetrain.getKinematics(),  
        Rotation2d.fromRotations(pigeon.getYaw().getValueAsDouble()),
        positions,
        new Pose2d(4.5,4,pigeon.getRotation2d())
    );


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    //private final SwerveRequest.RobotCentric rdrive = new SwerveRequest.RobotCentric();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    SendableChooser<Command> m_chooser = new SendableChooser<>();
    SendableChooser<Command> p_chooser = new SendableChooser<>();


    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandXboxController controller2 = new CommandXboxController(1);
    private final Joystick              joystick   = new Joystick(5);
    // private final CommandXboxController a_pac1 = new CommandXboxController(2);
    // private final CommandXboxController a_pac2 = new CommandXboxController(3);


    private void configureBindings() 
    {


        ms_drivetrain.setDefaultCommand(new DrivetrainCommand(ms_drivetrain, ms_data, controller));
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
         drivetrain.setDefaultCommand(

        drivetrain.applyRequest(() ->
        drive.withVelocityX(ms_drivetrain.m_Xswerve * MaxSpeed)
             .withVelocityY(ms_drivetrain.m_Yswerve * MaxSpeed)
             .withRotationalRate(ms_drivetrain.m_Rswerve * MaxAngularRate))
        );



        controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))
        ));



        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on y press
        controller.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

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
        visionSim.addCamera(sim_pablo, pablo_transform);
        visionSim.addCamera(sim_baplo, baplo_transform);
        visionSim.addAprilTags(field_2026);


        p_chooser.setDefaultOption("left",   new PositionChooserCommand(choose_left));
        p_chooser.addOption("left",          new PositionChooserCommand(choose_left));
        p_chooser.addOption("middle",        new PositionChooserCommand(choose_middle));
        p_chooser.addOption("right",         new PositionChooserCommand(choose_right));
        SmartDashboard.putData(p_chooser);
        
    }

    public void updateInitPosition()
    {
        PositionChooserCommand PoseChooser = (PositionChooserCommand)p_chooser.getSelected();
        pose = PoseChooser.getInitPosition();
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

    public void setDriveMode(int mode)
    {
        ms_data.driveMode = mode;
    }

    

    public void runPositionSim()
    {
     m_lastSimTime = Utils.getCurrentTimeSeconds();

      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;
      
      drivetrain.updateSimState(deltaTime, 12);
    }

    public void runVisionSim()
    {
        boolean pabloResultValid = getPabloValidity();
        boolean baploResultValid = getBaploValidity();


        if(pabloResultValid)
        {
            Optional<EstimatedRobotPose>  pablo_est = pablo_estimator.estimateCoprocMultiTagPose(getPabloResult());
            if (pablo_est.isEmpty())      pablo_est = pablo_estimator.estimateLowestAmbiguityPose(p_result);
            poseEstimator.addVisionMeasurement(pablo_est.get().estimatedPose.toPose2d(), pablo_est.get().timestampSeconds);
        } 

        if (baploResultValid)
        {
            Optional<EstimatedRobotPose>  baplo_est = baplo_estimator.estimateCoprocMultiTagPose(getBaploResult());
            if (baplo_est.isEmpty())      baplo_est = baplo_estimator.estimateLowestAmbiguityPose(b_result);
            poseEstimator.addVisionMeasurement(baplo_est.get().estimatedPose.toPose2d(), baplo_est.get().timestampSeconds);

        }

        Transform2d visionTransform2d = new Transform2d(drivetrain.getState().Pose.getTranslation(),pigeon.getRotation2d());

        visionSim.update(pose.transformBy(visionTransform2d));

        positions[0] = drivetrain.getModule(0).getCachedPosition();
        positions[1] = drivetrain.getModule(1).getCachedPosition();
        positions[2] = drivetrain.getModule(2).getCachedPosition();
        positions[3] = drivetrain.getModule(3).getCachedPosition();

        poseEstimator.update(pigeon.getRotation2d(), positions);

        SmartDashboard.putNumber("driveX", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("driveY", poseEstimator.getEstimatedPosition().getY());

        SmartDashboard.putBoolean("p?", pabloResultValid);
        SmartDashboard.putBoolean("b?", baploResultValid);

    }

        public PhotonPipelineResult getPabloResult()
    {
        return pablo.getLatestResult();
    }

    public PhotonPipelineResult getBaploResult()
    {
        return baplo.getLatestResult();
    }

    public boolean getPabloValidity()
    {
        p_result = pablo.getLatestResult();

        return p_result.hasTargets();
    }

    public PhotonTrackedTarget getPabloTarget()
    {
        return p_result.getBestTarget();
    }

    public boolean getBaploValidity()
    {
        b_result = baplo.getLatestResult();

        return b_result.hasTargets();
    }

    public PhotonTrackedTarget getBaploTarget()
    {
        return b_result.getBestTarget();
    }
    
}