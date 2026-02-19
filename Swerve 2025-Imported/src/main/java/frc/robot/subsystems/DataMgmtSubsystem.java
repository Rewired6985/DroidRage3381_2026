package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class DataMgmtSubsystem extends SubsystemBase
{

    public Constants.eMode Mode = Constants.eMode.AUTO;
    public boolean inSim = false;

    public class aimStruct
    {
        public Constants.eAim state = Constants.eAim.ZERO;

        public boolean hasTarget  = false;
        public boolean isTracking = false;
        public boolean inDefense  = false;
        public boolean inEndgame  = false;
    }

    public class intakeStruct
    {
        public Constants.eIntake state = Constants.eIntake.STOP;

        public boolean inPosition = false;
        public boolean retractNow = false;
        public boolean speedZero  = false;
        public boolean deployNow  = false;
    }

    public class inputStruct
    {
        public boolean brake;
        public boolean reset;
        public boolean switchIntake;
    }

    public class positionStruct
    {

        private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private VisionSystemSim visionSim = new VisionSystemSim("test sim");

        public Pose2d pose;

        
        private double m_lastSimTime = Utils.getCurrentTimeSeconds();
        
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
        
        private AprilTagFieldLayout field_2026  = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        public PhotonPipelineResult p_result;
        public PhotonPipelineResult b_result;
        

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
        
        public StatusSignal<Angle> getHeading()
        {
            return pigeon.getYaw();
        }

        public void updateDrivetrain(CommandSwerveDrivetrain input)
        {
            drivetrain = input;
        }

        public Translation2d getTranslation()
        {
            return drivetrain.getState().Pose.getTranslation();
        }

        public void addMeasurement(Optional<EstimatedRobotPose> estimate)
        {
            poseEstimator.addVisionMeasurement(estimate.get().estimatedPose.toPose2d(), estimate.get().timestampSeconds);
        }

        public void updateEstimator()
        {

            positions[0] = drivetrain.getModule(0).getCachedPosition();
            positions[1] = drivetrain.getModule(1).getCachedPosition();
            positions[2] = drivetrain.getModule(2).getCachedPosition();
            positions[3] = drivetrain.getModule(3).getCachedPosition();

            poseEstimator.update(pigeon.getRotation2d(), positions);

            SmartDashboard.putNumber("X", poseEstimator.getEstimatedPosition().getX());
            SmartDashboard.putNumber("Y", poseEstimator.getEstimatedPosition().getY());
        }

        public void setupVisionSim()
        {
            visionSim.addCamera(sim_pablo, pablo_transform);
            visionSim.addCamera(sim_baplo, baplo_transform);
            visionSim.addAprilTags(field_2026);
        }

        public void updateVisionSim()
        {
            Transform2d visionTransform = new Transform2d(drivetrain.getState().Pose.getTranslation(),pigeon.getRotation2d());
            visionSim.update(pose.transformBy(visionTransform));
        }

        
        public void runPositionSim()
        {
            m_lastSimTime = Utils.getCurrentTimeSeconds();
            
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            
            drivetrain.updateSimState(deltaTime, 12);
        }

    }
    
    public aimStruct    aim        = new aimStruct();
    public intakeStruct intake     = new intakeStruct();
    public inputStruct  inputs     = new inputStruct();
    public positionStruct position = new positionStruct();

    public DataMgmtSubsystem()
    {

    }

    public void ShotHandler()
    {

        switch (aim.state)
        {
            case ZERO: 
            {
                if (aim.hasTarget) aim.state = Constants.eAim.HUNT;
                break;
            }
            case HUNT:
            {
                if (aim.isTracking) aim.state = Constants.eAim.STANDBY;
                break;
            }
            case FIRE: 
            {
                if (aim.hasTarget = false) aim.state = Constants.eAim.STANDBY;
                break;
            }
            case STANDBY: 
            {
                if (aim.hasTarget) aim.state = Constants.eAim.FIRE;
                if (aim.inDefense) aim.state = Constants.eAim.ZERO;
                if (aim.inEndgame) aim.state = Constants.eAim.ZERO;
                break;
            }
        }

    }

    public void IntakeHandler()
    {
        switch (intake.state)
        {
            case DEPLOY:
            {
                if (intake.inPosition) intake.state = Constants.eIntake.RUN;
                break;
            }
            case RUN:
            {
                if (intake.retractNow) intake.state = Constants.eIntake.STOP;
                break;
            }
            case STOP:
            {
                if (intake.speedZero) intake.state = Constants.eIntake.RETRACT;
                break;
            }
            case RETRACT:
            {
                if (intake.deployNow) intake.state = Constants.eIntake.DEPLOY;
                break;
            }
        }
    }

    public boolean brake() 
    {
        return inputs.brake;
    }

    public boolean reset()
    {
        return inputs.reset;
    }

    public boolean switchIntake()
    {
        return inputs.switchIntake;
    }

}
