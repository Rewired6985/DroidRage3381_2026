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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class DataMgmtSubsystem extends SubsystemBase
{

    public Constants.eMode Mode = Constants.eMode.AUTO;
    public boolean inSim = false;
    public boolean AllianceIsRed = true;
    private CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

    public class positionStruct
    {

        private VisionSystemSim visionSim = new VisionSystemSim("test sim");

        public Pose2d initOffset;
        private Pose2d estimatedPose;

        
        private double m_lastSimTime = Utils.getCurrentTimeSeconds();

        private AprilTagFieldLayout field_2026  = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        //init positions & rotations
        final Rotation2d blueInitRotation = new Rotation2d(Math.toRadians(180));
        final Rotation2d redInitRotation  = new Rotation2d(Math.toRadians(0));

        final Pose2d leftBluePosition    = new Pose2d(3.5,5,redInitRotation);
        final Pose2d middleBluePosition  = new Pose2d(3.5,4,redInitRotation);  
        final Pose2d rightBluePosition   = new Pose2d(3.5,3,redInitRotation);
        final Pose2d leftRedPosition     = new Pose2d(13,3,blueInitRotation);
        final Pose2d middleRedPosition   = new Pose2d(13,4,blueInitRotation);
        final Pose2d rightRedPosition    = new Pose2d(13,5,blueInitRotation);

        //pablo configs                          (X,Y,Z,R,P,Y)
        public double[] pablo_pos              = {0.1,-0.1,0.1,0,-15,15};
        public PhotonCamera pablo              = new PhotonCamera("DR3381_pablo");
        public Translation3d pablo_translation = new Translation3d(pablo_pos[0],pablo_pos[1],pablo_pos[2]);
        public Rotation3d    pablo_rotation    = new Rotation3d(Math.toRadians(pablo_pos[3]),Math.toRadians(pablo_pos[4]),Math.toRadians(pablo_pos[5]));
        public Transform3d   pablo_transform   = new Transform3d(pablo_translation, pablo_rotation);
        public SimCameraProperties pabloProp   = new SimCameraProperties();
        public PhotonCameraSim sim_pablo       = new PhotonCameraSim(pablo, pabloProp);
        public PhotonPoseEstimator pablo_estimator   = new PhotonPoseEstimator(field_2026, pablo_transform);
        public PhotonPipelineResult p_result;

        //baplo configs                           (X,Y,Z,R,P,Y)
        public double[] baplo_pos              = {0.1,0.1,0.1,0,-15,-15};
        public PhotonCamera baplo              = new PhotonCamera("DR3381_baplo");
        public Translation3d baplo_translation = new Translation3d(baplo_pos[0],baplo_pos[1],baplo_pos[2]);
        public Rotation3d    baplo_rotation    = new Rotation3d(Math.toRadians(baplo_pos[3]),Math.toRadians(baplo_pos[4]),Math.toRadians(baplo_pos[5]));
        public Transform3d   baplo_transform   = new Transform3d(baplo_translation, baplo_rotation);
        public SimCameraProperties baploProp   = new SimCameraProperties();
        public PhotonCameraSim sim_baplo       = new PhotonCameraSim(baplo, baploProp);
        public PhotonPoseEstimator baplo_estimator   = new PhotonPoseEstimator(field_2026, baplo_transform);
        public PhotonPipelineResult b_result;


        //variables for setting up simulated drivetrain
        Pigeon2 pigeon = m_drivetrain.getPigeon2();

        SwerveModulePosition[] positions = 
        {
            m_drivetrain.getModules()[0].getCachedPosition(),
            m_drivetrain.getModules()[1].getCachedPosition(),
            m_drivetrain.getModules()[2].getCachedPosition(),
            m_drivetrain.getModules()[3].getCachedPosition()
        };

        private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator
        (
            m_drivetrain.getKinematics(),  
            Rotation2d.fromRotations(pigeon.getYaw().getValueAsDouble()),
            positions,
            new Pose2d(4.5,4,pigeon.getRotation2d())
        );

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
            m_drivetrain = input;
        }

        public void addMeasurement(Optional<EstimatedRobotPose> estimate)
        {
            poseEstimator.addVisionMeasurement(estimate.get().estimatedPose.toPose2d(), estimate.get().timestampSeconds);
        }

        public void updateEstimator()
        {

            positions[0] = m_drivetrain.getModule(0).getCachedPosition();
            positions[1] = m_drivetrain.getModule(1).getCachedPosition();
            positions[2] = m_drivetrain.getModule(2).getCachedPosition();
            positions[3] = m_drivetrain.getModule(3).getCachedPosition();

            poseEstimator.update(pigeon.getRotation2d(), positions);
            estimatedPose = poseEstimator.getEstimatedPosition();
        }

        public double getEstX()
        {
            double return_value = (estimatedPose.getX() * 3.2804);
            SmartDashboard.putNumber("robotX", return_value);
            return return_value;
        }

        public double getEstY()
        {
            double return_value = (estimatedPose.getY() * 3.2804);
            SmartDashboard.putNumber("robotY", return_value);
            return return_value;
        }

        public void setupVisionSim()
        {
            visionSim.addCamera(sim_pablo, pablo_transform);
            visionSim.addCamera(sim_baplo, baplo_transform);
            visionSim.addAprilTags(field_2026);
        }

        public void updateVisionSim()
        {
            Transform2d visionTransform = new Transform2d(m_drivetrain.getState().Pose.getTranslation(),pigeon.getRotation2d());
            visionSim.update(initOffset.transformBy(visionTransform));
        }

        
        public void runPositionSim()
        {
            m_lastSimTime = Utils.getCurrentTimeSeconds();
            
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            
            m_drivetrain.updateSimState(deltaTime, 12);
        }

        
        public void updateInitPosition(Constants.eInitPose position)
        {
            Optional<Alliance> ally = DriverStation.getAlliance();
            boolean AllianceIsRed = true;
            Pose2d pose = new Pose2d();

            if (ally.isPresent())
            {
                if (ally.get() == Alliance.Blue) AllianceIsRed = false;  
            }

            switch (position)
            {
                case LEFT:
                {
                    if (AllianceIsRed) pose = leftRedPosition;
                    else               pose = leftBluePosition;
                    break;
                }
                case MIDDLE:
                {
                    if (AllianceIsRed) pose = middleRedPosition;
                    else               pose = middleBluePosition;
                    break;
                }
                case RIGHT:
                {
                    if (AllianceIsRed) pose = rightRedPosition;
                    else               pose = rightBluePosition;
                    break;
                }
            }

            initOffset = pose;

        }

    }

    public class aimStruct
    {
        public Constants.eAim state = Constants.eAim.ZERO;

        public boolean hasTarget  = false;
        public boolean isTracking = false;
        public boolean inDefense  = false;
        public boolean inEndgame  = false;

        public double[] target = {0, 0, 0};
        public double velocityX = 0;
        public double velocityY = 0;
        public double fuelVelocity = 0;

        public double distance = 0;
        public double flightTime = 0;

        public double angle = 70 * Math.PI / 180;

        public final double SHOOTERHEIGHT = 2.208;

        public final double[] REDHUB   = {39.047, 13.193,  6};
        public final double[] LEFTRED  = {46.612, 19.79,   0};
        public final double[] RIGHTRED = {46.612,  6.5967, 0};

        public final double[] BLUEHUB   = {15.13, 13.193,  6};
        public final double[] LEFTBLUE  = {7.565, 19.79,   0};
        public final double[] RIGHTBLUE = {7.565,  6.5967, 0};

        public final double[] LEFTNEUTRAL  = {27.088, 19.79,   0};
        public final double[] RIGHTNEUTRAL = {27.088,  6.5967, 0};

        public void updateVelocity()
        {
            SwerveDriveKinematics kinematics = m_drivetrain.getKinematics();
            SwerveModuleState[]   modules = m_drivetrain.getState().ModuleStates;
            ChassisSpeeds         speeds = kinematics.toChassisSpeeds(modules);

            velocityX = speeds.vxMetersPerSecond;
            velocityY = speeds.vyMetersPerSecond;

        }

    }

    public class inputStruct
    {
        public boolean brake;
        public boolean resetGyro;
        public boolean callIntake;
    }

    public class autoStruct
    {
        public boolean doDepot   = false;
        public boolean doOutpost = false;

        public boolean beginRest = false;
        public boolean atCenter = false;
        public boolean atDepot = false;
        public boolean atOutpost = false;
        public boolean depotCollected = false;
        public boolean outpostCollected = false;

        public boolean depotFirst   = false;
        public boolean outpostFirst = false;

        public Constants.eAutoGoal Target = Constants.eAutoGoal.NOMOVE;


        public Constants.eAuto state = Constants.eAuto.REST;

        public void updateParams(Constants.eAutoGoal objective)
        {
            switch (objecti)
            {
                
                case NOMOVE:
                {
                    break;
                }
                case DEPOT:
                {
                    break;
                }
                case OUTPOST:
                {
                    break;
                }
                case DEPOT_THEN_OUTPOST:
                {
                    break;
                }
                case OUTPOST_THEN_DEPOT:
                {
                    break;
                }
            }
        }
        
    }
    
    public aimStruct      aim      = new aimStruct();
    public inputStruct    inputs   = new inputStruct();
    public positionStruct position = new positionStruct();
    public autoStruct     auto     = new autoStruct();

    
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
                if (aim.hasTarget  = false) aim.state = Constants.eAim.STANDBY;
                if (aim.isTracking = false) aim.state = Constants.eAim.STANDBY;
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

    public void autoHandler()
    {
        switch (auto.state)
        {
            case REST:
            {
                if ((auto.doOutpost && (auto.outpostCollected == false)) ||
                    (auto.doDepot   && (auto.depotCollected   == false))) auto.state = Constants.eAuto.TOCENTER;
                break;
            }
            case TODEPOT:
            {
                if (auto.atDepot) auto.state = Constants.eAuto.COLLECTDEPOT;
                break;
            }
            case TOOUTPOST:
            {
                if (auto.atOutpost) auto.state = Constants.eAuto.COLLECTOUTPOST;
                break;
            }
            case TOCENTER:
            {
                if (auto.atCenter)
                {
                    if (auto.depotFirst)
                    {
                        if (auto.depotCollected) auto.state = Constants.eAuto.TOOUTPOST;
                        else                     auto.state = Constants.eAuto.TODEPOT;
                    }
                    else if (auto.outpostFirst)
                    {
                        if (auto.outpostCollected) auto.state = Constants.eAuto.TODEPOT;
                        else                       auto.state = Constants.eAuto.TOOUTPOST;
                    }
                }
                break;
            }
            case COLLECTDEPOT:
            {
                if (auto.depotCollected)
                {
                    if (auto.depotFirst) auto.state = Constants.eAuto.TOCENTER;
                    else                 auto.state = Constants.eAuto.REST;
                }
                break;
            }
            case COLLECTOUTPOST:
            {
                if (auto.outpostCollected)
                {
                    if (auto.outpostFirst) auto.state = Constants.eAuto.TOCENTER;
                    else                   auto.state = Constants.eAuto.REST;
                }
                break;
            }
        }
    }

    
    public void setAlliance()
    {
        
        Optional<Alliance> ally = DriverStation.getAlliance();
        
        if (ally.isPresent())
        {
            if (ally.get() == Alliance.Blue) AllianceIsRed = false;  
        }

    }

    public boolean brake() 
    {
        return inputs.brake;
    }

    public boolean resetGyro()
    {
        return inputs.resetGyro;
    }

    public boolean callIntake()
    {
        return inputs.callIntake;
    }

}
