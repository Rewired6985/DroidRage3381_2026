package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.eAim;
import frc.robot.Constants.eInitPose;
import frc.robot.Constants.eMode;
import frc.robot.generated.TunerConstants;

public class DataMgmtSubsystem extends SubsystemBase
{

    public eMode Mode = eMode.AUTO;
    public boolean inSim = false;
    public boolean AllianceIsRed = true;
    public CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

    public boolean inRest = false;

    @Override
    public void periodic()
    {
        position.updateEstimator();
    }

    public class positionStruct
    {

        private VisionSystemSim visionSim = new VisionSystemSim("test sim");

        private Pose2d initOffset;
        private Pose2d estimatedPose;

        
        private double m_lastSimTime = Utils.getCurrentTimeSeconds();

        private AprilTagFieldLayout field_2026  = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        private Matrix<N3, N1> multiTagConfidence = VecBuilder.fill(0.25, 0.25, 0.5);
        private Matrix<N3, N1> singleTagConfidence = VecBuilder.fill(2,   2,    4);
        private Matrix<N3, N1> limelightConfidence = VecBuilder.fill(2,   2,    4);

        //init positions & rotations
        final Rotation2d blueInitRotation = new Rotation2d(Math.toRadians(0));
        final Rotation2d redInitRotation  = new Rotation2d(Math.toRadians(180));

        final Pose2d leftBluePosition    = new Pose2d(3.5,5,redInitRotation);
        final Pose2d middleBluePosition  = new Pose2d(3.5,4,redInitRotation);  
        final Pose2d rightBluePosition   = new Pose2d(3.5,3,redInitRotation);
        final Pose2d leftRedPosition     = new Pose2d(13,3,blueInitRotation);
        final Pose2d middleRedPosition   = new Pose2d(13,4,blueInitRotation);
        final Pose2d rightRedPosition    = new Pose2d(13,5,blueInitRotation);

        //right
        //pablo configs                          (X,Y,Z,R,P,Y)
        private double[] pablo_pos                 = {0.1524, -0.3429, 0.4699, 0, -10, 90};
        private PhotonCamera pablo                 = new PhotonCamera("DR3381_pablo");
        private Translation3d pablo_translation    = new Translation3d(pablo_pos[0],pablo_pos[1],pablo_pos[2]);
        private Rotation3d    pablo_rotation       = new Rotation3d(Math.toRadians(pablo_pos[3]),Math.toRadians(pablo_pos[4]),Math.toRadians(pablo_pos[5]));
        private Transform3d   pablo_transform      = new Transform3d(pablo_translation, pablo_rotation);
        private SimCameraProperties pabloProp      = new SimCameraProperties();
        private PhotonCameraSim sim_pablo          = new PhotonCameraSim(pablo, pabloProp);
        public PhotonPoseEstimator pablo_estimator = new PhotonPoseEstimator(field_2026, pablo_transform);

        //left
        //baplo configs                           (X,Y,Z,R,P,Y)
        private double[] baplo_pos              = {0.1524, 0.3429, 0.4699, 0, -10, -90};
        private PhotonCamera baplo              = new PhotonCamera("DR3381_baplo");
        private Translation3d baplo_translation = new Translation3d(baplo_pos[0],baplo_pos[1],baplo_pos[2]);
        private Rotation3d    baplo_rotation    = new Rotation3d(Math.toRadians(baplo_pos[3]),Math.toRadians(baplo_pos[4]),Math.toRadians(baplo_pos[5]));
        private Transform3d   baplo_transform   = new Transform3d(baplo_translation, baplo_rotation);
        private SimCameraProperties baploProp   = new SimCameraProperties();
        private PhotonCameraSim sim_baplo       = new PhotonCameraSim(baplo, baploProp);
        public PhotonPoseEstimator baplo_estimator   = new PhotonPoseEstimator(field_2026, baplo_transform);

        private String LimelightName = "";


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

        private void updateEstimatePablo()
        {

            List<PhotonPipelineResult> list = pablo.getAllUnreadResults();

            list.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> 
            {
                int val;

                if (a.getTimestampSeconds() >= b.getTimestampSeconds()) val = 1;
                else                                                    val = -1;

                return val;
            });


            for (PhotonPipelineResult result : list)
            {
                if (result.hasTargets())
                {
                    Optional<EstimatedRobotPose>  estimate = pablo_estimator
                                                            .estimateCoprocMultiTagPose(result);
                    Matrix<N3, N1> confidence = multiTagConfidence;
                    if (estimate.isEmpty())      
                    {
                        estimate   = pablo_estimator.estimateLowestAmbiguityPose(result);
                        confidence = singleTagConfidence;
                    }
                    addPhotonMeasurement(estimate, confidence);
                }
            }
        }

        private void updateEstimateBaplo()
        {

            List<PhotonPipelineResult> list = baplo.getAllUnreadResults();

            list.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> 
            {
                int val;

                if (a.getTimestampSeconds() >= b.getTimestampSeconds()) val = 1;
                else                                                    val = -1;

                return val;
            });


            for (PhotonPipelineResult result : list)
            {
                if (result.hasTargets())
                {
                    Optional<EstimatedRobotPose>  estimate = baplo_estimator
                                                            .estimateCoprocMultiTagPose(result);
                    Matrix<N3, N1> confidence = multiTagConfidence;
                    if (estimate.isEmpty())      
                    {
                        estimate   = baplo_estimator.estimateLowestAmbiguityPose(result);
                        confidence = singleTagConfidence;
                    }
                    addPhotonMeasurement(estimate, confidence);
                }
            }
        }

        public void updateEstimateLimelight()
        {
            double yaw = getYaw();

            LimelightHelpers.SetRobotOrientation(LimelightName, yaw, 0.0, 0.0, 0.0, 0.0, 0.0);

            LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightName);

            addLimelightMeasurement(estimate, limelightConfidence);

        }

        /**
         * specifically used for getting the StatusSignal<Angle> heading of the robot
         * @return
         */
        public StatusSignal<Angle> getHeading()
        {
            return pigeon.getYaw();
        }

        /**
         * specifically for getting a yaw double, between -180 and 180 degrees
         * @return
         */
        public double getYaw()
        {
            double yaw = rolloverHelper(pigeon.getYaw().getValueAsDouble());
            return yaw;
        }

        public void updateDrivetrain(CommandSwerveDrivetrain input)
        {
            m_drivetrain = input;
        }

        public void addPhotonMeasurement(Optional<EstimatedRobotPose> estimate, Matrix<N3, N1> confidence)
        {
            poseEstimator.addVisionMeasurement(estimate.get().estimatedPose.toPose2d(), 
                                               estimate.get().timestampSeconds, 
                                               confidence);
        }

        public void addLimelightMeasurement(LimelightHelpers.PoseEstimate estimate, Matrix<N3, N1> confidence)
        {
            poseEstimator.addVisionMeasurement(estimate.pose, 
                                               estimate.timestampSeconds, 
                                               confidence);
        }

        public void updateEstimator()
        {

            updateEstimateBaplo();
            updateEstimatePablo();
            updateEstimateLimelight();

            positions[0] = m_drivetrain.getModule(0).getCachedPosition();
            positions[1] = m_drivetrain.getModule(1).getCachedPosition();
            positions[2] = m_drivetrain.getModule(2).getCachedPosition();
            positions[3] = m_drivetrain.getModule(3).getCachedPosition();

            poseEstimator.update(pigeon.getRotation2d(), positions);
            estimatedPose = poseEstimator.getEstimatedPosition();
        }

        public double getEstX()
        {
            double return_value = (estimatedPose.getX() * Constants.FEET_PER_METER);
            SmartDashboard.putNumber("robotX", return_value);
            return return_value;
        }

        public double getEstY()
        {
            double return_value = (estimatedPose.getY() * Constants.FEET_PER_METER);
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

        
        public void updateInitPosition(eInitPose position)
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
        public eAim state = eAim.ZERO;

        public boolean shoot = false;
        public boolean tracking = false;
        public boolean stop  = false;

        public double turretX;
        public double turretY;

        public double[] target = {0, 0, 0};
        public double velocityX = 0;
        public double velocityY = 0;
        public double fuelVelocity = 0;

        public double distance = 0;
        public double flightTime = 0;

        public double angle = 75 * Math.PI / 180;

        public void updateVelocity()
        {
            SwerveDriveKinematics kinematics = m_drivetrain.getKinematics();
            SwerveModuleState[]   modules = m_drivetrain.getState().ModuleStates;
            ChassisSpeeds         speeds = kinematics.toChassisSpeeds(modules);

            velocityX = speeds.vxMetersPerSecond;
            velocityY = speeds.vyMetersPerSecond;
        }

        public void updateAngle(int param)
        {
            angle = param * Math.PI/180;
        }

    }

    public class inputStruct
    {
        public boolean brake = false;
        public boolean resetGyro = false;
        public boolean intake = false;
        public boolean carousel = false;
    }

    public aimStruct      aim      = new aimStruct();
    public inputStruct    inputs   = new inputStruct();
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
                if (!aim.stop) aim.state = eAim.TRANSITION;
                break;
            }
            case TRANSITION: 
            {
                if (aim.tracking && aim.shoot) aim.state = eAim.FIRE;
                if (aim.stop)   aim.state = eAim.ZERO;
                break;
            }
            case FIRE:
            {
                if (!aim.shoot) aim.state = eAim.TRANSITION;
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

    /**
     * automatically calculates error term;
     * ONLY FOR USE WITH ROLLOVERS
     * @param actual
     * @param command
     * @return error
     */
    public double errorHelper(double actual, double command)
    {
        double error = command - actual;

        if      ((actual >  90) && (command < -90)) error = error + 360;
        else if ((actual < -90) && (command >  90)) error = error - 360;

        return error;
    }

    public double rolloverHelper(double angle)
    {
        double value = angle - Math.round(angle/360) * 360;
        return value;
    }

    public boolean brake() 
    {
        return inputs.brake;
    }

    public boolean resetGyro()
    {
        return inputs.resetGyro;
    }

    public boolean intake()
    {
        return inputs.intake;
    }

    public boolean carousel()
    {
        return inputs.carousel;
    }

}
