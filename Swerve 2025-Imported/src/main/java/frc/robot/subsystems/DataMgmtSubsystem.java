package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DataMgmtSubsystem extends SubsystemBase
{
    // public Constants Constants = new Constants();
    public Constants.eMode Mode = Constants.eMode.AUTO;

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

    public class cameraStruct
    {
        
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
        

    }
    
    public aimStruct    aim     = new aimStruct();
    public intakeStruct intake  = new intakeStruct();
    public inputStruct  inputs  = new inputStruct();
    public cameraStruct cameras = new cameraStruct();

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
