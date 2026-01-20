package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DataMgmtSubsystem extends SubsystemBase
{
    

    final PhotonCamera pablo = new PhotonCamera("DR3381_pablo");
    final PhotonCamera baplo = new PhotonCamera("DR3381_baplo");

    public DataMgmtSubsystem(){}

    public boolean pabloResultValid;
    public boolean baploResultValid;

    public void StateHandler()
    {

    }

    public PhotonTrackedTarget getPabloTargets()
    {
        PhotonPipelineResult result = pablo.getLatestResult();
        PhotonTrackedTarget return_value;

        return_value     = result.getBestTarget();
        pabloResultValid = result.hasTargets();

        return return_value;
    }

    public PhotonTrackedTarget getBaploTargets()
    {
        PhotonPipelineResult result = baplo.getLatestResult();
        PhotonTrackedTarget return_value;

        return_value     = result.getBestTarget();
        baploResultValid = result.hasTargets();

        return return_value;
    }

}
