package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DataMgmtSubsystem extends SubsystemBase
{
    

    private PhotonCamera pablo = new PhotonCamera("DR3381_pablo");
    private PhotonCamera baplo = new PhotonCamera("DR3381_baplo");

    public DataMgmtSubsystem(){}
        
    private PhotonPipelineResult p_result;
    private PhotonPipelineResult b_result;

    public void StateHandler()
    {

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
