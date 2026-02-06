package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class PositionChooserCommand extends Command
{

    final Rotation2d blueRotation = new Rotation2d(Math.toRadians(180));
    final Rotation2d redRotation  = new Rotation2d(Math.toRadians(0));

    final Pose2d leftBluePosition    = new Pose2d(3.5,5,redRotation);
    final Pose2d middleBluePosition  = new Pose2d(3.5,4,redRotation);  
    final Pose2d rightBluePosition   = new Pose2d(3.5,3,redRotation);
    final Pose2d leftRedPosition     = new Pose2d(13,3,blueRotation);
    final Pose2d middleRedPosition   = new Pose2d(13,4,blueRotation);
    final Pose2d rightRedPosition    = new Pose2d(13,5,blueRotation);

    private int m_position;

    public PositionChooserCommand(int position) 
    {
        m_position = position;
    }
    
    public Pose2d getInitPosition()
    {
        Optional<Alliance> ally = DriverStation.getAlliance();
        boolean AllianceIsRed = true;
        Pose2d return_pose = new Pose2d();

        if (ally.isPresent())
        {
            if (ally.get() == Alliance.Blue) AllianceIsRed = false;  
        }

        switch (m_position)
        {
            case 0:
            {
                break;
            }
            case 1:
            {
                if (AllianceIsRed) return_pose = leftRedPosition;
                else               return_pose = leftBluePosition;
                break;
            }
            case 2:
            {
                if (AllianceIsRed) return_pose = middleRedPosition;
                else               return_pose = middleBluePosition;
                break;
            }
            case 3:
            {
                if (AllianceIsRed) return_pose = rightRedPosition;
                else               return_pose = rightBluePosition;
                break;
            }
        }

        return return_pose;

    }

}
