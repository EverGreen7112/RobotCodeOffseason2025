package frc.robot.Utils;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Translation3d;

public class GamePieceCamera {
    public enum GamePieceType{
        Algea,
        Coral
    }
    private PhotonCamera m_cam;
    private GamePieceType m_gamePieceType;
    private Translation3d m_robotToCam;
    public GamePieceCamera(String camName,GamePieceType gamePieceType,Translation3d robotToCam){
        m_cam = new PhotonCamera(camName);
        m_gamePieceType = gamePieceType;
        m_robotToCam = robotToCam;
    }

    public PhotonPipelineResult getGamePieces(){
        PhotonPipelineResult lastestResult = m_cam.getLatestResult();
        if(!lastestResult.hasTargets())
            return null;
        return lastestResult;
    }

    public GamePieceType getGamePieceType(){
        return m_gamePieceType;
    }

    public Translation3d getRobotToCam(){
        return m_robotToCam;
    }
}
