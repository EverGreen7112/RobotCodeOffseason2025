package frc.robot.Utils;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Utils.Math.Funcs;

public class GamePieceCamera {
    public enum GamePieceType{
        Algea,
        Coral
    }

    private double m_frameWidth, m_frameHieght;
    private PhotonCamera m_cam;
    private GamePieceType m_gamePieceType;
    private Transform3d m_robotToCam;
    private double m_xFov;
    public GamePieceCamera(String camName,GamePieceType gamePieceType,Transform3d robotToCam){
        m_cam = new PhotonCamera(camName);
        m_gamePieceType = gamePieceType;
        m_robotToCam = robotToCam;
    }

    public GamePieceCamera(String camName, GamePieceType gamePieceType, Transform3d robotToCam,
                           double width, double hieght, double xFov)
    {
        m_cam = new PhotonCamera(camName);
        m_gamePieceType = gamePieceType;
        m_robotToCam = robotToCam;
        m_frameWidth = width;
        m_frameHieght = hieght;
        m_xFov = xFov;

    }

    public PhotonPipelineResult getGamePieces(){
        PhotonPipelineResult lastestResult = m_cam.getLatestResult();
        if(!lastestResult.hasTargets()) {
            return null;
        }
        return lastestResult;
    }

    public GamePieceType getGamePieceType(){
        return m_gamePieceType;
    }

    public Transform3d getRobotToCam(){
        return m_robotToCam;
    }

    public double getWidth(){
        return m_frameWidth;
    }

    public double getHieght(){
        return m_frameHieght;
    }

    public double getXFov(){
        return m_xFov;
    }
}
