package frc.robot.Utils;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Utils.Math.Funcs;

public class GamePieceCamera {
    public enum GamePieceType{
        Algea(0.413/2.0),
        Coral(0.114),
        Fuel(1);

        private final double radius;

        GamePieceType(double radius){
            this.radius = radius;
        }

        public double getRadius(){
            return radius;
        }
    }

    private double m_frameWidth;
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
                           double width, double xFov)
    {
        m_cam = new PhotonCamera(camName);
        m_gamePieceType = gamePieceType;
        m_robotToCam = robotToCam;
        m_frameWidth = width;
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

    public double getXFov(){
        return m_xFov;
    }
}
