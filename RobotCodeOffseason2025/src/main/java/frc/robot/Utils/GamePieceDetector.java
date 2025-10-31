package frc.robot.Utils;

import java.lang.annotation.Documented;
import java.util.ArrayList;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.GamePieceCamera.GamePieceType;

import frc.robot.Utils.Math.Funcs;

public class GamePieceDetector {
    private static final double CORAL_RADIUS = 0.114;
    private static final GamePieceCamera[] M_CAMS = {
        new GamePieceCamera("left_cam", GamePieceType.Coral, new Translation3d(0,0 , 0), 58,640,360)};
    private static GamePieceDetector m_instance = new GamePieceDetector();

    public GamePieceDetector() {

    }

    public static GamePieceDetector getInstance() {
        return m_instance;
    }

    public Translation3d getClosestGamePieceByType(GamePieceType gamePieceType){
        Translation3d gamePieceToRobot = new Translation3d();

        ArrayList<GamePieceCamera> detectedGamePieceCams = new ArrayList<>();
        ArrayList<PhotonTrackedTarget> closestGamePiecesOfWantedType = new ArrayList<>();
        for(GamePieceCamera cam : M_CAMS){
            if(cam.getGamePieces().getBestTarget() != null){
                detectedGamePieceCams.add(cam);
                closestGamePiecesOfWantedType.add(cam.getGamePieces().getBestTarget());
            }
        }

        if(closestGamePiecesOfWantedType.isEmpty())
            return new Translation3d(0,0,0);

        GamePieceCamera closestGamePieceCam = detectedGamePieceCams.get(0);
        PhotonTrackedTarget closestGamePiece = closestGamePiecesOfWantedType.get(0);
        for (PhotonTrackedTarget gamePiece : closestGamePiecesOfWantedType) {
            if(gamePiece.getArea() > closestGamePiece.getArea())
                closestGamePiece = gamePiece;
                closestGamePieceCam = detectedGamePieceCams.get(
                                       closestGamePiecesOfWantedType.indexOf(closestGamePiece));
        }
        
        switch(gamePieceType){
            case Coral:
                gamePieceToRobot = Funcs.calculateCoralTranslation();
                break;
            case Algea:
                gamePieceToRobot = Funcs.calculateAlgeaTranslation();
                break;
  
        }

        gamePieceToRobot.plus(closestGamePieceCam.getRobotToCam());
        
        return gamePieceToRobot;
        
    }


    /**
     * 
     * @return the gamepiece position relative to the robot center
     
    public Translation3d getClosestCoralTranslation3d() {
        Translation3d gamePieceToCam = new Translation3d();
        ArrayList<GamePieceCamera> coralCams = new ArrayList<GamePieceCamera>();
        for (GamePieceCamera cam : M_CAMS) {
            if (cam.getGamePieceType() == GamePieceType.Coral && cam.getGamePieces().hasTargets()) {
                coralCams.add(cam);
            }
        }
        if (coralCams.isEmpty())
            return gamePieceToCam;
        ArrayList<PhotonTrackedTarget> bestCameraCoralTargets  = new ArrayList<PhotonTrackedTarget>();
        for (GamePieceCamera cam : coralCams) {
            bestCameraCoralTargets.add(cam.getGamePieces().getBestTarget());
        }        
        PhotonTrackedTarget closestCoral = bestCameraCoralTargets.get(0);
        GamePieceCamera closestCoralCam = coralCams.get(0);
        for (PhotonTrackedTarget coral : bestCameraCoralTargets) {
            if(closestCoral.getArea() < coral.getArea()){
                closestCoral = coral;
                closestCoralCam = coralCams.get(bestCameraCoralTargets.indexOf(coral));
            }
        }
        TargetCorner[] coralCornersArr = Funcs.orgenizeTargets(closestCoral.getMinAreaRectCorners());
        double shortestSide = Funcs.getShortestSide(coralCornersArr);
        double[] coralCenter = Funcs.getRectCenter(coralCornersArr);
        double z = (closestCoralCam.getFocalX() * CORAL_RADIUS) / shortestSide;
        double x = ((coralCenter[0] - (closestCoralCam.getWidth() / 2)) * z) / closestCoralCam.getFocalX(); 
        double y = ((coralCenter[1] - (closestCoralCam.getHieght() / 2)) * z) / closestCoralCam.getFocalY(); 
        
        gamePieceToCam = new Translation3d(z,x,y);
        gamePieceToCam.rotateBy(new Rotation3d());//need to rotate by gyro rotation
        return closestCoralCam.getRobotToCam().plus(gamePieceToCam);
    }*/
}
