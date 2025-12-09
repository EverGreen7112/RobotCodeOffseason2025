package frc.robot.Utils;

import java.lang.annotation.Documented;
import java.util.ArrayList;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.GamePieceCamera.GamePieceType;

import frc.robot.Utils.Math.Funcs;
// TODO :
// write algorithm for calculating game pieace distance
// get real consts
// test
public class GamePieceDetector {
    public static final double CORAL_RADIUS = 0.114;
    public static final double ALGEA_RADIUS = 0.413;
    private static final GamePieceCamera[] M_CAMS = {
        new GamePieceCamera("Brio_100", GamePieceType.Algea, 
        new Transform3d(new Translation3d(0, 0, 0.765), new Rotation3d(0,-85,0)))
};
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
            if(cam.getGamePieces() != null && cam.getGamePieceType() == gamePieceType){
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

        double y = closestGamePieceCam.getRobotToCam().getZ();
        switch(gamePieceType){
            case Coral:
                y -= CORAL_RADIUS / 2;
                break;
            case Algea:
                y -= ALGEA_RADIUS / 2;
                break;
  
        }

        double alpha = closestGamePieceCam.getRobotToCam().getRotation().getY() + Math.toRadians(closestGamePiece.getPitch());
        double hyp = y / Math.tan(alpha);
        SmartDashboard.putNumber("hyp",hyp);
        double beta = Math.abs(closestGamePiece.getYaw());
        double x = hyp * Math.sin(Math.toRadians(beta));
        double z = hyp * Math.cos(Math.toRadians(beta));
        gamePieceToRobot = new Translation3d(z,x,y);

        //gamePieceToRobot.plus(closestGamePieceCam.getRobotToCam().getTranslation());
        
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
