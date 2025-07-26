package frc.robot.Utils;

import java.lang.annotation.Documented;
import java.util.ArrayList;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Utils.GamePieceCamera.GamePieceType;
import frc.robot.Utils.Math.Funcs;

public class GamePieceDetector {
    private static final double CORAL_RADIUS = 1;
    private static final GamePieceCamera[] M_CAMS = {};
    private static GamePieceDetector m_instance = new GamePieceDetector();

    public GamePieceDetector() {

    }

    public static GamePieceDetector getInstance() {
        return m_instance;
    }

    /**
     * 
     * @return the gamepiece position relative to the robot center
     */
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
    }
}
