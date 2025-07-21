package frc.robot.Utils;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Utils.GamePieceCamera.GamePieceType;

public class GamePieceDetector {
    private static final GamePieceCamera[] M_CAMS = {};
    private static GamePieceDetector m_instance = new GamePieceDetector();

    public GamePieceDetector() {

    }

    public static GamePieceDetector getInstance() {
        return m_instance;
    }

    public Pose2d getClosestCoralPose2d() {
        List<GamePieceCamera> coralCams;
        for (GamePieceCamera cam : M_CAMS) {
            if (cam.getGamePieceType() == GamePieceType.Coral && cam.getGamePieces().hasTargets()) {
                coralCams.add(cam);
            }
        }
        if (coralCams.isEmpty())
            return null;
        List<PhotonTrackedTarget> bestCameraCoralTargets;
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
    }
}
