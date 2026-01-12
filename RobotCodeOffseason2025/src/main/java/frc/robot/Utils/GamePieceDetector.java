package frc.robot.Utils;

import java.util.ArrayList;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Utils.GamePieceCamera.GamePieceType;

import frc.robot.Utils.Math.Funcs;
// TODO :
// write algorithm for calculating game pieace distance
// get real consts
// test
public class GamePieceDetector {
    public static final double CORAL_RADIUS = 0.114;
    public static final double ALGEA_RADIUS = 0.413/2.0;
    private static final GamePieceCamera[] M_CAMS = {
        new GamePieceCamera("Brio_100", GamePieceType.Algea, 
        new Transform3d(new Translation3d(0, 0, 0.765), new Rotation3d(0,-85,0)),
        640,37.4)
};
    private static GamePieceDetector m_instance = new GamePieceDetector();

    public GamePieceDetector() {

    }

    public static GamePieceDetector getInstance() {
        return m_instance;
    }

    public Pose3d getClosestGamePieceByTypeLocation(GamePieceType gamePieceType){
        Pose3d gamePieceToRobot = new Pose3d();

        ArrayList<GamePieceCamera> detectedGamePieceCams = new ArrayList<>();
        ArrayList<PhotonTrackedTarget> closestGamePiecesOfWantedType = new ArrayList<>();

        for(GamePieceCamera cam : M_CAMS){
            if(cam.getGamePieces() != null && cam.getGamePieceType() == gamePieceType){
                detectedGamePieceCams.add(cam);
                closestGamePiecesOfWantedType.add(cam.getGamePieces().getBestTarget());
            }
        }

        if(closestGamePiecesOfWantedType.isEmpty())
            return new Pose3d(new Translation3d(), new Rotation3d());

        GamePieceCamera closestGamePieceCam = detectedGamePieceCams.get(0);
        PhotonTrackedTarget closestGamePiece = closestGamePiecesOfWantedType.get(0);
        for (PhotonTrackedTarget gamePiece : closestGamePiecesOfWantedType) {
            if(gamePiece.getArea() > closestGamePiece.getArea())
                closestGamePiece = gamePiece;
                closestGamePieceCam = detectedGamePieceCams.get(
                                       closestGamePiecesOfWantedType.indexOf(closestGamePiece));
        }

        TargetCorner[] corners = Funcs.orgenizeTargets(closestGamePiece.getMinAreaRectCorners());
        double[] sides = Funcs.getRectSides(corners);
        double[] center = Funcs.getRectCenter(corners);

        gamePieceToRobot = Funcs.VisionCalculateGamePieceTranslation(center[0], center[1],sides[0], sides[1],
                                                                     closestGamePieceCam.getWidth(), closestGamePieceCam.getXFov(), GamePieceType.Algea);
        gamePieceToRobot.plus(closestGamePieceCam.getRobotToCam());
        
        return gamePieceToRobot;
        
    }

}
