package frc.robot.Utils.Math;

import java.lang.annotation.Target;
import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.GamePieceCamera;
import frc.robot.Utils.GamePieceDetector;
import frc.robot.Utils.GamePieceCamera.GamePieceType;

public class Funcs {

    /**
     * @param vec - vector in the standard 2d axes (positive y is forward positive x is right)
     * @return the vector in wpilib's axes, NWU - positive X is forward positive Y is left
     */

    public static Vector2d convertFromStandardAxesToWpilibs(Vector2d vec){
        Vector2d tmp = new Vector2d(vec).rotate(Math.toRadians(90));
        return tmp;
    }

    public static double getShortestAnglePath(double a, double b) {
        // get direction
        double dir = modulo(b, 360.0) - modulo(a, 360.0);

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0) {
            dir = -(Math.signum(dir) * 360.0) + dir;
        }
        return dir;
    }

    public static double modulo(double a, double b) {
        return ((a % b) + b) % b;
    }

    public static double roundAfterDecimalPoint(double num, int amount) {
        num *= Math.pow(10, amount);
        num = (int) num;
        num /= Math.pow(10, amount);
        return num;
    }

    public static double convertRotationsToDegrees(double rotations){
        //convert rotations to degrees
        rotations *= 360;
       
        //convert from -180 - 180 to 0 - 360 
        if(rotations < 0){
            rotations += 360;
        }
        return rotations;
    }

    public static Rotation2d degreesToRotation2d(double degrees){
        return new Rotation2d(Math.toRadians(degrees));
    }

    public static double getDis(Pose2d first, Pose2d second){
        return (first.minus(second)).getTranslation().getNorm();
    }

    /**
     * 
     * @param 
     * photon vision list of corners
     * @return
     * corners orgenized in the order tL-> tR ->bR ->bL
     */
    
    public static TargetCorner[] orgenizeTargets(List<TargetCorner> corners){
        TargetCorner[] arr = new TargetCorner[4];

        arr[0] = corners.get(0);
        for(TargetCorner corner : corners){
            if(corner.x < arr[0].x && corner.y < arr[0].y){
                arr[0] = corner;
            }
        }

        for(TargetCorner corner : corners){
            if(arr[0].y == corner.y && arr[0].x != corner.x){
                arr[1] = corner;
            }
        }

        for(TargetCorner corner: corners){
            if(arr[0].x != corner.x && arr[1].y != corner.y){
                arr[2] = corner;
            }
        }

        for(TargetCorner corner: corners){
            if(arr[2].y == corner.y && arr[2].x != corner.x){
                arr[3] = corner;
            }
        }

        return arr;
    }

    public static double[] getRectSides(TargetCorner[] cornersArr){
        double[] sides = new double[2];
        sides[0] = Math.abs(cornersArr[0].x - cornersArr[1].x);
        sides[1] = Math.abs(cornersArr[1].y - cornersArr[2].y);
        return sides;
    }
    
    /**
     * return the center of the rectengle 0 - x, 1 - y
     */
    public static double[] getRectCenter(TargetCorner[] cornersArr){
        double x = (Math.abs(cornersArr[0].x - cornersArr[1].x)) / 2;
        double y = (Math.abs(cornersArr[1].y - cornersArr[2].y)) / 2;
        double[] center = new double[2];
        center[0] = x;
        center[1] = y;
        return center;
    }
    /**
     * 
     * @return hFOV index 0 vFOV index 1 - 16:9 ratio
     */
    // public static double[] diagFovToHorizontalAndVertialFov(double diagFOV){
    //     double[] FovArr = new double[2];
    //     FovArr[0] = 2 * Math.atan2(Math.tan(Math.toRadians(diagFOV / 2)) * 16,Math.sqrt(Math.pow(16, 2) + Math.pow(9, 2)));
    //     FovArr[1] = 2 * Math.atan2(Math.tan(FovArr[0] / 2) * 9, 16);
    //     return FovArr;
    // }

    // public static double FovToFocalLenght(double fov, double pxLenght){
    //     return pxLenght / ( 2 * Math.tan(fov / 2));
    // }

    // /**
    //  * Todo
    //  * @return
    //  */
    // public static Translation3d calculateCoralTranslation(){
    //     return null;
    // }

    /**
     * Todo
     * @return
     */
    // public static Translation3d TrigCalculateGamePieceTranslation(GamePieceCamera closestGamePieceCam, PhotonTrackedTarget closestGamePiece, GamePieceType gamePieceType){
    //     Translation3d gamePieceToCam;

    //     double y = closestGamePieceCam.getRobotToCam().getZ();
    //     switch(gamePieceType){
    //         case Coral:
    //             y -= GamePieceDetector.CORAL_RADIUS / 2;
    //             break;
    //         case Algea:
    //             y -= GamePieceDetector.ALGEA_RADIUS / 2;
    //             break;
  
    //     }

    //     double alpha = (Math.PI/2 - Math.abs(closestGamePieceCam.getRobotToCam().getRotation().getY())) + Math.toRadians(closestGamePiece.getPitch());
    //     double a = y * Math.tan(alpha);
    //     SmartDashboard.putNumber("hyp",a);
    //     double beta = Math.abs(closestGamePiece.getYaw());
    //     double x = a * Math.sin(Math.toRadians(beta));
    //     double z = a * Math.cos(Math.toRadians(beta));
    //     gamePieceToCam = new Translation3d(z,x,y);

    //     return gamePieceToCam;
    // }

    public static Translation3d VisionCalculateGamePieceTranslation(double centerX, double centerY, double pixelX, double pixelY,
                                                                    double resX, double XFov){

        double plainX = 2.0 * GamePieceDetector.ALGEA_RADIUS / (pixelX / resX);
        double z = plainX / (2.0 * Math.tan(Math.toRadians(XFov) / 2.0));
        double x = (centerX - 640 / 2) * plainX / resX;//(2.0 * GamePieceDetector.ALGEA_RADIUS * centerX) / pixelX;
        double y = (2.0 * GamePieceDetector.ALGEA_RADIUS * (centerY - 480 / 2)) / pixelY;

        SmartDashboard.putNumber("vision - x", x);
        SmartDashboard.putNumber("vision - y", y);
        SmartDashboard.putNumber("vision - z", z);
        SmartDashboard.putNumber("abs", Math.sqrt(Math.pow(x, 2) + 
                                                      Math.pow(y, 2) + 
                                                      Math.pow(z, 2)));

        return new Translation3d(z, x, y);
    }

}