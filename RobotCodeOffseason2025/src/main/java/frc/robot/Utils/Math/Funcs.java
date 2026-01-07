package frc.robot.Utils.Math;

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
        TargetCorner[] cornersArr = new TargetCorner[4];
        cornersArr[0] = corners.get(0); 
        for (TargetCorner Corner : corners) {
            if(Corner.x < cornersArr[0].x && Corner.y < cornersArr[0].y){
                cornersArr[0] = Corner;
            }
        }
        cornersArr[1] = corners.get(0); 
        for (TargetCorner Corner : corners) {
            if(Corner.x > cornersArr[1].x && Corner.y < cornersArr[0].y){
                cornersArr[1] = Corner;
            }
        }

        cornersArr[2] = corners.get(0); 
        for (TargetCorner Corner : corners) {
            if(Corner.x > cornersArr[2].x && Corner.y > cornersArr[0].y){
                cornersArr[2] = Corner;
            }
        }

        cornersArr[3] = corners.get(0); 
        for (TargetCorner Corner : corners) {
            if(Corner.x < cornersArr[3].x && Corner.y > cornersArr[0].y){
                cornersArr[3] = Corner;
            }
        }
        return cornersArr;
    }

    public static double getLongestSide(TargetCorner[] cornersArr){
        double verticalSide = Math.abs(cornersArr[1].y - cornersArr[2].y);
        double horizontalSide = Math.abs(cornersArr[0].x - cornersArr[1].x);
        return (verticalSide >= horizontalSide) ? verticalSide : horizontalSide;
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
    public static Translation3d calculateAlgeaTranslation(double focalX,double focalY,double width,
                                                          double hieght, double algeaDiameter, 
                                                          double algeaX, double algeaY){

        double z = (focalX * GamePieceDetector.ALGEA_RADIUS) / algeaDiameter;
        double x = (algeaX - (width / 2))  * (z / focalX); 
        double y = (algeaY - (hieght / 2))  * (z / focalY);

        return new Translation3d(z,x,y);
    }
    public static Translation3d TrigCalculateGamePieceTranslation(GamePieceCamera closestGamePieceCam, PhotonTrackedTarget closestGamePiece, GamePieceType gamePieceType){
        Translation3d gamePieceToCam;

        double y = closestGamePieceCam.getRobotToCam().getZ();
        switch(gamePieceType){
            case Coral:
                y -= GamePieceDetector.CORAL_RADIUS / 2;
                break;
            case Algea:
                y -= GamePieceDetector.ALGEA_RADIUS / 2;
                break;
  
        }

        double alpha = (Math.PI/2 - Math.abs(closestGamePieceCam.getRobotToCam().getRotation().getY())) + Math.toRadians(closestGamePiece.getPitch());
        double a = y * Math.tan(alpha);
        SmartDashboard.putNumber("hyp",a);
        double beta = Math.abs(closestGamePiece.getYaw());
        double x = a * Math.sin(Math.toRadians(beta));
        double z = a * Math.cos(Math.toRadians(beta));
        gamePieceToCam = new Translation3d(z,x,y);

        return gamePieceToCam;
    }

    public static Translation3d VisionCalculateGamePieceTranslation(double centerX, double centerY, double pixelY, double focalY, double focalX, double resX, double resY,
                                                                    double yFov, GamePieceType gamePieceType){
        double hieght = 0;
        switch(gamePieceType){
            case Coral:
                hieght = GamePieceDetector.CORAL_RADIUS;
                break;
            case Algea:
                hieght = GamePieceDetector.ALGEA_RADIUS;
        }

        double thetaY = (pixelY / resY) * yFov;
        double z = (hieght / 2) / (Math.tan(thetaY / 2));
        double x = ((centerX - (resX / 2)) * z) / focalX;
        double y = ((centerY - (resY / 2)) * z) / focalY;

        return new Translation3d(z, x, y);
    }

}