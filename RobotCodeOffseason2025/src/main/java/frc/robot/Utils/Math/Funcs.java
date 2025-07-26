package frc.robot.Utils.Math;

import java.util.List;

import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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

    public static double getShortestSide(TargetCorner[] cornersArr){
        double verticalSide = Math.abs(cornersArr[1].y - cornersArr[2].y);
        double horizontalSide = Math.abs(cornersArr[0].x - cornersArr[1].x);
        return (verticalSide <= horizontalSide) ? verticalSide : horizontalSide;
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

}