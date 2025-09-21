package frc.robot.Subsystems.Localiztion;

import java.security.Timestamp;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.w3c.dom.views.DocumentView;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Utils.LocalizationCamera;
import frc.robot.Utils.EverKit.EverGyro;
import frc.robot.Utils.EverKit.Implementations.Gyros.EverNavX;
import frc.robot.Utils.Math.Vector2d;

public class Vision extends SubsystemBase {

    private static Vision m_instance;
    private static final LocalizationCamera[] CAMS = {
            new LocalizationCamera("left_cam",
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
                    new Transform3d(new Translation3d(0.115, 0.055, 0.32), new Rotation3d(0, 0 ,0)),
                    VecBuilder.fill(0.0, 0.0, 0), VecBuilder.fill(0.0, 0.0, 0)),
            new LocalizationCamera("right_cam",
                                        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
                                        new Transform3d(0.115, -0.155, 0.32, new Rotation3d(0, 0, 0)),
                                        VecBuilder.fill(0, 0, 0), VecBuilder.fill(0, 0, 0)),
            /*new LocalizationCamera("back_cam",
                                        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
                                        new Transform3d(-0.27985, -0.295, 0.56, new Rotation3d(Math.toRadians(-1), Math.toRadians(-44), Math.toRadians(180))), //new Rotation3d(Math.toRadians(25), Math.toRadians(1.4), Math.toRadians(180))
                                        VecBuilder.fill(0, 0, 0), VecBuilder.fill(0, 0, 0))*/
    };

    private static final double FIELD_WIDTH = 8.05;
    private static final double FIELD_HEIGHT = 17.55;
    private static final double MAX_ESTIMATION_HEIGHT = 0.08;
    private static final double MAX_DISTANCE_FROM_TAG = 3;

    private double m_estRotation = 0.0, m_gyroAbs = 0.0; // Rotation estimation from the vision system, and gyro to abduction angle offset
    private AprilTagFieldLayout m_fieldLayout;
    private ArrayList<LocalizationCamera> m_cams;
    private Pose2d m_estimatedPose = new Pose2d();
    private double m_timeStamp = 0.0; // Timestamp of the last pose estimation

    private Consumer<LocalizationCamera> m_abdPostionConsumer = cam -> {
        Optional<EstimatedRobotPose> est = cam.getEstimatedGlobalPose();
        if (!takeVisionPoseEstimation(est)) {
            return;
        }
        m_estimatedPose = est.get().estimatedPose.toPose2d();
        m_timeStamp = est.get().timestampSeconds;
        Localiztion.getPoseEstimator().addVisionMeasurement(m_estimatedPose, m_timeStamp, cam.getEstimationStdDevs());
        
    };
    
    
    private Consumer<Double> m_rotatiConsumer = rotation -> {
        m_gyroAbs += (m_gyroAbs + Swerve.m_instance.getGyroOrientedAngle()) / 2; // Average the gyro readings to smooth out the estimation
        m_estRotation = ((m_estimatedPose.getRotation().getDegrees() - m_gyroAbs) + rotation) / 2 ; // Average the last rotations to smooth out the estimation
        Swerve.getInstance().setGyroOffset(m_gyroAbs); 
    }; 

    private Vision() {
        m_cams = new ArrayList<>(Arrays.asList(CAMS));
        m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    }

    public static Vision getInstance() {
        if (m_instance == null) {
            m_instance = new Vision();
        }
        return m_instance;
    }

    public Pose2d getEstimatedPose() {
        return m_estimatedPose;
    }

    public double getEstimatedRotation() {
        return m_estRotation;
    }   

    @Override
    public void periodic() {
        for (LocalizationCamera cam : m_cams) {
            m_abdPostionConsumer.accept(cam);
        }
        m_rotatiConsumer.accept(m_estRotation);
        SmartDashboard.putNumber("x", 1);
        SmartDashboard.putNumber("y", m_estimatedPose.getY());
    }

    
    private boolean takeVisionPoseEstimation(Optional<EstimatedRobotPose> est) {
        if (!est.isPresent() || est == null)
            return false;

        Pose2d estPos = est.get().estimatedPose.toPose2d();
        double x = estPos.getX();
        double y = estPos.getY();
        double z = est.get().estimatedPose.getZ();

        boolean outOfField = x < 0.0 || x > FIELD_HEIGHT || y < 0.0 || y > FIELD_WIDTH;
        boolean aboveCamera = z > MAX_ESTIMATION_HEIGHT;
        boolean underGround = z < -0.03;

        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an
        // average-distance metric
        for (var tgt : est.get().targetsUsed) {
            var tagPose = m_fieldLayout.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(est.get().estimatedPose.toPose2d().getTranslation());
        }
        avgDist /= numTags;

        boolean isTooFar = avgDist > MAX_DISTANCE_FROM_TAG;
        return !outOfField && !aboveCamera && !underGround && !isTooFar;
    }
    
}
