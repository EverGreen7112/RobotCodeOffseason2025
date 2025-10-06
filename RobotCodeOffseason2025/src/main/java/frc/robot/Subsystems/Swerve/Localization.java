package frc.robot.Subsystems.Swerve;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.LocalizationCamera;
import frc.robot.Utils.EverKit.EverGyro;
import frc.robot.Utils.EverKit.Periodic;
import frc.robot.Utils.EverKit.Implementations.Gyros.EverNavX;

public class Localization implements Periodic {
    
    private static Localization m_instance;

    private SwerveDrivePoseEstimator m_poseEstimator;
    
    private AprilTagFieldLayout m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark) ;
    private static final double FIELD_WIDTH = 8.05;
    private static final double FIELD_HEIGHT = 17.55;
    private static final double MAX_ESTIMATION_HEIGHT = 0.08;
    private static final double MAX_DISTANCE_FROM_TAG = 3;


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


    private Localization() {

        Pose2d startingPose = new Pose2d(0 ,0, new Rotation2d());
        Translation2d pos[] = new Translation2d[4];
        for(int i = 0; i < 4; i++){
            pos[i] = new Translation2d(SwerveConsts.modulesPositions[i].x, SwerveConsts.modulesPositions[i].y);
        }
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(pos);
        m_poseEstimator = new SwerveDrivePoseEstimator(kinematics, Swerve.getInstance().getGyroRotation2d(), Swerve.getInstance().getModulesPositions(),startingPose);

        start(PeriodicTime.kRobotPeriodic);
    }

    
    public static Localization getInstance() {
        if (m_instance == null) {
            m_instance = new Localization();
        }
        return m_instance;
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return m_instance.m_poseEstimator;
    }

    @Override
    public void periodic() {
        m_poseEstimator.update(Swerve.getInstance().getGyroRotation2d(), Swerve.getInstance().getModulesPositions());

        for(LocalizationCamera cam : CAMS){
            addCameraVision(cam);
            
        }
        log();
    }

    public void addCameraVision(LocalizationCamera cam) {
        Optional<EstimatedRobotPose> est = cam.getEstimatedGlobalPose();
        if (!takeVisionPoseEstimation(est))
            return;
        m_poseEstimator.addVisionMeasurement(est.get().estimatedPose.toPose2d(), est.get().timestampSeconds,
                cam.getEstimationStdDevs());
    }

    private boolean takeVisionPoseEstimation(Optional<EstimatedRobotPose> est) {
        if (!est.isPresent() || est == null){
            return false;
        }

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

        SmartDashboard.putBoolean("OUT OF FIELD", outOfField);
        SmartDashboard.putBoolean("ABOVE CAMERA", aboveCamera);
        SmartDashboard.putBoolean("UNDERGROUND", underGround);
        SmartDashboard.putBoolean("IS TOO FAR", isTooFar);
        SmartDashboard.putNumber("x", x);
        SmartDashboard.putNumber("y", y);
        SmartDashboard.putNumber("z", z);



        return !outOfField && !aboveCamera && !underGround && !isTooFar;
    }

    private void log(){
        
        SmartDashboard.putNumber("Pose X", m_poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Pose Y", m_poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Pose Rotation", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());

    }

    public double getFieldOrientedAngle() {
        return m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    }

    public Pose2d getCurrentPoint() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void setCurrentPoint(Pose2d newPoint) {
        m_poseEstimator.resetPosition(
                Swerve.getInstance().getGyroRotation2d(),
                Swerve.getInstance().getModulesPositions(),
                newPoint);
    }

}
