package frc.robot.Subsystems.Localiztion;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Utils.EverKit.EverGyro;
import frc.robot.Utils.EverKit.Periodic;
import frc.robot.Utils.EverKit.Implementations.Gyros.EverNavX;

public class Localiztion extends SubsystemBase {
    

    private static Localiztion m_instance;
    private Vision m_vision;
    private Odomatry m_odometry;

    private SwerveDrivePoseEstimator m_poseEstimator; 

    private Localiztion() {
        
    }

    public static Localiztion getInstance() {
        if (m_instance == null) {
            m_instance = new Localiztion();
        }
        return m_instance;
    }

    public static SwerveDrivePoseEstimator getPoseEstimator() {
        return m_instance.m_poseEstimator;
    }

    @Override
    public void periodic() {
        m_odometry.getRelativePosition();
        m_vision.getEstimatedPose();
    }


}
