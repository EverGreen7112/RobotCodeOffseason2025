package frc.robot.Subsystems.Localiztion;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Utils.Math.Vector2d;
import frc.robot.Utils.DeltaTime;

public class Odomatry extends SubsystemBase {
    

    private static Odomatry m_instance;
    private Vector2d m_relativePosition; // Relative position of the robot in the field
    private double m_deltaTime; // Time step for periodic updates, can be adjusted based on your needs

    private Odomatry() {
        m_relativePosition = new Vector2d(0, 0); // Initialize the relative position to (0, 0)
        m_deltaTime = DeltaTime.getDeltaTime(); // Get the initial delta time
    }

    public static Odomatry getInstance() {
        if(m_instance == null) {
            m_instance = new Odomatry();
        }
        return m_instance;
    }

    public Pose2d getRelativePosition(){
        double theta = m_relativePosition.theta();
        Rotation2d rotation = new Rotation2d(Math.toRadians(theta));
        return new Pose2d(m_relativePosition.x, m_relativePosition.y, rotation);
    }

    public void reset(){
        m_relativePosition = new Vector2d(0,0);
    }

    @Override
    public void periodic() {
        
        m_deltaTime = DeltaTime.getDeltaTime(); // Get the time since the last update
        Vector2d current = Swerve.getInstance().getGyroOrientedVelocity();
        m_relativePosition = m_relativePosition.add(current.mul(m_deltaTime)); // not using that shit but nice to have 🤷‍♂️

        Localiztion.getPoseEstimator().update(Swerve.getInstance().getGyroRotation2d(),Swerve.getInstance().getModulesPositions());
    }

}
