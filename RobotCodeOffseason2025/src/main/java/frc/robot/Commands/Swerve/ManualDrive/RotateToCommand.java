package frc.robot.Commands.Swerve.ManualDrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAngleController;

public class RotateToCommand extends Command{
    
    private double m_targetAngle;
    private boolean m_fieldOriented;
    int cnt = 0;

    public RotateToCommand(double targetAngle, boolean fieldOriented){
        m_targetAngle = targetAngle;
        m_fieldOriented = fieldOriented;
    }

    @Override
    public void initialize() {
        cnt++;
        SmartDashboard.putNumber("cnt", cnt);
        SwerveAngleController.getInstance().start(m_targetAngle, m_fieldOriented);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SwerveAngleController.getInstance().stop();
    }
}