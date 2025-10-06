
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.LedStrip;
import frc.robot.Subsystems.Swerve.Localization;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Utils.ReefFace;
import frc.robot.Utils.RobotOperatorController;
import frc.robot.Utils.EverKit.Periodic;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverExternalMotorPIDController;

public class Robot extends TimedRobot {

  public static ArrayList<Periodic> robotPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> teleopPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> testPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> autonomousPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> simulationPeriodicFuncs = new ArrayList<Periodic>();

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Localization m_Localizer = Localization.getInstance();
  private RobotOperatorController robotOperatorController = new RobotOperatorController();
  private Field2d m_field = new Field2d();




  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    SwerveAutoController.getInstance().addChoosersToDashboard();
    SmartDashboard.putData("Field", m_field);
    LedStrip.getInstance().initialize();

      SmartDashboard.putString("right branch" + 0, ReefFace.RED_REEF[0].getRightBranchRobotPose().toString());
      SmartDashboard.putString("right branch" + 1, ReefFace.RED_REEF[1].getRightBranchRobotPose().toString());
      SmartDashboard.putString("right branch" + 2, ReefFace.RED_REEF[2].getRightBranchRobotPose().toString());
      SmartDashboard.putString("right branch" + 3, ReefFace.RED_REEF[3].getRightBranchRobotPose().toString());
      SmartDashboard.putString("right branch" + 4, ReefFace.RED_REEF[4].getRightBranchRobotPose().toString());
      SmartDashboard.putString("right branch" + 5, ReefFace.RED_REEF[5].getRightBranchRobotPose().toString());

      SmartDashboard.putString("left branch" + 0, ReefFace.RED_REEF[0].getLeftBranchRobotPose().toString());
      SmartDashboard.putString("left branch" + 1, ReefFace.RED_REEF[1].getLeftBranchRobotPose().toString());
      SmartDashboard.putString("left branch" + 2, ReefFace.RED_REEF[2].getLeftBranchRobotPose().toString());
      SmartDashboard.putString("left branch" + 3, ReefFace.RED_REEF[3].getLeftBranchRobotPose().toString());
      SmartDashboard.putString("left branch" + 4, ReefFace.RED_REEF[4].getLeftBranchRobotPose().toString());
      SmartDashboard.putString("left branch" + 5, ReefFace.RED_REEF[5].getLeftBranchRobotPose().toString());

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    for (Periodic method : robotPeriodicFuncs) {
      try {
        method.periodic();

      } catch (Exception e) {
        e.printStackTrace();
      }
    }

    m_field.setRobotPose(Localization.getInstance().getCurrentPoint());

    LedStrip.getInstance().periodic();

  } 

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = SwerveAutoController.getInstance().getAutoCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    for (Periodic method : autonomousPeriodicFuncs) {
      try {
        method.periodic();

      } catch (Exception e) {
        e.printStackTrace();
      }
    }

    
  }
  @Override
  public void autonomousExit() {}
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Swerve.getInstance().setGyroOffset(Swerve.getInstance().getHeadingDegree());
  }

  @Override
  public void teleopPeriodic() { 
    for (Periodic method : teleopPeriodicFuncs) {
      try {
        method.periodic();
      } catch (Exception e) {
        e.printStackTrace();
      }
      }   
      
    
    }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    for (Periodic method : testPeriodicFuncs) {
      try {
        method.periodic();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  @Override
  public void testExit() {}  

  @Override
  public void simulationPeriodic() {
    for (Periodic method : simulationPeriodicFuncs) {
      try {
        method.periodic();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }


  
  
}