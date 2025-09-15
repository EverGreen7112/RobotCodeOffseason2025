// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Swerve.ManualDrive.TeleopDriveCommand;
import frc.robot.Subsystems.Swerve.Swerve;

public class RobotContainer {

  private static final int CHASSIS_PORT = 0;

  //controllers
  public static final CommandXboxController chassis = new CommandXboxController(CHASSIS_PORT);

  //Triggers
  public static final Trigger chassisStart = chassis.start();
  public static final Trigger chassisBack = chassis.back();
  public static final Trigger chassisA = chassis.a();
  public static final Trigger chassisB = chassis.b();
  public static final Trigger chassisX = chassis.x();
  public static final Trigger chassisY = chassis.y();
  public static final Trigger chassisRT = chassis.rightTrigger();
  public static final Trigger chassisLT = chassis.leftTrigger();
  public static final Trigger chassisRB = chassis.rightBumper();
  public static final Trigger chassisLB = chassis.leftBumper();
  public static final Trigger chassisPovUp = chassis.povUp();
  public static final Trigger chassisPovDown = chassis.povDown();

  //commands
  public static final Trigger chassisPovRight = chassis.povRight();
  public static final Trigger chassisPovLeft = chassis.povLeft();
  public static final TeleopDriveCommand teleopCommand = new TeleopDriveCommand(chassis::getLeftX, chassis::getLeftY, chassis::getRightX);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    //chassis
    Swerve.getInstance().setDefaultCommand(teleopCommand);
  }

  
}
