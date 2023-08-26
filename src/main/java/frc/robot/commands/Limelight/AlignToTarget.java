// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import java.util.function.DoubleSupplier;

import Common.Limelight;
import Common.Target;
import Common.Target.DetectionType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AlignToTarget extends CommandBase {

  DrivetrainSubsystem drivetrain;
  Vision vision;
  Limelight limelight;
  Target target;

  PIDController pidForward;
  PIDController pidStrafe;
  PIDController pidRotate;

  DoubleSupplier strafeAxis;

  public AlignToTarget(DrivetrainSubsystem drivetrain, Vision vision, Limelight limelight, Target target, DoubleSupplier strafeAxis) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.vision = vision;
    this.target = target;
    this.strafeAxis = strafeAxis;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    pidForward.setPID(0.05, 0, 0);
    pidForward.setTolerance(target.getAlignmentTolerance(), Constants.Swerve.maxSpeed);

    pidStrafe.setPID(0.05, 0, 0);
    pidStrafe.setTolerance(target.getAlignmentTolerance(), Constants.Swerve.maxSpeed);

    pidRotate.setPID(0.05, 0, 0);
    pidRotate.setTolerance(target.getAlignmentTolerance(), Constants.Swerve.maxAngularVelocity);

    if (target.getDetectionType().equals(DetectionType.Reflective)) {
      // Switch Pipeline to Reflective Tracking 
    } else if (target.getDetectionType().equals(DetectionType.AprilTag)) {
      // Switch Pipeline to AprilTag Tracking
    } else if (target.getDetectionType().equals(DetectionType.GamePiece)) {
      // Switch Pipeline to GamePiece Tracking
    }

  }

  @Override
  public void execute() {

    drivetrain.drive(new ChassisSpeeds(
      pidForward.calculate(limelight.getVerticalError(), 0),
      strafeAxis.getAsDouble(),
      pidRotate.calculate(limelight.getHorizontalError(), 0)
    ));

  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public boolean isFinished() {
    return (pidForward.atSetpoint() && pidRotate.atSetpoint());
  }
}
