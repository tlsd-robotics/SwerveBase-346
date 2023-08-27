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
  Limelight limelight;
  Target target;

  PIDController pidForward;
  PIDController pidStrafe;
  PIDController pidRotate;

  DoubleSupplier strafeAxis;

  public AlignToTarget(DrivetrainSubsystem drivetrain, Limelight limelight, Target target, DoubleSupplier strafeAxis) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.target = target;
    this.strafeAxis = strafeAxis;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {

    // ======================== Create PID Controller Objects and Configure Values and Tolerances ========================

    pidForward.setPID(0.05, 0, 0);
    pidForward.setTolerance(target.getAlignmentTolerance(), Constants.Swerve.maxSpeed);

    pidStrafe.setPID(0.05, 0, 0);
    pidStrafe.setTolerance(target.getAlignmentTolerance(), Constants.Swerve.maxSpeed);

    pidRotate.setPID(0.05, 0, 0);
    pidRotate.setTolerance(target.getAlignmentTolerance(), Constants.Swerve.maxAngularVelocity);

    // ========================= Set Correct Limelight Pipeline According to Target Detection Type ===================

    /* 
    This is a general purpose targeting function, hense why it supports all 3 of the standard targets.
    Compared to seperate more specialized functions, this will lack customization and precision, but should
    work as a baseline test and template for how a targetting command is layed out.
    */

    if (target.getDetectionType().equals(DetectionType.Reflective)) {
      Vision.setPipeline(limelight, Vision.Pipelines.getReflectivePipeline());

    } else if (target.getDetectionType().equals(DetectionType.AprilTag)) {
      Vision.setPipeline(limelight, Vision.Pipelines.getApriltagPipeline());

    } else if (target.getDetectionType().equals(DetectionType.GamePiece)) {
      Vision.setPipeline(limelight, Vision.Pipelines.getGamepicePL());
    }

  }

  /*
  Everything found in execute will "execute" every single cycle, which is 50 times a second, or 20ms. If something only needs to run once, put it
  in initalize. If it runs more than once, such as when held buttons, put it in execute. 
  */

  @Override
  public void execute() {

    /* 
    If target is found, set the drivetrain's drive command to the error in degrees * the constant values declared above.
    The larger the error, the faster it will move, smaller will be slower. This means as it appoaches the setpoint, it will
    decrease in speed. This speed can be changed via the kP, or proportional constant, declared above. A good test is take the largest error
    you may find, say 100 degrees, and multiply it by your kP, or in this case 0.05. Though the velpcity is capped using the set velocity tolerance
    parameter, you want to keep the result of that math to a low value. Our max speed is 4.5, so if you see a 10 or 50, decrease your kP. Same goes
    for if it is too slow. If you have read the comment regarding deadbands in RobotContainer, you will know of motors overcomming friction. When your kP 
    value is too low, your motor speed will get so small that the motors cannot move anymore. Which will make you miss your target and burn up your motors. Both 
    bad outcomes. Gotta find a sweet spot.

    Unless you know what you are doing or found a really good tutorial somewhere, I would avoid touching kI and kD, those being integral and derivitive constants.
    A lot more calculus level math to determine those things.

    Always tune kP to the utmost limit before touching those. 
    */

    if (limelight.getIsTargetFound()) {

      drivetrain.drive(new ChassisSpeeds(
        pidForward.calculate(limelight.getVerticalError(), 0),
        strafeAxis.getAsDouble(),
        pidRotate.calculate(limelight.getHorizontalError(), 0)
      ));

    }
  }

  /*
  When you end a driving command of any kind. Please for the love of god include the drive function and set it to zero. If not, those motors will run infinitely until it
  breaks or gets disabled. I know from experience. Do not forget this section.
   */

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    Vision.setPipeline(limelight, Vision.Pipelines.getDefaultPipeline());
  }



  @Override
  public boolean isFinished() {
    return (pidForward.atSetpoint() && pidRotate.atSetpoint());
  }
}
