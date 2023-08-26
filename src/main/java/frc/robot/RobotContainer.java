// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import Common.ArcadeControls;
import Common.LogitechF310;
import Common.ThrustMaster;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Drive.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class RobotContainer {

  // Create controller objects
  public static final ThrustMaster driveJoy = new ThrustMaster(Constants.Controls.DRIVE_CONTROLLER);
  public static final LogitechF310 gamepad = new LogitechF310(Constants.Controls.GAMEPAD);
  public static final ArcadeControls arcade = new ArcadeControls(Constants.Controls.ARCADE);
  
  //Event map for path following
  HashMap <String, Command> eventMap = new HashMap<String, Command>();

  //Instantiate Subsystems
  DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  public RobotContainer() {
    CommandScheduler.getInstance().registerSubsystem(drivetrain);

    //NOTE: Mapping is now done in DefualtDriveCommand
    drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, this::getXInput, this::getYInput, this::getZInput));

    configureBindings(); 
    configureEventMap();
  }

  private void configureBindings() {

  // ======================= Button Commands ============================
  // Joysticks
    driveJoy.getLeftFrontMiddle().onTrue(Commands.runOnce(drivetrain::resetOdometry, drivetrain));
    driveJoy.getLeftFrontRight().onTrue(Commands.runOnce(drivetrain::resetEncoders, drivetrain));
  }

  //==================== Path Following Event Map ========================
  private void configureEventMap() {}

  //Factory for a command that executes a path group with events
  //Path groups break up paths into sections between which events
  //can be executed without conflict. This builds a command handling
  //this logic.
  public SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    drivetrain::getPose,
    drivetrain::resetOdometry,
    new PIDConstants(1.5, 0.0, 0.0),
    new PIDConstants(0.5, 0.0, 0.0),
    drivetrain::driveFieldRelative,
    eventMap,
    drivetrain
  );

  //Factory for a command that follows a trajectory.
  public Command followTrajectory(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
         new InstantCommand(() -> {
           // Reset odometry for the first path you run during auto
           if(isFirstPath){
               drivetrain.resetOdometry(traj.getInitialHolonomicPose());
           }
         }),
         new PPSwerveControllerCommand(
          traj,
          drivetrain::getPose,
          new PIDController(1.5, 0.0, 0.0),
          new PIDController(1.5, 0.0, 0.0),
          new PIDController(0.5, 0.0, 0.0),
          drivetrain::driveFieldRelative,
          drivetrain
         )
     );
 }

  public void removeNotUsedError() {
  }

  public DrivetrainSubsystem getDrivetrain() {
    return drivetrain;
  }
  
  public ThrustMaster getDriveJoy() {
    return driveJoy;
  }

  public LogitechF310 getGamepad() {
    return gamepad;
  }

  public ArcadeControls getArcade() {
    return arcade;
  }

  private static double deadband(double value, double tolerance) {
    if (Math.abs(value) < tolerance)
        return 0.0;

    return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
  }

  private static double square(double value) {
    return Math.copySign(value * value, value);
  }

  public double getXInput() {
    return -square(deadband(driveJoy.getY(), 0.1)) * Constants.Swerve.maxSpeed;
  }

  public double getYInput() {
      return -square(deadband(driveJoy.getX(), 0.1)) * Constants.Swerve.maxSpeed;
  }

  public double getZInput() {
      return -square(deadband(driveJoy.getZ(), 0.1))
              * Constants.Swerve.maxAngularVelocity;
  }
}
