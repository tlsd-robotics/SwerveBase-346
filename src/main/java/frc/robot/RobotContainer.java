package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import Common.ThrustMaster;
import Common.Utilities;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Drive.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {

  HashMap <String, Command> eventMap = new HashMap<String, Command>();

//==============================================================================
//======================== Create Subsystem Instance ===========================
  private final static DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final Superstructure superstructure = new Superstructure();

  private final ThrustMaster driveJoy = new ThrustMaster(Constants.Controls.DRIVE_CONTROLLER);

  public RobotContainer() {
      drivetrain.register();

      drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, this::getForwardInput, this::getStrafeInput, this::getRotationInput, this::getThrottleInput));

      configureBindings();
      configureEventMap();
  }


//==============================================================================
//=============================== Getter Methods ===============================
  public static DrivetrainSubsystem getDrivetrain() {
    return drivetrain;
  }

  public ThrustMaster getDriveJoystick() {
    return driveJoy;
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }


//==============================================================================
//=========================== Configure Bindings ===============================
  private void configureBindings() {
    driveJoy.getMiddle().onTrue(Commands.runOnce(drivetrain::resetOdometry, drivetrain));
    driveJoy.getLeft().onTrue(Commands.runOnce(drivetrain::resetEncoders, drivetrain));
  }

//==============================================================================
//=========================== Modify Joystick Input ============================
  private static double deadband(double value, double tolerance) {
    if (Math.abs(value) < tolerance) {
      return 0.0;
    }
      return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
  }

  private static double square(double value) {
    return Math.copySign(value * value, value);
  }


//============================================================================
//============================ Get Joystick Input ============================
  private double getForwardInput() {
    return -square(deadband(driveJoy.getY(), 0.1)) * Constants.Swerve.maxSpeed;
  }

  private double getStrafeInput() {
    return -square(deadband(driveJoy.getX(), 0.1)) * Constants.Swerve.maxSpeed;
  }
    
  private double getRotationInput() {
    return -square(deadband(driveJoy.getZ(), 0.1)) * Constants.Swerve.maxAngularVelocity;
  }
  
  private double getThrottleInput() {
    return Utilities.map(driveJoy.getThrottle(), 1, -1, 0, 1);
  }


//============================================================================
//============================== Path Planner ================================

  //==================== Path Following Event Map ========================
  private void configureEventMap() {
  //  eventMap.put("BalancingCommand", new BalancingCommand(drivetrain));
  }
  
  public SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    drivetrain::getPose,
    drivetrain::resetOdometry,
    new PIDConstants(1.5, 0.0, 0.0),
    new PIDConstants(0.5, 0.0, 0.0),
    drivetrain::drive,
    eventMap,
    drivetrain
  );
}
