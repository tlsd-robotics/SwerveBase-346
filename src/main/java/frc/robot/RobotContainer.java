package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import Common.ArcadeControls;
import Common.LogitechF310;
import Common.ThrustMaster;
import Common.Utilities;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Drive.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {

/*
 * NOTE - This file has a lot of green. All green text like this is a comment. These do not do anything to the code and are purely for information.
 * The majority of these comments are a type known as JavaDoc. The cool thing about those is that it makes it so when you hover over functions that
 * have been commented by JavaDoc, it will show the description. This works from any file in the project. Idealy, all functions will have one of these so if
 * you are ever unsure of what something does. Hover over and read the description. 
 * 
 * And remember, the green makes these files look very messy and overwhelming, but as long as you know
 * it is there you help you, I am sure you will grow to enjoy having a lot of green comments everywhere. 
 */

//==================================================================================================================
//======================== Create Subsystem Instance, Controllers, and various Variables ===========================
  private final static DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final Superstructure superstructure = new Superstructure();

  private final ThrustMaster driveJoy = new ThrustMaster(Constants.Controls.DRIVE_CONTROLLER);
  // private final LogitechF310 gamepad = new LogitechF310(Constants.Controls.GAMEPAD);
  // private final ArcadeControls arcade = new ArcadeControls(Constants.Controls.ARCADE);

  HashMap <String, Command> eventMap = new HashMap<String, Command>();

  public RobotContainer() {
      drivetrain.register();

      drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, this::getForwardInput, this::getStrafeInput, this::getRotationInput, this::getThrottleInput));

      configureBindings();
      configureEventMap();
  }


//==============================================================================
//=============================== Getter Methods ===============================

/**
 * @return Drivetrain subsystem instance
 */
  public static DrivetrainSubsystem getDrivetrain() {
    return drivetrain;
  }

/**
 * @return Thrustmaster joystick instance
 */
  public ThrustMaster getDriveJoystick() {
    return driveJoy;
  }

/**
 * @return Logitech gamepad instance
 */
  // public LogitechF310 getGamepad() {
  //   return gamepad;
  // }

/**
 * @return Arcade controller instance
 */
  // public ArcadeControls getArcade() {
  //   return arcade;
  // }

/**
 * @return Superstructure instance
 */
  public Superstructure getSuperstructure() {
    return superstructure;
  }


//==============================================================================
//=========================== Configure Bindings ===============================

/**
 * This function will bind commands to buttons and conditional triggers. 
 */
  private void configureBindings() {
    driveJoy.getMiddle().onTrue(Commands.runOnce(drivetrain::resetOdometry, drivetrain));
    driveJoy.getLeft().onTrue(Commands.runOnce(drivetrain::resetEncoders, drivetrain));
  }

//==============================================================================
//=========================== Modify Joystick Input ============================

/**
 * Apply a deadband to the inputted value. A deadband is a value that will prevent a joystick from sending super small values. 
 * The reason why this is crucial is because motors must have a certain amount of power sent to them in order for them to start moving.
 * This is known as overcomming fricion. When a small value is send to a motor, it will be unable to move and eventually burn up and damage the motor.
 * @param value
 * @param tolerance
 * @return Value with specified deadband applied
 */
  private static double deadband(double value, double tolerance) {
    if (Math.abs(value) < tolerance) {
      return 0.0;
    }
      return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
  }

  /**
   * Squaring a value will make the value more precise and controllable at lower speeds. Makes the joystick exponential rather than a linear change.
   * @param value
   * @return Squared value
   */
  private static double square(double value) {
    return Math.copySign(value * value, value);
  }


//============================================================================
//============================ Get Joystick Input ============================

/**
 * Runs the y axis joystick value through both the deadband and square function. This value can be inversed by adding or removing the "-" before the square
 * or multiplying the whole value by "-1".
 * @return Modifed joystick input value
 */
  private double getForwardInput() {
    return -square(deadband(driveJoy.getY(), 0.1)) * Constants.Swerve.maxSpeed;
  }

/**
 * Runs the x axis joystick value through both the deadband and square function. This value can be inversed by adding or removing the "-" before the square
 * or multiplying the whole value by "-1".
 * @return Modifed joystick input value
 */
  private double getStrafeInput() {
    return -square(deadband(driveJoy.getX(), 0.1)) * Constants.Swerve.maxSpeed;
  }

    /**
 * Runs the z axis joystick value through both the deadband and square function. This value can be inversed by adding or removing the "-" before the square
 * or multiplying the whole value by "-1".
 * @return Modifed joystick input value
 */
  private double getRotationInput() {
    return -square(deadband(driveJoy.getZ(), 0.1)) * Constants.Swerve.maxAngularVelocity;
  }

  /**
 * Maps the Throttle selector on the joystick to a range of <code>0 to 1</code> compared to the original <code>-1 to 1.</code>
 * This will allow the throttle to be used to control the speed at which the robot drives.
 * @return Modifed throttle input value
 */
  private double getThrottleInput() {
    return Utilities.map(driveJoy.getThrottle(), 1, -1, 0, 1);
  }


//============================================================================
//============================== Path Planner ================================

  //==================== Path Following Event Map ========================

  /**
   * Configures the EventMap array as the name implies. The map is a key-value pair where the name will be used to
   * reference the corresponding command. These names will be used within the path planner software to create events mid path. 
   */
  private void configureEventMap() {
    // Example for adding a command
  //  eventMap.put("BalancingCommand", new BalancingCommand(drivetrain));
  }
  
  /**
   * This function will turn a Path Planner file (Found under src/main/deploy/pathPlanner), into a sequencial command group that will execute
   * when called. Most notably during autonomous.
   */
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
