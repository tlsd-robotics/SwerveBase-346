package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import Common.NavX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
 * I have probably said this before but this is the heart of the swerve robot. Using the Swerve Module blueprint in the SwerveModule file, 4 modules are created
 * using the Mod0 -> Mod4 constants. Personally I am not a fan of naming them Mod0 -> Mod4 compared to Left front, Right front.... etc. 
 * 
 * So as a test for the new programmers lets swap all the constant classes to their proper names for readabilities sake.
 * 
 * Anyways back to the drivetrain. As the common theme with all the files here, a lot less is happening when it looks like.
 * Proceed down to see what each thing does. 
 */

public class DrivetrainSubsystem extends SubsystemBase {
  // Declare variables for the gyroscope, odometry, the array for holding SwerveModules, and a Field2d objetct for storing field positioning in conjunction 
  // with the odometry
  private final AHRS gyro;
  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;
  private Field2d field;

  // Everything in here will be created when a DrivetrainSubsystem object is made. Which happen to be created in RobotContainer at the top.
  public DrivetrainSubsystem() {

    // Get the Gyro object from the Common Library file and zero the gyro upon creation.
    gyro = NavX.getNavX();
    zeroGyro();

    // Populate module array with 4 modules, each created using the constants found in the constants file. 
    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        // Create our Swerve Odometry object using the Swerve Kinematics, default Yaw value, and the default position for each swerve module. 
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), new SwerveModulePosition[]{mSwerveMods[0].getPosition(), mSwerveMods[1].getPosition(), mSwerveMods[2].getPosition(), mSwerveMods[3].getPosition()});

    // Create Field2d object
    field = new Field2d();

    // Put Field2d data onto SmartDashboard under the key "Field"
    SmartDashboard.putData("Field", field);
  }

  /**
   * Main Drive Function. Loops through each module and sets the desired state of each module
   * @param speeds ChasssisSpeed Object (x, y, z)
   * @param fieldRelative - True for field relative, false for robot orientied
   * @param isOpenLoop - Openloop = No Feedback : ClosedLoop = Feedback
   */
  public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw())
                : speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }

    // Push X,Y,Z data to SmartDashboard
    SmartDashboard.putNumber("Current X Command: ", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Current Y Command: ", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Current Z Command: ", speeds.omegaRadiansPerSecond);
  }

  /* 
  This is the drive function we use for teleop. Not going to lie, this one is a bit confusing for me. Not sure why it was setup this way. 
  You can see in the original drive function above that if fieldRelative is true, use fromFieldRelativeSpeeds. This function is to convert field relative TO robot relative
  So in theory, fieldRelative = true is acutally making it robot relative.... And in the DefaultDriveCommand it runs the same fromFieldRelative, so it doesn't matter what you choose there.
  This should result in a robot that is robot relative (front of robot is where drive forward will be) rather than field relative (forward is based on gyroscope readings and field position)

  This annoys me but ignore it you won't have to deal with any of this. hopefully.
  */ 
  /**
   * @param speeds ChassisSpeed (Forward, Strafe, Rotate)
   */
  public void drive(ChassisSpeeds speeds) {
    drive(speeds, false, false); //TODO: Determine if closed loop is beneficial
  }

  // This is the "fieldRelative" version of drive which I believe is used for the PathPlanner commands. Once again, doubt you will ever need to deal with this function.
    /**
   * @param speeds ChassisSpeed (Forward, Strafe, Rotate)
   */
  public void driveFieldRelative(ChassisSpeeds speeds) {
    drive(speeds, true, false);
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  /**
   * @return Odometry Position (x,y) cartesian coordinates
   */
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  /**
   * Resets the odometry objects position according to the current readings for each module and gyro... I assume. The description for the resetPostion function isn't the best.
   * @param pose Pse2d object
   */
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), new SwerveModulePosition[]{mSwerveMods[0].getPosition(), mSwerveMods[1].getPosition(), mSwerveMods[2].getPosition(), mSwerveMods[3].getPosition()}, pose);
  }

    /**
   * Resets the odometry objects position completely
   */
  public void resetOdometry() {
    resetOdometry(new Pose2d(swerveOdometry.getPoseMeters().getX(), swerveOdometry.getPoseMeters().getY(), Rotation2d.fromDegrees(0.0)));
  }

  /**
   * @return Swerve Module States
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  /**
   * Resets each CANCoder for their paired module. 
   * This code should run at robot start and be bound to a button in case of misalignment. 
   * The issue is that the speed controllers would load before the encoders, therefore not seeding the starting values
   * which caused the wheels to go haywire and behave in unwanted ways.
   */
  public void resetEncoders() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  /**
   * Zeros the gyroscope heading
   */
  public void zeroGyro() {
    gyro.reset();
  }

  /**
   * If the constants invertGyro is true, the value will be inverted before returned.
   * @return Yaw of Gyro
   */
  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  // This runs every loop (every 20ms). Updates odometry according to the current module positions. 
  @Override
  public void periodic() {  
    swerveOdometry.update(getYaw(), new SwerveModulePosition[]{mSwerveMods[0].getPosition(), mSwerveMods[1].getPosition(), mSwerveMods[2].getPosition(), mSwerveMods[3].getPosition()});
    field.setRobotPose(getPose());

    // Post all data into SmartDashboards
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
