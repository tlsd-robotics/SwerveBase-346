package frc.robot.commands.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

/*
 * Welcome to the DEFAULT DRIVE COMMAND!!!!
 * 
 * This command is the heart and soul of our swerve drive robot since it is the thing that makes it move.
 * It looks more complex than it is. Pretty much just gets values passed into it found on comment #1, and then 
 * sets the drive command to those values after getting them into a usable format. 
 * 
 * In the execute, it takes the joystick value, multiplies it against the max speed constant and the throttle control 
 * to get the overall speed being passed into the drivetrain. The extra " * 0.4 " is to make the turning slower than is normally would be allowed.
 * This is setup to only run it at 40% speed. But if the driver is more confortable, it can be increased or decreased as needed.
 */

public class DefaultDriveCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private final DoubleSupplier throttleSupplier;


    // Comment 1 - Getting values passed to it. 
    public DefaultDriveCommand(
            DrivetrainSubsystem drivetrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            DoubleSupplier throttleSupplier
    ) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.throttleSupplier = throttleSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double translationXPercent = translationXSupplier.getAsDouble();
        double translationYPercent = translationYSupplier.getAsDouble();
        double rotationPercent = rotationSupplier.getAsDouble();
        double throttlePercent = throttleSupplier.getAsDouble();

        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationXPercent * Constants.Swerve.maxSpeed * throttlePercent,
                        translationYPercent * Constants.Swerve.maxSpeed * throttlePercent,
                        rotationPercent * Constants.Swerve.maxAngularVelocity * throttlePercent * 0.4,
                        drivetrain.getPose().getRotation()
                )
        );
    }

    // And once again, SET MOTORS TO ZERO WHEN DONE. If not they will drive into infinity and never been seen again. 
    // And probably run into something and break it and itself.

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
