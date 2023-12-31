// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  /*
   * This is robot. Where the robot does its thing. Main things to know in here are
   * Adding new autonomous routines can be done by the sendableChooser and adding options. 
   * Reference 2023 Charged-Up-Revised Temporary branch for making auto commands here.
   * 
   * Next is the init and periodic functions. They run on initaliztion and periodicly (every 20ms).
   * These are good for startup routines, such as reseting the motor encoders, or providing dashboard values and charts periodicly.
   */

  public static SendableChooser<Command> sendablechooser = new SendableChooser<Command>();
  private Command m_autonomousCommand;

  @SuppressWarnings("unused")
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    RobotContainer m_robotContainer = new RobotContainer();

    sendablechooser.setDefaultOption("Do nothing", null);
    sendablechooser.addOption("Path Auto", m_robotContainer.autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath", 2, 1)));
    SmartDashboard.putData("Autonomous", sendablechooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    RobotContainer.getDrivetrain().resetEncoders();

    m_autonomousCommand = sendablechooser.getSelected();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
