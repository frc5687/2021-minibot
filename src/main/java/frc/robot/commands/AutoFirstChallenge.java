// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoFirstChallenge extends SequentialCommandGroup {
  private final double pause = 0.5;
  private final double turnSpeed = 0.5;
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutoFirstChallenge(Drivetrain drivetrain) {
    addCommands(
        new DriveDistance(0.5, 17, drivetrain),
        new WaitCommand(pause),
        new TurnDegrees(turnSpeed, -90, 0.4,  drivetrain),
        new WaitCommand(pause),
        new DriveDistance(0.5, 19, drivetrain),
        new WaitCommand(pause),
        new TurnDegrees(turnSpeed, -150, 0.4, drivetrain),
        new WaitCommand(pause),
        new DriveDistance(0.5, 28, drivetrain),
        new WaitCommand(pause),
        new TurnDegrees(turnSpeed, 115, 0.4, drivetrain),
        new WaitCommand(pause),
        new DriveDistance(0.5, 19, drivetrain),
        new WaitCommand(pause),
        new TurnDegrees(turnSpeed, 90, 0.4, drivetrain),
        new WaitCommand(pause),
        new DriveDistance(0.5, 15, drivetrain)
        );
  }
}
