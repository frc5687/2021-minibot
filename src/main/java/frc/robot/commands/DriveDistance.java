// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class DriveDistance extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_distance;
  private final double m_speed;

  private final PIDController m_pidController;

  private final double kP = 0.02;
  private final double kI = 0.00;
  private final double kD = 0.00;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistance(double speed, double inches, Drivetrain drive) {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
    SmartDashboard.putNumber("DriveDistance/p", kP);
    SmartDashboard.putNumber("DriveDistance/i", kI);
    SmartDashboard.putNumber("DriveDistance/d", kD);
    m_pidController = new PIDController(kP, kI, kD);
    m_pidController.enableContinuousInput(-180, 180);
    m_pidController.setTolerance(0.5);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.setPID(
      SmartDashboard.getNumber("DriveDistance/p", kP), 
      SmartDashboard.getNumber("DriveDistance/i", kI), 
      SmartDashboard.getNumber("DriveDistance/d", kD)
    );

    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
    m_pidController.reset();
    m_drive.resetGyro();
    m_pidController.setSetpoint(m_drive.getGyroAngleZ());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yaw = m_drive.getGyroAngleZ();
    SmartDashboard.putNumber("DriveDistance/yaw", yaw);
    double correction = m_pidController.calculate(yaw);
    SmartDashboard.putNumber("DriveDistance/correction", correction);
    m_drive.arcadeDrive(m_speed, correction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(m_drive.getAverageDistanceInch()) >= m_distance;
  }
}
