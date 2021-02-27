// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class TurnDegrees extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_degrees;
  private final double m_tolerance;
  private final double m_speed;

  private double m_targetDegrees;

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Should be positive-we'll adjust based on direction.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param tolerance Amount of error (in degrees) to accept.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegrees(double speed, double degrees, double tolerance, Drivetrain drive) {
    m_degrees = degrees;
    m_speed = Math.abs(speed);
    m_drive = drive;
    m_tolerance = tolerance;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
    m_drive.resetGyro();
    m_targetDegrees = m_degrees;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double P = 0.1;
    double turnSpeed = 0;
    double gyroAngle = m_drive.getGyroAngleZ();
    double error = m_targetDegrees - gyroAngle;
    SmartDashboard.putNumber("TurnDegrees/gyroAngle",gyroAngle);
    SmartDashboard.putNumber("TurnDegrees/error", error);
    if (error < 0) {
      turnSpeed = -Math.min(m_speed * Math.abs(error) * P, m_speed) ;
    } else if(error>0) {
      turnSpeed = Math.min(m_speed * Math.abs(error) * P, m_speed);
    }
    SmartDashboard.putNumber("TurnDegrees/turnSpeed", turnSpeed);

    m_drive.arcadeDrive(0, turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drive.getGyroAngleZ() -  m_targetDegrees) <= m_tolerance;
  }

  private double getAverageTurningDistance() {
    double leftDistance = Math.abs(m_drive.getLeftDistanceInch());
    double rightDistance = Math.abs(m_drive.getRightDistanceInch());
    return (leftDistance + rightDistance) / 2.0;
  }
}
