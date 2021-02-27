// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpiutil.math.MathUtil;

public class TurnDegrees extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_degrees;
  private final double m_tolerance;
  private final double m_speed;

  private final PIDController m_pidController;

  private double m_targetDegrees;

  private final double kP = 0.008;
  private final double kI = 0.005;
  private final double kD = 0.001;

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
    SmartDashboard.putNumber("TurnDegrees/p", kP);
    SmartDashboard.putNumber("TurnDegrees/i", kI);
    SmartDashboard.putNumber("TurnDegrees/d", kD);
    m_pidController = new PIDController(kP, kI, kD);
    m_pidController.enableContinuousInput(-180, 180);
    m_pidController.setTolerance(tolerance);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    m_pidController.setPID(
      SmartDashboard.getNumber("TurnDegrees/p", kP), 
      SmartDashboard.getNumber("TurnDegrees/i", kI), 
      SmartDashboard.getNumber("TurnDegrees/d", kD)
    );
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
    m_pidController.reset();
    m_drive.resetGyro();
    m_pidController.setSetpoint(m_degrees);
    m_targetDegrees = m_degrees;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yaw = m_drive.getGyroAngleZ();
    SmartDashboard.putNumber("TurnDegrees/yaw", yaw);
    double modYaw = yaw % 360;
    SmartDashboard.putNumber("TurnDegrees/modyaw", modYaw);
    double pidOut = m_pidController.calculate(modYaw);
    double correction = MathUtil.clamp(pidOut, -m_speed, m_speed);
    SmartDashboard.putNumber("TurnDegrees/pidOut", pidOut);
    SmartDashboard.putNumber("TurnDegrees/correction", correction);

    m_drive.arcadeDrive(0, correction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }

  private double getAverageTurningDistance() {
    double leftDistance = Math.abs(m_drive.getLeftDistanceInch());
    double rightDistance = Math.abs(m_drive.getRightDistanceInch());
    return (leftDistance + rightDistance) / 2.0;
  }
}
