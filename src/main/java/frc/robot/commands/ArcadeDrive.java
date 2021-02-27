// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Utilities;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_zaxisRotateSupplier;

  private final PIDController m_pidController;

  private final double kP = 0.085;
  private final double kI = 0.001;
  private final double kD = 0.00;

  private boolean wasStraight = false;


  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param xaxisSpeedSupplier Lambda supplier of forward/backward speed
   * @param zaxisRotateSuppplier Lambda supplier of rotational speed
   */
  public ArcadeDrive(
      Drivetrain drivetrain,
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSuppplier) {
    m_drivetrain = drivetrain;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSuppplier;

    SmartDashboard.putNumber("ArcadeDrive/p", kP);
    SmartDashboard.putNumber("ArcadeDrive/i", kI);
    SmartDashboard.putNumber("ArcadeDrive/d", kD);
    m_pidController = new PIDController(kP, kI, kD);
    m_pidController.enableContinuousInput(-180, 180);
    m_pidController.setTolerance(0.5);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.setPID(
      SmartDashboard.getNumber("ArcadeDrive/p", kP), 
      SmartDashboard.getNumber("ArcadeDrive/i", kI), 
      SmartDashboard.getNumber("ArcadeDrive/d", kD)
    );

    m_pidController.reset();
    m_drivetrain.resetGyro();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_xaxisSpeedSupplier.get();
    double rotation = m_zaxisRotateSupplier.get();
    rotation = Utilities.applyDeadband(rotation, 0.01);
    rotation = Utilities.applySensitivityFactor(rotation, 0.5);

    if (Math.abs(rotation) < 0.01) {
      if (wasStraight) {
        double yaw = m_drivetrain.getGyroAngleZ();
        SmartDashboard.putNumber("ArcadeDrive/yaw", yaw/180);
        rotation = m_pidController.calculate(yaw);
        SmartDashboard.putNumber("ArcadeDrive/correction", rotation);    
      } else {
        m_pidController.setSetpoint(m_drivetrain.getGyroAngleZ());
        wasStraight = true;
      }
    } else {
      wasStraight = false;
    }

    m_drivetrain.arcadeDrive(speed, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
