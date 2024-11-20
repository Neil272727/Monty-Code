// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DrivetrainSubsystem extends SubsystemBase {
  DrivetrainIO io;
  DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged();

  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);

  private void setVoltages(double left, double right) {
    io.setVolts(
        left * RobotController.getInputVoltage(), right * RobotController.getInputVoltage());
  }

  public Command setVoltagesCommand(DoubleSupplier left, DoubleSupplier right) {
    return new RunCommand(() -> this.setVoltages(left.getAsDouble(), right.getAsDouble()), this);
  }

  public Command voltagesArcadeCommand(DoubleSupplier driveSupplier, DoubleSupplier steerSupplier) {
    return new RunCommand(
        () -> {
          double drive = driveSupplier.getAsDouble();
          double steer = steerSupplier.getAsDouble();
          drive = MathUtil.applyDeadband(drive, 0.1);
          steer = MathUtil.applyDeadband(steer, 0.1);
          double left = drive - steer;
          double right = drive + steer;

          this.setVoltages(left, right);
        },
        this);
  }

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem(DrivetrainIO motor) {
    io = motor;
  }

  @Override
  public void periodic() {
    inputs.robotPose2d =
        odometry.update(
            odometry
                .getPoseMeters()
                .getRotation()
                // Use differential drive kinematics to find the rotation rate based on the
                // wheel speeds and distance between wheels
                .plus(
                    Rotation2d.fromRadians(
                        (inputs.leftVelocityMetersperSecond - inputs.rightVelocityMetersperSecond)
                            * 0.020
                            / Units.inchesToMeters(26))),
            inputs.leftPositionMeters,
            inputs.rightPositionMeters);
    io.updateInputs(inputs);
    Logger.processInputs("Drivetrain", inputs);
    Logger.recordOutput("Drivetrain Pose", odometry.getPoseMeters());
  }
}
