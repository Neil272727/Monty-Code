// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class DrivetrainSubsystem extends SubsystemBase {
DrivetrainIO io = new DrivetrainIOSim();
DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged();

DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);



  TalonFX leftFalcon = new TalonFX(Constants.drivetrainLeftFalconID);
  TalonFX rightFalcon = new TalonFX(Constants.drivetrainRightFalconID);

  VoltageOut leftVoltage = new VoltageOut(0);
  VoltageOut rightVoltage = new VoltageOut(0);

  private void setVoltages(double left, double right) {
    io.setVolts(left,right);
    leftFalcon.setControl(leftVoltage.withOutput(left));
    rightFalcon.setControl(rightVoltage.withOutput(left));
  }

  public Command setVoltagesCommand(DoubleSupplier left, DoubleSupplier right) {
    return new RunCommand(() -> this.setVoltages(left.getAsDouble(), right.getAsDouble()), this);
  }

  public Command setVoltagesArcadeCommand(DoubleSupplier drive, DoubleSupplier steer) {
    return new RunCommand(
        () -> {
          var speeds =
              DifferentialDrive.arcadeDriveIK(drive.getAsDouble(), steer.getAsDouble(), false);
          this.setVoltages(speeds.left * 12, speeds.right * 12);
        },
        this);
  }

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {}

  @Override
  public void periodic() {
io.updateInputs(inputs);
Logger.processInputs("Drivetrain", inputs);

odometry.update(
odometry.getPoseMeters().getRotation()
.plus(
Rotation2d.fromRadians(
  (inputs.leftSpeedMetersPerSecond - inputs.rightSpeedMetersPerSecond) * 0.020 / Units.inchesToMeters(26)
)), inputs.leftPositionMeters, inputs.rightPositionMeters);
Logger.recordOutput("Drivebase Pose", odometry.getPoseMeters());
  }
}
