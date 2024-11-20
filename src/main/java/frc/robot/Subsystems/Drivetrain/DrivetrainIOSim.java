package frc.robot.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

public class DrivetrainIOSim implements DrivetrainIO {

  DifferentialDrivetrainSim physicsSim =
      DifferentialDrivetrainSim.createKitbotSim(
          KitbotMotor.kDoubleFalcon500PerSide, KitbotGearing.k8p45, KitbotWheelSize.kSixInch, null);
  public static double leftVolts, rightVolts;

  @Override
  public void updateInputs(DrivetrainIOInputs inputs) {
    physicsSim.update(0.020);

    inputs.leftOutputVolts = 0.0;
    inputs.rightOutputVolts = 0.0;

    inputs.leftVelocityMetersperSecond = 0.0;
    inputs.rightVelocityMetersperSecond = 0.0;

    inputs.leftPositionMeters = 0.0;
    inputs.rightPositionMeters = 0.0;

    inputs.leftVelocityMetersperSecond = physicsSim.getLeftVelocityMetersPerSecond();
    inputs.rightVelocityMetersperSecond = physicsSim.getRightVelocityMetersPerSecond();

    inputs.leftPositionMeters = physicsSim.getLeftPositionMeters();
    inputs.rightPositionMeters = physicsSim.getRightPositionMeters();

    inputs.leftCurrentAmps = new double[0];
    inputs.leftTempCelsius = new double[0];
    inputs.rightCurrentAmps = new double[0];
    inputs.rightTempCelsius = new double[0];

    inputs.leftOutputVolts = leftVolts;
    inputs.rightOutputVolts = rightVolts;

    inputs.leftCurrentAmps = new double[] {physicsSim.getLeftCurrentDrawAmps()};
    inputs.leftTempCelsius = new double[0];
    inputs.rightCurrentAmps = new double[] {physicsSim.getRightCurrentDrawAmps()};
    inputs.rightTempCelsius = new double[0];
  }

  @Override
  public void setVolts(double left, double right) {
    physicsSim.setInputs(left, right);
    leftVolts = left;
    rightVolts = right;
  }
}
