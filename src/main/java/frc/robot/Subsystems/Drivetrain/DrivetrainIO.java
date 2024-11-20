package frc.robot.Subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface DrivetrainIO {

  @AutoLog
  public static class DrivetrainIOInputs {
    public Pose2d robotPose2d = new Pose2d();
    public double leftOutputVolts = 0.0;
    public double rightOutputVolts = 0.0;

    public double leftVelocityMetersperSecond = 0.0;
    public double rightVelocityMetersperSecond = 0.0;

    public double leftPositionMeters = 0.0;
    public double rightPositionMeters = 0.0;

    public double[] leftCurrentAmps = new double[0];
    public double[] leftTempCelsius = new double[0];
    public double[] rightCurrentAmps = new double[0];
    public double[] rightTempCelsius = new double[0];
  }

  public void updateInputs(DrivetrainIOInputs inputs);

  public void setVolts(double left, double right);
}
