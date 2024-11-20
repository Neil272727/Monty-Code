package frc.robot.Subsystems.Drivetrain;

public interface DrivetrainIO {
    public static class DrivetrainIOInputs {
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
public abstract void updateInputs(DrivetrainIOInputs inputs);

public abstract void setVolts(double left, double right);
}

