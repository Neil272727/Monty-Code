package frc.robot.Subsystems.Drivetrain;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import frc.robot.Constants;

public class DrivetrainIOSim implements DrivetrainIO{

      TalonFX leftFalcon = new TalonFX(Constants.drivetrainLeftFalconID);
  TalonFX rightFalcon = new TalonFX(Constants.drivetrainRightFalconID);

  VoltageOut leftVoltage = new VoltageOut(0);
  VoltageOut rightVoltage = new VoltageOut(0);


DifferentialDrivetrainSim physicsSim = DifferentialDrivetrainSim.createKitbotSim(
    KitbotMotor.kDoubleFalcon500PerSide, 
    KitbotGearing.k8p45,
    KitbotWheelSize.kSixInch, 
    null);


    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
    physicsSim.update(0.020);

var leftSimState = leftFalcon.getSimState();
leftSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());

var rightSimState = rightFalcon.getSimState();
rightSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());

physicsSim.setInputs(leftSimState.getMotorVoltage(), rightSimState.getMotorVoltage());

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

inputs.leftOutputVolts = leftSimState.getMotorVoltage();
inputs.rightOutputVolts = rightSimState.getMotorVoltage();

inputs.leftCurrentAmps = new double[] {leftSimState.getTorqueCurrent()};
inputs.leftTempCelsius = new double[0];
inputs.rightCurrentAmps = new double[] {rightSimState.getTorqueCurrent()};
inputs.rightTempCelsius = new double[0];

    }

    @Override
    public void setVolts(double left, double right) {
    leftFalcon.setControl(leftVoltage.withOutput(left));
    rightFalcon.setControl(rightVoltage.withOutput(right));
    }   
}
