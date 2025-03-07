package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
  private final TalonFX leftMotor = new TalonFX(55);
  private final TalonFX rightMotor = new TalonFX(56);

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);

  public Intake() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 150;
    config.CurrentLimits.SupplyCurrentLimit = 90;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 5;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    leftMotor.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightMotor.getConfigurator().apply(config);

    setDefaultCommand(stopIntake());
  }

  public static final double intakeVolts = 4;

  public Command runIntake() {
    return run(
        () -> {
          leftMotor.setControl(voltageOut.withOutput(intakeVolts));
          rightMotor.setControl(voltageOut.withOutput(intakeVolts));
        });
  }

  public Command run(double d) {
    return run(
        () -> {
          leftMotor.setControl(voltageOut.withOutput(d));
          rightMotor.setControl(voltageOut.withOutput(d));
        });
  }

  DutyCycleOut fullSend = new DutyCycleOut(-1.0);

  public Command fullSend2() {
    return run(
        () -> {
          leftMotor.setControl(fullSend);
          rightMotor.setControl(fullSend);
        });
  }

  public Command fullSend() {
    return run(
        () -> {
          leftMotor.setControl(voltageOut.withOutput(-12));
          rightMotor.setControl(voltageOut.withOutput(-12));
        });
  }

  public Command spinny() {
    return run(
        () -> {
          leftMotor.setControl(voltageOut.withOutput(12));
          rightMotor.setControl(voltageOut.withOutput(-12));
        });
  }

  public Command semiSpinny() {
    return run(
        () -> {
          leftMotor.setControl(voltageOut.withOutput(12));
          rightMotor.setControl(voltageOut.withOutput(0));
        });
  }

  public Command runVariable(DoubleSupplier d) {
    return run(
        () -> {
          leftMotor.setControl(voltageOut.withOutput(d.getAsDouble()));
          rightMotor.setControl(voltageOut.withOutput(d.getAsDouble()));
        });
  }

  public Command stopIntake() {
    return run(
        () -> {
          leftMotor.setControl(voltageOut.withOutput(0));
          rightMotor.setControl(voltageOut.withOutput(0));
        });
  }

  public Command ejectIntake() {
    return run(
        () -> {
          leftMotor.setControl(voltageOut.withOutput(-2));
          rightMotor.setControl(voltageOut.withOutput(-2));
        });
  }

  public Command leftMajority() {
    return run(
        () -> {
          leftMotor.setControl(voltageOut.withOutput(10));
          rightMotor.setControl(voltageOut.withOutput(4));
        });
  }
}
