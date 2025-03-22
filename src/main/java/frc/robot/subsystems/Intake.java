package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
  public static final double intakeVolts = 12;
  private final TalonFX leftMotor = new TalonFX(55);
  private final TalonFX rightMotor = new TalonFX(56);
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  public Trigger hasGamePieceTrigger = new Trigger(this::hasGamePiece).debounce(0.2);
  DutyCycleOut fullSend = new DutyCycleOut(-1.0);

  public Intake() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 200;
    config.CurrentLimits.SupplyCurrentLimit = 90;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 4;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    leftMotor.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightMotor.getConfigurator().apply(config);

    setDefaultCommand(stopIntake());
  }

  public boolean hasGamePiece() {
    if (Robot.isSimulation()) return false;
    return (leftMotor.getTorqueCurrent().getValueAsDouble() > 10
        && rightMotor.getTorqueCurrent().getValueAsDouble() > 10
        && leftMotor.getVelocity().getValueAsDouble() < 2
        && rightMotor.getVelocity().getValueAsDouble() < 2);
  }

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

  public Command holdIntake() {
    return run(3);
  }

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

  public Command semiSend() {
    return run(
        () -> {
          leftMotor.setControl(voltageOut.withOutput(-6));
          rightMotor.setControl(voltageOut.withOutput(-6));
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
    int[] a = new int[1];
    return run(
        () -> {
          // a[0]++;
          // if ((a[0] / 50) % 2 == 1) {
          leftMotor.setControl(voltageOut.withOutput(9));
          rightMotor.setControl(voltageOut.withOutput(4));
          // } else {
          // leftMotor.setControl(voltageOut.withOutput(11));
          // rightMotor.setControl(voltageOut.withOutput(5));
          // }
        });
  }
}
