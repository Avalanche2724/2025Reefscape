package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase {
  public static final int MOTOR_ID = 61;
  private final TalonFX motor = new TalonFX(MOTOR_ID);
  private final VoltageOut control = new VoltageOut(0);
  private final PositionVoltage positionControl = new PositionVoltage(0);

  public Climber() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.StatorCurrentLimit = 200;
    config.CurrentLimits.SupplyCurrentLimit = 90;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 4;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;

    config.Feedback.SensorToMechanismRatio = 100;
    config.Slot0.kP = 500;
    // config.Slot0.kS = 0.25;
    // config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    motor.getConfigurator().apply(config);
  }

  public Command runVoltage(DoubleSupplier arg) {
    return runEnd(
        () -> motor.setControl(control.withOutput(arg.getAsDouble())), this::stayAtPosition);
  }

  private void stayAtPosition() {
    motor.setControl(positionControl.withPosition(motor.getPosition().getValueAsDouble()));
  }

  private Command goToPosition(double arg) {
    return run(() -> motor.setControl(positionControl.withPosition(arg)));
  }

  public Command zeroPos() {
    return goToPosition(0);
  }

  public Command preClimbPos() {
    return goToPosition(1.1);
  }

  public Command postClimbPos() {
    return goToPosition(-1.25);
  }
}
