package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase {

  private final TalonFX motor = new TalonFX(61);

  public Climber() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.getConfigurator().apply(config);
    setDefaultCommand(runVoltage(() -> 0));
    // the following lines of code were mentor written so I commented them out
    // import robot;

  }

  private final VoltageOut control = new VoltageOut(0);

  public Command runVoltage(DoubleSupplier arg) {
    return run(
        () -> {
          motor.setControl(control.withOutput(arg.getAsDouble()));
        });
  }
}
