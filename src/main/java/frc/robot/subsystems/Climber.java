package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final TalonFX motor = new TalonFX(55);
  private final TalonFX motor2 = new TalonFX(66);

  private final VelocityVoltage control = new VelocityVoltage(1);
  private final VelocityVoltage control2 = new VelocityVoltage(1);

  public Command goUp() {
    return run(
        () -> {
          motor.setControl(control.withVelocity(5));
          motor2.setControl(control2.withVelocity(5));
        });
  }

  public Command goDown() {
    return run(
        () -> {
          motor.setControl(control.withVelocity(0));
          motor.setControl(control.withVelocity(0));
        });
  }
}
