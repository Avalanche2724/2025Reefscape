package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private static final int ELEVATOR_ID = 40;

  private static final double GEAR_RATIO = 20.0;
  private static final Mass MASS = Kilograms.of(4.0);
  private static final Distance DRUM_RADIUS = Inches.of(2.0);
  private static final Distance MIN_HEIGHT = Meters.of(0.0);
  private static final Distance MAX_HEIGHT = Meters.of(5.0);

  private final PositionVoltage control = new PositionVoltage(0);

  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          GEAR_RATIO,
          MASS.in(Kilograms),
          DRUM_RADIUS.in(Meters),
          MIN_HEIGHT.in(Meters),
          MAX_HEIGHT.in(Meters),
          true,
          MIN_HEIGHT.in(Meters),
          0.01,
          0.0);

  private final TalonFX motor = new TalonFX(ELEVATOR_ID);

  public Elevator() {
    var config = new TalonFXConfiguration();
    config.Slot0.kP = 1;
    config.Feedback.RotorToSensorRatio = GEAR_RATIO;
    motor.getConfigurator().apply(config);
    motor.setPosition(0);
  }

  private void setMotorPosition(double pos) {
    motor.setControl(control.withPosition(pos));
  }

  public Command downCommand() {
    return runOnce(() -> setMotorPosition(0));
  }

  public Command upCommand() {
    return runOnce(() -> setMotorPosition(0));
  }

  public Command midCommand() {
    return runOnce(() -> setMotorPosition(0));
  }

  public Command upmidCommand() {
    return runOnce(() -> setMotorPosition(0));
  }

  public Command downmidCommand() {
    return runOnce(() -> setMotorPosition(0));
  }

  @Override
  public void simulationPeriodic() {}
}
