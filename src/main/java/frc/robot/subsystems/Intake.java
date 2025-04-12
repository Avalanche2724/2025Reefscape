package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controls;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  public static final double intakeVolts = 12;
  public static final int LEFTMOTOR_ID = 55;
  public static final int RIGHTMOTOR_ID = 56;

  private final TalonFX leftMotor = new TalonFX(LEFTMOTOR_ID);
  private final TalonFX rightMotor = new TalonFX(RIGHTMOTOR_ID);
  private final VoltageOut voltageOut = new VoltageOut(0);
  private final TorqueCurrentFOC torqueCurrent = new TorqueCurrentFOC(0);
  public Trigger hasGamePieceTrigger = new Trigger(this::hasGamePiece).debounce(0.1);
  DutyCycleOut fullSend = new DutyCycleOut(-1.0);

  // TODO
  private boolean everControlledSinceDisable = false;

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

  private void setVoltages(double left, double right) {
    leftMotor.setControl(voltageOut.withOutput(left));
    rightMotor.setControl(voltageOut.withOutput(right));
  }

  public Command runIntake() {
    return run(() -> setVoltages(intakeVolts, intakeVolts));
  }

  public Command run(double d) {
    return run(() -> setVoltages(d, d));
  }

  // used during algae launch sequence to keep algae in
  public Command holdIntake() {
    return run(3);
  }

  // used during algae launch
  // todo: lower voltages so consistent?
  public Command fullSend() {
    return run(() -> setVoltages(-12, -12));
  }

  // general outtake command
  public Command semiSend() {
    return run(() -> setVoltages(-2.5, -2.5));
  }

  // outtake command in algae mode
  public Command algSend() {
    return run(() -> setVoltages(-1.1, -1.1));
  }

  public Command fixy() {
    int[] a = new int[1];
    return startRun(
        () -> a[0]++,
        () -> {
          // what even is this??
          setVoltages(12 * (((a[0] % 4) / 2 == 1) ? 1 : -1), 12 * ((a[0] % 2 == 0 ? 1 : -1)));
        });
  }

  // for L1 outtake maybe?
  public Command semiSpinny() {
    return run(() -> setVoltages(12, 0));
  }

  // used for default command
  public Command stopIntake() {
    return run(
        () -> {
          double thing = Controls.isOnCoralBindings ? 0 : 0.5;
          setVoltages(thing, thing);
        });
  }

  // used in some auto routines except none of them are being used rn?
  public Command ejectIntake() {
    return run(
        () -> {
          setVoltages(-2, -2);
        });
  }

  // used in l1forauto orig and it should probably be used in some others too
  public Command ejectL1intake() {
    return run(
        () -> {
          setVoltages(-2.6, -2.6);
        });
  }

  // current bound to intake command driver
  public Command leftMajority() {
    int[] a = new int[1];
    return startRun(
        () -> a[0]++,
        () -> {
          if (a[0] % 2 == 1) {
            setVoltages(8, 4);
          } else {
            setVoltages(4, 8);
          }
        });
  }

  // experiment 1
  public Command alternatingSpinny() {
    int[] a = new int[1];
    return startRun(
        () -> a[0] = 0,
        () -> {
          a[0]++;
          if (a[0] % 50 < 25) {
            setVoltages(12, -1);
          } else {
            setVoltages(-1, 12);
          }
        });
  }
}
