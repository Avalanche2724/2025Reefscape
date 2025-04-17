package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@SuppressWarnings("FieldCanBeLocal") // Stop intellij complaints
public class Superstructure extends SubsystemBase {
  public enum Position {
    // Intake:
    MIN_INTAKE_GROUND(Elevator.MIN_HEIGHT, -10),
    ALG_INTAKE_GROUND(0.33, 0),
    INTAKE_CORAL_STATION(0.83, 35),
    // Processor algae + lollipop algae
    // TODO: algae presets angled up to catch algae falling
    ALG_PROC(0.65, 0),

    // Various stowing configurations:
    STOW(Elevator.MIN_HEIGHT, 90),
    ALGAELAUNCHSTOW(Elevator.MIN_HEIGHT, 54),
    HPSTOW(Elevator.MIN_HEIGHT, 35),
    FLATSTOW(Elevator.MIN_HEIGHT, 0),

    // Straight outtake:
    OUTTAKE_L1(0.75, 0),
    // Launching outtake:
    OUTTAKE_L2_LAUNCH(0.95, 0),
    OUTTAKE_L3_LAUNCH(1.36, 0),
    // Vertical outtake:
    // Algae:
    INTAKE_ALGAE_L2(1.04, 0),
    INTAKE_ALGAE_L3(1.42, 0);

    // Meters
    public final double elevatorHeight;
    // Degrees
    public final double wristAngle;

    Position(double elevatorHeight, double wristAngle) {
      this.elevatorHeight = elevatorHeight;
      this.wristAngle = wristAngle;
    }
  }

  private static final double ELEVATOR_AT_POSITION_THRESHOLD = Meters.convertFrom(1, Inch);
  private static final double WRIST_THRESHOLD = 3; // DEGREES
  // Simulation
  private static final double SIM_LOOP_PERIOD = 0.005;

  // Use for command dependencies:
  static Subsystem instance;
  // Contained subsystems:
  public final Elevator elevator;
  public final Wrist wrist;

  // Logic for triggers
  public final Trigger isStowed;

  // State and things
  Position lastSetPosition = Position.STOW;
  double currentElevatorTargetPosition = Elevator.MIN_HEIGHT;
  double currentWristTargetPosition = 0;

  // Simulation stuff
  private Notifier simNotifier = null;
  private double m_lastSimTime;

  public Superstructure() {
    instance = this;
    elevator = new Elevator();
    wrist = new Wrist();

    createMechanism2d();
    if (Robot.isSimulation()) {
      createSimulationThread();
    }

    stopMotors();
    RobotModeTriggers.disabled().onTrue(stop().ignoringDisable(true));

    // TODO
    isStowed =
        new Trigger(
            () ->
                (lastSetPosition == Position.STOW
                        || lastSetPosition == Position.HPSTOW
                        || lastSetPosition == Position.FLATSTOW
                        || lastSetPosition == Position.ALGAELAUNCHSTOW)
                    && (atWristPositionApprox(currentWristTargetPosition)
                        || currentWristTargetPosition == Position.STOW.wristAngle
                        || currentWristTargetPosition == Position.HPSTOW.wristAngle
                        || currentWristTargetPosition == Position.FLATSTOW.wristAngle
                        || currentWristTargetPosition == Position.ALGAELAUNCHSTOW.wristAngle)
                    && atMostElevatorPosition(0.4)
                    && atLeastElevatorPosition(0.25)
                    && getCurrentCommand() == null);

    // Auto-zero elevator when stowed
    isStowed.onTrue(zeroElevatorCommand());
  }

  @Override
  public void periodic() {
    updateMechanism2d();

    // Run normal periodic methods
    elevator.periodic();
    wrist.periodic();
    var c = getCurrentCommand();
    if (c != null) SmartDashboard.putString("CURRENT SUP COMMAND", c.getName());
    else SmartDashboard.putString("CURRENT SUP COMMAND", "null");

    SmartDashboard.putNumber("S ELEV TARGET POSITION", currentElevatorTargetPosition);
    SmartDashboard.putNumber("S WRIST TARGET POSITION", currentWristTargetPosition);
    SmartDashboard.putBoolean("AT TARGET POSITION", atTargetPosition());
  }

  // Inputs
  public boolean atTargetPosition() {
    return atElevatorPosition(currentElevatorTargetPosition)
        && atWristPosition(currentWristTargetPosition);
  }

  public boolean atLeastElevatorPosition(double height) {
    return elevator.getElevatorHeight() - height > 0;
  }

  public boolean atMostElevatorPosition(double height) {
    return elevator.getElevatorHeight() - height < 0;
  }

  public boolean atElevatorPosition(double height) {
    return Math.abs(elevator.getElevatorHeight() - height) < ELEVATOR_AT_POSITION_THRESHOLD;
  }

  public boolean atWristPosition(double angle) {
    return Math.abs(wrist.getWristDegrees() - angle) < WRIST_THRESHOLD;
  }

  public boolean atWristPositionApprox(double angle) {
    return Math.abs(wrist.getWristDegrees() - angle) < 15;
  }

  // Controls
  public void setPositions(double elevatorHeight, double wristAngle) {
    currentElevatorTargetPosition = elevatorHeight;
    currentWristTargetPosition = wristAngle;

    elevator.setMotorPosition(elevatorHeight);
    wrist.setMotorDegrees(wristAngle);
  }

  private void setPositions(Position pos) {
    lastSetPosition = pos;
    setPositions(pos.elevatorHeight, pos.wristAngle);
  }

  private void stopMotors() {
    elevator.stopMotor();
    wrist.stopMotor();
  }

  // Commands
  public Command setWristPositionCommand(double angle) {
    return runOnce(() -> wrist.setMotorDegrees(angle));
  }

  public Command stop() {
    return run(this::stopMotors);
  }

  public Command zeroElevatorCommand() {
    return sequence(
            parallel(
                run(elevator::setMotorZeroingVelocity).until(elevator.isStallingTrigger),
                Commands.print("setting zmv")),
            run(elevator::stopMotor).withTimeout(0.5).finallyDo(elevator::zeroElevatorPosition))
        .withName("ZERO ELEV");
  }

  public Command elevatorAlgaeLaunchSetup() {
    return sequence(
        setWristPositionCommand(Position.ALGAELAUNCHSTOW.wristAngle),
        runOnce(elevator::setMotorLaunchingVelocityUp),
        waitUntil(() -> atLeastElevatorPosition(0.88)));
  }

  public Command elevatorAlgaeLaunchPostscript() {
    return sequence(
        runOnce(elevator::setMotorLaunchingVelocityDown),
        waitSeconds(0.3),
        runOnce(() -> setPositions(Position.ALGAELAUNCHSTOW)));
  }

  public Command protectCommand(Command t) {
    return t.finallyDo(
        (interrupted) -> {
          if (!t.isFinished() || interrupted) {
            setPositions(Position.ALGAELAUNCHSTOW);
          }
        });
  }

  public Command goToPosition(Position pos) {
    return run(() -> setPositions(pos));
  }

  public Command goToPositionOnce(Position pos) {
    return runOnce(() -> setPositions(pos));
  }

  public Command goToPosition(Supplier<Position> pos) {
    return run(() -> setPositions(pos.get()));
  }

  public Command getToPositionThenHold(Position pos) {
    return goToPosition(pos).finallyDo(() -> setPositions(Position.STOW));
  }

  public Command incrementElevator(DoubleSupplier d) {
    return run(
        () ->
            setPositions(
                currentElevatorTargetPosition + d.getAsDouble(), currentWristTargetPosition));
  }

  public Command incrementWrist(DoubleSupplier d) {
    return run(
        () ->
            setPositions(
                elevator.getElevatorHeight(), currentWristTargetPosition + d.getAsDouble()));
  }

  // Mechanism + simulation stuff
  public void createMechanism2d() {
    // the main mechanism object
    var mechanism = new Mechanism2d(Meters.convertFrom(28, Inches), 4);
    // the mechanism root node
    var root = mechanism.getRoot("root", Inches.of(19.5).in(Meters), 0);

    root.append(elevator.createMechanism2d()).append(wrist.createMechanism2d());

    SmartDashboard.putData("Mechanism", mechanism);
  }

  public void updateMechanism2d() {
    elevator.updateMechanism2d();
    wrist.updateMechanism2d();
  }

  public void createSimulationThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;
              elevator.simulationPeriodic(deltaTime);
              wrist.simulationPeriodic(deltaTime);
            });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }
}
