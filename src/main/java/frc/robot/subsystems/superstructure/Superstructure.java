package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

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

public class Superstructure extends SubsystemBase {
  public enum Position {
    // Intake:
    MIN_INTAKE_GROUND(Elevator.MIN_HEIGHT, -7.5),
    ALG_INTAKE_GROUND(0.28, 0),
    ALG_PROC(0.55, 0),

    STOW(Elevator.MIN_HEIGHT, 90),
    INTAKE_CORAL_STATION(0.75, 35),
    // Straight outtake:
    OUTTAKE_L1(0.57, 0),
    /*OUTTAKE_L2(0.927, -35),
    OUTTAKE_L3(1.3, -35),*/
    // Launching outtake:
    OUTTAKE_L2_LAUNCH(0.98, 0),
    OUTTAKE_L3_LAUNCH(1.42, 0),
    OUTTAKE_L4_LAUNCH(1.45, 80),
    // Vertical outtake:
    OUTTAKE_L1_VERTICAL(0.875, -45),
    OUTTAKE_L4_VERT_P1(1.52, 60),
    OUTTAKE_L4_VERT_P2(1.52, 0),
    // Algae:
    INTAKE_ALGAE_L2(1.15, 10),
    INTAKE_ALGAE_L3(1.52, 10),
    OUTTAKE_NET(1.52, 60);

    // Meters
    public final double elevatorHeight;
    // Degrees
    public final double wristAngle;

    Position(double elevatorHeight, double wristAngle) {
      this.elevatorHeight = elevatorHeight;
      this.wristAngle = wristAngle;
    }
  }

  // Use for command dependencies:
  static Subsystem instance;

  {
    instance = this;
  }

  public Elevator elevator = new Elevator();
  public Wrist wrist = new Wrist();

  public Trigger isStowed;

  public Superstructure() {
    createMechanism2d();
    if (Robot.isSimulation()) {
      createSimulationThread();
    }

    stopMotors();
    RobotModeTriggers.disabled().onTrue(runOnce(this::stopMotors).ignoringDisable(true));

    /*isStowed =
        new Trigger(
            () -> lastSetPosition == Position.STOW && atWristPosition(currentWristTargetPosition));

    // Auto-zero elevator when stowed
    isStowed.onTrue(zeroElevatorCommand());*/
  }

  @Override
  public void periodic() {
    updateMechanism2d();
    // TODO: implement this correctly and prevent it from causing weird movements
    /*
        // Safety threshold for crash prevention
    // private static final double ELEVATOR_SAFETY_THRESHOLD = 0.45; // meters
      if (!RobotModeTriggers.disabled().getAsBoolean()) {
        // Crash prevention
        double currentElevatorHeight = elevator.getElevatorHeight();
        double currentWristAngle = wrist.getWristDegreesOffset();
        // If we have negative wrist target but elevator isn't high enough yet
        if (currentElevatorHeight < ELEVATOR_SAFETY_THRESHOLD && currentWristTargetPosition < 0) {
          // Keep wrist at safe angle until elevator rises above threshold
          wrist.setMotorDegreesOffset(0);
          // Continue moving elevator to target
          elevator.setMotorPosition(currentElevatorTargetPosition);
        }
        // If elevator is moving down while wrist is negative, ensure elevator doesn't go below
        // threshold
        // This did not work in general; find a better solution later
        // Tbh this is not going to happen anyways
        /*
        else if (currentWristAngle < 0
            && currentElevatorTargetPosition < ELEVATOR_SAFETY_THRESHOLD
            && currentElevatorHeight > currentElevatorTargetPosition) {
          // Prevent elevator from going below safety threshold
          elevator.setMotorPosition(ELEVATOR_SAFETY_THRESHOLD);
        else {
          wrist.setMotorDegreesOffset(currentWristTargetPosition);
          elevator.setMotorPosition(currentElevatorTargetPosition);
        }
      } */
    // Run normal periodic methods
    elevator.periodic();
    wrist.periodic();
    var c = getCurrentCommand();
    if (c != null) SmartDashboard.putString("CURRENT SUP COMMAND", c.getName());
    else SmartDashboard.putString("CURRENT SUP COMMAND", "null");
  }

  Position lastSetPosition = Position.STOW;
  double currentElevatorTargetPosition = Elevator.MIN_HEIGHT;
  double currentWristTargetPosition = 0;

  private static final double ELEV_THRESHOLD = Meters.convertFrom(1, Inch);
  private static final double WRIST_THRESHOLD = Rotations.convertFrom(3, Degree);

  public boolean atPosition(Position pos) {
    return atElevatorPosition(currentElevatorTargetPosition)
        && atWristPosition(currentWristTargetPosition);
  }

  public boolean atTargetPosition() {
    return atElevatorPosition(currentElevatorTargetPosition)
        && atWristPosition(currentWristTargetPosition);
  }

  public boolean atLeastElevatorPosition(double height) {
    return elevator.getElevatorHeight() - height > 0;
  }

  public boolean atElevatorPosition(double height) {
    return Math.abs(elevator.getElevatorHeight() - height) < ELEV_THRESHOLD;
  }

  public boolean atWristPosition(double angle) {
    return Math.abs(wrist.getWristDegreesOffset() - angle) < WRIST_THRESHOLD;
  }

  public void setPositions(double elevatorHeight, double wristAngle) {
    boolean shouldStopWrist =
        false; // elevatorHeight < ELEVATOR_SAFETY_THRESHOLD && wristAngle < 0;
    currentElevatorTargetPosition = elevatorHeight;
    currentWristTargetPosition = wristAngle;

    elevator.setMotorPosition(elevatorHeight);
    wrist.setMotorDegreesOffset(shouldStopWrist ? 0 : wristAngle);
  }

  public Command setWristPositionCommand(double angle) {
    return runOnce(() -> wrist.setMotorDegreesOffset(angle));
  }

  private void stopMotors() {
    elevator.setMotorDutyCycle(0);
    wrist.setMotorDutyCycle(0);
  }

  public Command stop() {
    return run(this::stopMotors);
  }

  public Command zeroElevatorCommand() {
    return run(elevator::setMotorZeroing)
        .until(elevator.isStalling)
        .andThen(() -> elevator.setMotorDutyCycle(0))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(runOnce(elevator::zero));
  }

  public Command elevatorAlgaeLaunch(double v) {
    return run(() -> elevator.setMotorAlgaeLaunchVelocity(v));
  }

  private void setPositions(Position pos) {
    setPositions(pos.elevatorHeight, pos.wristAngle);
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

  public Command getToPositionThenHold(Supplier<Position> pos) {
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
                currentElevatorTargetPosition, currentWristTargetPosition + d.getAsDouble()));
  }

  // Simulation
  private static final double SIM_LOOP_PERIOD = 0.005;
  private Notifier simNotifier = null;
  private double m_lastSimTime;

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

  // Mechanism2d
  public void createMechanism2d() {
    // the main mechanism object
    var mech = new Mechanism2d(Meters.convertFrom(28, Inches), 4);
    // the mechanism root node
    var root = mech.getRoot("root", Inches.of(19.5).in(Meters), 0);

    root.append(elevator.createMechanism2d()).append(wrist.createMechanism2d());

    SmartDashboard.putData("Mechanism", mech);
  }

  public void updateMechanism2d() {
    elevator.updateMechanism2d();
    wrist.updateMechanism2d();
  }
}
