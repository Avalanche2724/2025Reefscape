package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Superstructure extends SubsystemBase {
  public enum Position {
    // Intake:
    INTAKE_CORAL_STATION(0.625, 35),
    INTAKE_VERTICAL_CORAL(0.22, -15),
    // Straight outtake:
    OUTTAKE_L1(0.53, 0),
    OUTTAKE_L2(0.927, -35),
    OUTTAKE_L3(1.3, -35),
    // Launching outtake:
    OUTTAKE_L2_LAUNCH(0.36, 35),
    OUTTAKE_L3_LAUNCH(0.74, 35),
    OUTTAKE_L4_LAUNCH(1.2, 63),
    // Vertical outtake:
    OUTTAKE_L1_VERTICAL(0.875, -45),
    OUTTAKE_L4_VERT_P1(1.676, 60),
    OUTTAKE_L4_VERT_P2(1.676, 0),
    // Algae:
    INTAKE_ALGAE_L2(0.88, 0),
    INTAKE_ALGAE_L3(1.25, 0),
    OUTTAKE_NET(1.676, 60);

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

  public Superstructure() {
    createMechanism2d();
    if (Robot.isSimulation()) {
      createSimulationThread();
    }

    elevator.setMotorDutyCycle(0);
    wrist.setMotorDutyCycle(0);
  }

  @Override
  public void periodic() {
    updateMechanism2d();
  }

  double currentElevatorTargetPosition = Elevator.MIN_HEIGHT;
  double currentWristTargetPosition = 0;

  public void setPositions(double elevatorHeight, double wristAngle) {
    currentElevatorTargetPosition = elevatorHeight;
    currentWristTargetPosition = wristAngle;

    // elevator.setMotorPosition(elevatorHeight);
    wrist.setMotorDegreesOffset(wristAngle);
  }

  public Command stop() {
    return run(
        () -> {
          elevator.setMotorDutyCycle(0);
          wrist.setMotorDutyCycle(0);
        });
  }

  public void setPositions(Position pos) {
    setPositions(pos.elevatorHeight, pos.wristAngle);
  }

  public Command goToPosition(Position pos) {
    return runOnce(() -> setPositions(pos));
  }

  public Command incrementElevator(double d) {
    return run(() -> setPositions(currentElevatorTargetPosition + d, currentWristTargetPosition));
  }

  public Command incrementWrist(double d) {
    return run(() -> setPositions(currentElevatorTargetPosition, currentWristTargetPosition + d));
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
