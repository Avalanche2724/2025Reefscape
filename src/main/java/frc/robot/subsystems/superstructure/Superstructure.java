package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Superstructure implements Subsystem {
  public enum Position {
    A(1, 1),
    B(2, 1);

    double elevatorHeight;
    double wristAngle;

    Position(double elevatorHeight, double wristAngle) {
      this.elevatorHeight = elevatorHeight;
      this.wristAngle = wristAngle;
    }
  }

  // ONLY USE FOR SYSID!!!
  static Subsystem instance;

  {
    instance = this;
  }

  public Elevator elevator = new Elevator();
  public Wrist wrist = new Wrist();

  public void periodic() {
    updateMechanism2d();
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
