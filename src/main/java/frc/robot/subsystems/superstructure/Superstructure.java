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

/**
 * Superstructure subsystem that coordinates the Elevator and Wrist subsystems.
 * Manages predefined positions, visualization, and simulation.
 */
public class Superstructure extends SubsystemBase {

  //-----------------------------------------------------------------------------
  // Predefined Robot Positions
  //-----------------------------------------------------------------------------
  
  /**
   * Enumeration of predefined positions for the superstructure.
   * Each position defines both elevator height (in meters) and wrist angle (in degrees).
   */
  public enum Position {
    // Intake positions
    MIN_INTAKE_GROUND(0.165, 0),
    INTAKE_CORAL_STATION(0.625, 35),
    INTAKE_VERTICAL_CORAL(0.22, -15),
    
    // Straight outtake positions
    OUTTAKE_L1(0.53, 0),
    OUTTAKE_L2(0.927, -35),
    OUTTAKE_L3(1.3, -35),
    
    // Launching outtake positions
    OUTTAKE_L2_LAUNCH(0.56, 35),
    OUTTAKE_L3_LAUNCH(1.25, 35),
    OUTTAKE_L4_LAUNCH(1.2, 63),
    
    // Vertical outtake positions
    OUTTAKE_L1_VERTICAL(0.875, -45),
    OUTTAKE_L4_VERT_P1(1.676, 60),
    OUTTAKE_L4_VERT_P2(1.676, 0),
    
    // Algae positions
    INTAKE_ALGAE_L2(0.88, 0),
    INTAKE_ALGAE_L3(1.25, 0),
    OUTTAKE_NET(1.676, 60);

    /** Elevator height in meters */
    public final double elevatorHeight;
    
    /** Wrist angle in degrees */
    public final double wristAngle;

    /**
     * Constructs a Position with specified elevator height and wrist angle.
     *
     * @param elevatorHeight Elevator height in meters
     * @param wristAngle Wrist angle in degrees
     */
    Position(double elevatorHeight, double wristAngle) {
      this.elevatorHeight = elevatorHeight;
      this.wristAngle = wristAngle;
    }
  }

  //-----------------------------------------------------------------------------
  // Constants
  //-----------------------------------------------------------------------------
  
  /** Simulation loop period in seconds */
  private static final double SIM_LOOP_PERIOD = 0.005;
  
  /** Elevator zeroing duty cycle */
  private static final double ELEVATOR_ZEROING_DUTY_CYCLE = -0.04;
  
  //-----------------------------------------------------------------------------
  // Subsystem Instance
  //-----------------------------------------------------------------------------
  
  /** Static instance for command dependencies */
  static Subsystem instance;
  
  /** Initialize the static instance */
  {
    instance = this;
  }

  //-----------------------------------------------------------------------------
  // Components and State
  //-----------------------------------------------------------------------------
  
  /** The elevator subsystem */
  public final Elevator elevator = new Elevator();
  
  /** The wrist subsystem */
  public final Wrist wrist = new Wrist();
  
  /** Current elevator target position in meters */
  private double currentElevatorTargetPosition = Elevator.MIN_HEIGHT_M;
  
  /** Current wrist target position in degrees */
  private double currentWristTargetPosition = 0;
  
  /** Simulation notifier for running periodic simulation updates */
  private Notifier simNotifier = null;
  
  /** Last simulation time for calculating delta time */
  private double lastSimTime;

  /**
   * Creates a new Superstructure subsystem.
   * Initializes subsystems, visualization, and simulation if needed.
   */
  public Superstructure() {
    createMechanism2d();
    
    if (Robot.isSimulation()) {
      createSimulationThread();
    }
    
    // Initialize motors to stopped state
    stopMotors();
  }

  /**
   * Stops all motors in the superstructure.
   */
  private void stopMotors() {
    elevator.setMotorDutyCycle(0);
    wrist.setMotorDutyCycle(0);
  }

  @Override
  public void periodic() {
    updateMechanism2d();
  }

  //-----------------------------------------------------------------------------
  // Position Control Methods
  //-----------------------------------------------------------------------------
  
  /**
   * Sets elevator and wrist to specific positions.
   *
   * @param elevatorHeight Elevator height in meters
   * @param wristAngle Wrist angle in degrees
   */
  public void setPositions(double elevatorHeight, double wristAngle) {
    currentElevatorTargetPosition = elevatorHeight;
    currentWristTargetPosition = wristAngle;

    elevator.setMotorPosition(elevatorHeight);
    wrist.setMotorDegreesOffset(wristAngle);
  }

  /**
   * Sets the elevator and wrist to a predefined position.
   *
   * @param position The predefined position to set
   */
  public void setPositions(Position position) {
    setPositions(position.elevatorHeight, position.wristAngle);
  }

  //-----------------------------------------------------------------------------
  // Command Factory Methods
  //-----------------------------------------------------------------------------
  
  /**
   * Creates a command that stops all motors.
   *
   * @return A command that stops all motors
   */
  public Command stop() {
    return run(this::stopMotors);
  }

  /**
   * Creates a command that zeros the elevator by detecting a stall condition.
   *
   * @return A command that zeros the elevator
   */
  public Command zeroElevatorCommand() {
    return run(() -> elevator.setMotorDutyCycle(ELEVATOR_ZEROING_DUTY_CYCLE))
        .until(elevator::isStalling)
        .andThen(() -> elevator.setMotorDutyCycle(0))
        .andThen(runOnce(elevator::zero));
  }

  /**
   * Creates a command to go to a predefined position.
   *
   * @param position The position to go to
   * @return A command that moves to the specified position
   */
  public Command goToPosition(Position position) {
    return runOnce(() -> setPositions(position));
  }

  /**
   * Creates a command that incrementally adjusts the elevator height.
   *
   * @param increment The amount to increment the elevator height in meters
   * @return A command that adjusts the elevator height
   */
  public Command incrementElevator(double increment) {
    return run(() -> setPositions(
        currentElevatorTargetPosition + increment, 
        currentWristTargetPosition));
  }

  /**
   * Creates a command that incrementally adjusts the wrist angle.
   *
   * @param increment The amount to increment the wrist angle in degrees
   * @return A command that adjusts the wrist angle
   */
  public Command incrementWrist(double increment) {
    return run(() -> setPositions(
        currentElevatorTargetPosition, 
        currentWristTargetPosition + increment));
  }

  /**
   * Creates a command that zeros the wrist position.
   *
   * @return A command that zeros the wrist
   */
  public Command zeroWristCommand() {
    return runOnce(wrist::zero);
  }

  //-----------------------------------------------------------------------------
  // Simulation Methods
  //-----------------------------------------------------------------------------
  
  /**
   * Creates a thread to run the simulation at a fixed rate.
   */
  public void createSimulationThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();
    
    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier = new Notifier(() -> {
        final double currentTime = Utils.getCurrentTimeSeconds();
        double deltaTime = currentTime - lastSimTime;
        lastSimTime = currentTime;
        
        elevator.simulationPeriodic(deltaTime);
        wrist.simulationPeriodic(deltaTime);
    });
    
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }

  //-----------------------------------------------------------------------------
  // Visualization Methods
  //-----------------------------------------------------------------------------
  
  /**
   * Creates a 2D mechanism visualization for the dashboard.
   */
  public void createMechanism2d() {
    // The main mechanism object
    var mech = new Mechanism2d(Meters.convertFrom(28, Inches), 4);
    
    // The mechanism root node
    var root = mech.getRoot("root", Inches.of(19.5).in(Meters), 0);
    
    // Append elevator and wrist mechanisms
    root.append(elevator.createMechanism2d()).append(wrist.createMechanism2d());
    
    // Add to SmartDashboard
    SmartDashboard.putData("Mechanism", mech);
  }

  /**
   * Updates the 2D mechanism visualization with current positions.
   */
  public void updateMechanism2d() {
    elevator.updateMechanism2d();
    wrist.updateMechanism2d();
  }
}
