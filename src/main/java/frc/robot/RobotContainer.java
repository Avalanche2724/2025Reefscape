// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
  // Subsystems
  public final Drivetrain drivetrain = TunerConstants.createDrivetrain();
  public final LED led = new LED();
  public final Intake intake = new Intake();
  public final Elevator elevator = new Elevator();
  public final Wrist wrist = new Wrist();

  // Other utility classes
  private final Controls controls = new Controls(this);
  private final Telemetry logger = new Telemetry();

  // Auto stuff
  private final AutoFactory autoFactory = drivetrain.createAutoFactory();
  private final AutoRoutines autoRoutines = new AutoRoutines(autoFactory);
  private final AutoChooser autoChooser = new AutoChooser();

  public RobotContainer() {
    // Register autos
    autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Set up drivetrain telemetry
    drivetrain.registerTelemetry(logger::telemeterize);
    // Set up controls
    controls.configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
