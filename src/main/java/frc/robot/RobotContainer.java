// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.superstructure.Superstructure;

public class RobotContainer {
  // Subsystems
  public final Drivetrain drivetrain = TunerConstants.createDrivetrain();
  public final LED led = new LED();
  public final Intake intake = new Intake();
  public final Superstructure superstructure = new Superstructure();
  public final Climber climber = new Climber();

  // Other utility classes
  public final Controls controls = new Controls(this);
  private final Telemetry logger = new Telemetry();

  // Auto stuff
  private final AutoFactory autoFactory = drivetrain.createAutoFactory();
  private final AutoRoutines autoRoutines = new AutoRoutines(autoFactory, this);
  private final AutoChooser autoChooser = new AutoChooser();

  public RobotContainer() {
    // Register autos
    autoChooser.addRoutine("SimplePath (ONLY FORWARD)", autoRoutines::simplePathAuto);
    autoChooser.addRoutine("NOT SIMPLE PATH", autoRoutines::simplePathAuto2);
    autoChooser.addRoutine("StartToHumanStation", autoRoutines::StartToHumanStation);
    autoChooser.addRoutine("BRR FORWARD", autoRoutines::brrforward);
    autoChooser.addRoutine("verycoolpath", autoRoutines::verycoolpath);

    autoChooser.addRoutine("l1forauto_ORIG (leftmost)", autoRoutines::l1forauto_ORIG);
    autoChooser.addRoutine("l1forauto_LEFT_OPPCAGE", autoRoutines::l1forauto_LEFT);
    autoChooser.addRoutine("l1forauto_RIGHT", autoRoutines::l1forauto_RIGHT);
    autoChooser.addRoutine("l1forauto_PUSH", autoRoutines::l1forauto_PUSH);

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
