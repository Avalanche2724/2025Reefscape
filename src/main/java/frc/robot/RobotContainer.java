// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.List;
import java.util.stream.Stream;

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
    autoChooser.addRoutine("coolpath2back", autoRoutines::coolpath2back);
    autoChooser.addRoutine("coolpath2backsamecage", autoRoutines::coolpath2backsamecage);

    autoChooser.addRoutine("l1forauto_ORIG (leftmost)", autoRoutines::l1forauto_ORIG);
    autoChooser.addRoutine("l1forauto_LEFT_OPPCAGE", autoRoutines::l1forauto_LEFT);
    autoChooser.addRoutine("l1forauto_RIGHT", autoRoutines::l1forauto_RIGHT);
    autoChooser.addRoutine("l1forauto_PUSH", autoRoutines::l1forauto_PUSH);

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Set up drivetrain telemetry
    drivetrain.registerTelemetry(logger::telemeterize);
    // Set up controls
    controls.configureBindings();

    var canMotors =
        Stream.concat(
                Stream.of(11, 12, 13, 14, 21, 22, 23, 24)
                    .map(id -> new DeviceIdentifier(id, "talon fx", "CANivore")),
                Stream.of(41, 42, 51, 55, 56).map(id -> new DeviceIdentifier(id, "talon fx", "")))
            .map(TalonFXConfigurator::new)
            .toList();

    var userButton =
        new Trigger(RobotController::getUserButton)
            .and(DriverStation::isDisabled)
            .debounce(0.1, Debouncer.DebounceType.kRising)
            .debounce(6, Debouncer.DebounceType.kFalling);

    userButton.whileTrue(
        Commands.startEnd(
                () -> setMotorBrake(canMotors, false), () -> setMotorBrake(canMotors, true))
            .withTimeout(5)
            .ignoringDisable(true));
  }

  private void setMotorBrake(List<TalonFXConfigurator> configurator, boolean brake) {
    for (var conf : configurator) {
      var motorOutputConfigs = new MotorOutputConfigs();
      var retval = conf.refresh(motorOutputConfigs);
      if (retval.isOK()) {
        motorOutputConfigs.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        conf.apply(motorOutputConfigs, 0);
      }
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
