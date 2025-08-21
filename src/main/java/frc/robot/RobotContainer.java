// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.superstructure.Superstructure;

public class RobotContainer {
  public static RobotContainer instance;
  public final Robot robot = new Robot();
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

  private zOrchestra orchestra;

  public RobotContainer() {

    // Register autos
    autoChooser.addRoutine("Start Left - SimplePath (ONLY FORWARD)", autoRoutines::simplePathAuto);
    // autoChooser.addRoutine("NOT SIMPLE PATH", autoRoutines::simplePathAuto2);
    // autoChooser.addRoutine("StartToHumanStation", autoRoutines::StartToHumanStation);
    //  autoChooser.addRoutine("BRR FORWARD", autoRoutines::brrforward);
    autoChooser.addRoutine("verycoolpath", autoRoutines::verycoolpath);
    autoChooser.addRoutine("coolpath2back", autoRoutines::coolpath2back);
    autoChooser.addRoutine("coolpath2backsamecage", autoRoutines::coolpath2backsamecage);

    autoChooser.addRoutine("l1forauto_ORIG (leftmost)", autoRoutines::l1forauto_ORIG);
    autoChooser.addRoutine("l1forauto_LEFT_OPPCAGE", autoRoutines::l1forauto_LEFT);
    autoChooser.addRoutine("l1forauto_RIGHT", autoRoutines::l1forauto_RIGHT);
    autoChooser.addRoutine("l1forauto_PUSH", autoRoutines::l1forauto_PUSH);
    autoChooser.addRoutine("BestMiddlePathLOL", autoRoutines::BestMiddlePathLOL);
    autoChooser.addRoutine("BestMiddlePathxD_PROC", autoRoutines::BestMiddlePathxD_PROC);
    autoChooser.addRoutine("Samecage2net", autoRoutines::Samecage2net);

    autoChooser.addRoutine("RightSideL3", autoRoutines::RightSideL3);

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Set up drivetrain telemetry
    drivetrain.registerTelemetry(logger::telemeterize);
    // Set up controls
    controls.configureBindings();

    Util.configureUserButton();

    // Set up music
    /*
    orchestra = Util.configureOrchestra();

    orchestra.play();

     */
  }

  public static void main(String... args) {
    RobotBase.startRobot(() -> new RobotContainer().robot);
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }

  public class Robot extends TimedRobot {
    private Command autonomousCommand;

    {
      RobotContainer.instance = RobotContainer.this;
      System.out.println("test");
    }

    @Override
    protected void loopFunc() {
      double time = Timer.getFPGATimestamp();

      Threads.setCurrentThreadPriority(true, 2);
      super.loopFunc();
      Threads.setCurrentThreadPriority(false, 0);

      double loopTime = Timer.getFPGATimestamp() - time;
      Util.logDouble("Loop Time", loopTime * 1000);
    }

    @Override
    public void robotPeriodic() {
      CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
      autonomousCommand = getAutonomousCommand();

      if (autonomousCommand != null) {
        autonomousCommand.schedule();
      }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
      if (autonomousCommand != null) {
        autonomousCommand.cancel();
      }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
      CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationPeriodic() {}
  }
}
