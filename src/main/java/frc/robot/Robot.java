// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.photonvision.estimation.OpenCVHelp;

public class Robot extends TimedRobot {
  public static Robot instance;
  public final RobotContainer robotContainer;
  private Command autonomousCommand;

  public Robot() {
    OpenCVHelp.forceLoadOpenCV();
    instance = this;
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // DataLogManager.start();
    // DataLogManager.logNetworkTables(false);
    double time = Timer.getFPGATimestamp();
    Threads.setCurrentThreadPriority(true, 2);
    CommandScheduler.getInstance().run();
    Threads.setCurrentThreadPriority(false, 0);
    double loopTime = Timer.getFPGATimestamp() - time;
    SmartDashboard.putNumber("Loop Time ms", loopTime * 1000);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
