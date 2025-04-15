// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  public static Robot instance;
  public final RobotContainer robotContainer;
  private Command autonomousCommand;

  public Robot() {
    instance = this;
    robotContainer = new RobotContainer();
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
    autonomousCommand = robotContainer.getAutonomousCommand();

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
