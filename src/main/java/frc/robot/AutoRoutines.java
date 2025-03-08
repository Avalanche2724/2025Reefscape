package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoRoutines {
  private final AutoFactory m_factory;

  public AutoRoutines(AutoFactory factory) {
    m_factory = factory;
  }

  public AutoRoutine simplePathAuto() {
    final AutoRoutine routine = m_factory.newRoutine("test auto");
    final AutoTrajectory simplePath = routine.trajectory("New Path");

    routine
        .active()
        .onTrue(
            Commands.print("RUNNING AUTO ROUTINE") 
                .andThen(simplePath.resetOdometry().andThen(simplePath.cmd())));

    return routine;


  }

  public AutoRoutine StartToHumanStation() {
    final AutoRoutine routine = m_factory.newRoutine("StartLeftEdgetoHumanPlayer");
    final AutoTrajectory Start_to_reef = routine.trajectory("Start to reef");
    final AutoTrajectory reef_to_lollipop = routine.trajectory("reef to lollipop");
    final AutoTrajectory lollipop_to_reef = routine.trajectory("lollipop to reef");   
    final AutoTrajectory reef_to_human_player_station = routine.trajectory("reef to human player station");

    routine.active().onTrue(
          Commands.sequence(
            Start_to_reef.resetOdometry(),
            Start_to_reef.cmd()
            
        )
      );
    





    
    
    return routine;
  }
}
