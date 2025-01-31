package frc.robot.subsystems;


import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final TalonFX motor = new TalonFX(55);

  private final PositionVoltage control = new PositionVoltage(1);

  public Command goPosition(double arg) {
    return run(
        () -> {
          motor.setControl(control.withPosition(arg));
        });    
  }




  public Command goDown() {
    return goPosition(0);
  }

  public Command goUp() {
    return goPosition(5);
  }
  
  



}


