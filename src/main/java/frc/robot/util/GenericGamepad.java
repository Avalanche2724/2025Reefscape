package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class GenericGamepad {
  /** Equivalent to × Blue Cross on PS4 */
  public Trigger a;

  /** Equivalent to ○ Red Circle on PS4 */
  public Trigger b;

  /** Equivalent to □ Purple Square on PS4 */
  public Trigger x;

  /** Equivalent to △ Green Triangle on PS4 */
  public Trigger y;

  // Also known as L1 on PS4
  public Trigger leftBumper;
  // Also known as R1 on PS4
  public Trigger rightBumper;

  public DoubleSupplier leftTrigger;
  public DoubleSupplier rightTrigger;

  // Warning: leftY is -1 when up
  public DoubleSupplier leftY;
  public DoubleSupplier leftX;
  // Warning: rightY is -1 when up
  public DoubleSupplier rightY;
  public DoubleSupplier rightX;
  public Trigger leftJoystickPushed;
  public Trigger rightJoystickPushed;
  // Equivalent to Back on Xbox or Share on PS4
  public Trigger leftMiddle;
  // Equivalent to Start on Xbox or Options on PS4
  public Trigger rightMiddle;
  public Trigger leftTriggerB;
  public Trigger rightTriggerB;
  public Trigger povLeft;
  public Trigger povRight;
  public Trigger povUp;
  public Trigger povDown;
  // Note does not work on PS4
  public DoubleConsumer rumble;
  // Equivalent to Touchpad on PS4 (No equivalent on Xbox)
  protected Trigger topMiddle;
  // Equivalent to Options on PS4 (No equivalent on Xbox)
  protected Trigger bottomMiddle;

  public GenericGamepad(CommandPS4Controller controller) {
    a = controller.cross();
    b = controller.circle();
    x = controller.square();
    y = controller.triangle();

    leftBumper = controller.L1();
    rightBumper = controller.R1();
    leftTrigger = controller::getL2Axis;
    rightTrigger = controller::getR2Axis;
    leftTriggerB = controller.L2();
    rightTriggerB = controller.R2();

    leftY = controller::getLeftY;
    leftX = controller::getLeftX;
    rightY = controller::getRightY;
    rightX = controller::getRightX;

    leftJoystickPushed = controller.L3();
    rightJoystickPushed = controller.R3();

    leftMiddle = controller.share();
    rightMiddle = controller.options();

    topMiddle = controller.touchpad();
    bottomMiddle = controller.PS();

    povLeft = controller.povLeft();
    povRight = controller.povRight();
    povUp = controller.povUp();
    povDown = controller.povDown();

    rumble = (val) -> {};
  }

  public GenericGamepad(CommandXboxController controller) {
    a = controller.a();
    b = controller.b();
    x = controller.x();
    y = controller.y();

    leftBumper = controller.leftBumper();
    rightBumper = controller.rightBumper();
    leftTrigger = controller::getLeftTriggerAxis;
    rightTrigger = controller::getRightTriggerAxis;
    rightTriggerB = controller.rightTrigger();
    leftTriggerB = controller.leftTrigger();

    leftY = controller::getLeftY;
    leftX = controller::getLeftX;
    rightY = controller::getRightY;
    rightX = controller::getRightX;

    leftJoystickPushed = controller.leftStick();
    rightJoystickPushed = controller.rightStick();

    leftMiddle = controller.back();
    rightMiddle = controller.start();

    povLeft = controller.povLeft();
    povRight = controller.povRight();
    povUp = controller.povUp();
    povDown = controller.povDown();

    topMiddle = new Trigger(() -> false);
    bottomMiddle = new Trigger(() -> false);

    rumble = (val) -> controller.getHID().setRumble(RumbleType.kBothRumble, val);
  }

  public static GenericGamepad from(int port) {
    if (Robot.isSimulation()
        && System.getProperty("os.name").toLowerCase().contains("mac")
        && false) {
      return new GenericGamepad(new CommandPS4Controller(port));
    } else {
      return new GenericGamepad(new CommandXboxController(port));
    }
  }

  public double getLeftY() {
    return leftY.getAsDouble();
  }

  public double getLeftX() {
    return leftX.getAsDouble();
  }

  public double getRightY() {
    return rightY.getAsDouble();
  }

  public double getRightX() {
    return rightX.getAsDouble();
  }
}
