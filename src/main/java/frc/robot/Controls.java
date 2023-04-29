package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.XboxController;

public final class Controls {
    public static final XboxController m_controller = new XboxController(0);

    public static final DoubleSupplier xMoveControl = m_controller::getRightY;
    public static final DoubleSupplier yMoveControl = m_controller::getRightX;
    public static final DoubleSupplier turnControl = m_controller::getLeftX;

    public static final BooleanSupplier lockWheelsControl = Controls::getRightTrigger;
    public static final BooleanSupplier autoLineUpControl = Controls::getLeftTrigger;

    public static final BooleanSupplier changeModeControl = m_controller::getLeftStickButton;

    public static final BooleanSupplier prePlaceControl = m_controller::getRightStickButton;

    public static final BooleanSupplier intakeControl = m_controller::getRightBumper;
    public static final BooleanSupplier outtakeControl = m_controller::getLeftBumper;

    public static final BooleanSupplier moveElevatorUpControl = m_controller::getXButton;
    public static final BooleanSupplier moveElevatorDownControl = m_controller::getBButton;

    public static final BooleanSupplier moveClawUpControl = m_controller::getYButton;
    public static final BooleanSupplier moveClawDownControl = m_controller::getAButton;

    public static final BooleanSupplier zeroRobotControl = m_controller::getStartButton;

    public static final BooleanSupplier autoPlaceHighControl = Controls::getDpadUp;
    public static final BooleanSupplier autoPlaceMidControl = Controls::getDpadRight;
    public static final BooleanSupplier autoTuckControl = Controls::getDpadDown;
    // public static final BooleanSupplier autoPickUpControl = Controls::getDpadDown;
    public static final BooleanSupplier autoSubstationConePickUpControl = Controls::getDpadLeft;
    public static final BooleanSupplier autoDoubleSubstationConePickUpControl = m_controller::getBackButton;


    public static boolean getDpadUp() {
        if (m_controller.getPOV() == 0) {
          return true;
        } else {
          return false;
        }
      }
    
      public static boolean getDpadRight() {
        if (m_controller.getPOV() == 90) {
          return true;
        } else {
          return false;
        }
      }
    
      public static boolean getDpadDown() {
        if (m_controller.getPOV() == 180) {
          return true;
        } else {
          return false;
        }
      }
    
      public static boolean getDpadLeft() {
        if (m_controller.getPOV() == 270) {
          return true;
        } else {
          return false;
        }
      }
    
      public static boolean getRightTrigger() {
        if (m_controller.getRightTriggerAxis() > 0.5) {
          return true;
        } else {
          return false;
        }
      }
    
      public static boolean getLeftTrigger() {
        if (m_controller.getLeftTriggerAxis() > 0.5) {
          return true;
        } else {
          return false;
        }
      }
}
