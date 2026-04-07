package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.WLEDController;

public class LEDSubsystem extends SubsystemBase {
  Vision vision;

  public LEDSubsystem(Vision vision) {
    this.vision = vision;
  }

  public enum LEDState {
    OFF(-1),
    IDLE(1),
    HAS_TAG(2),
    NO_TAG(3);

    private final int presetId;

    LEDState(int id) {
      this.presetId = id;
    }
  }

  private final WLEDController wled = new WLEDController(Constants.WLED_IP);
  private LEDState currentState = LEDState.OFF;

  public void setState(LEDState newState) {
    if (newState == currentState) return; // Optimization

    if (newState == LEDState.OFF) {
      wled.setPower(false);
    } else {
      wled.setPower(true);
      wled.setPreset(newState.presetId);
    }

    currentState = newState;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setState(LEDState.IDLE);
      return;
    }
    if (vision.hasTag()) {
      this.setState(LEDState.HAS_TAG);
    } else {
      this.setState(LEDState.NO_TAG);
    }
  }
}
