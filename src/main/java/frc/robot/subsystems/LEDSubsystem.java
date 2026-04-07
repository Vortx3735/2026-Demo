package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.WLEDController;
import frc.robot.util.WLEDController.State;

public class LEDSubsystem extends SubsystemBase {
  Vision vision;

  public LEDSubsystem(Vision vision) {
    this.vision = vision;
  }

  private final WLEDController wled = new WLEDController(Constants.WLED_IP);
  private State lastSentState = null;

  public void setRobotState(State newState) {
    // Optimization: Only send the network packet if the state actually changed
    if (newState != lastSentState) {
      wled.set(newState);
      lastSentState = newState;
    }
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setRobotState(State.IDLE);
      return;
    }
    if (vision.hasTag()) {
      this.setRobotState(WLEDController.State.HAS_TAG);
    } else {
      this.setRobotState(WLEDController.State.NO_TAG);
    }
  }
}
