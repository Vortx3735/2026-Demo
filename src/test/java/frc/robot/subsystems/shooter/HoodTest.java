package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.simulation.SimHooks;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class HoodTest {

  @BeforeAll
  static void halInit() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
  }

  @Test
  public void testInitialization() {
    Hood hood =
        new Hood(
            Constants.HoodConstants.HOOD_MOTOR_ID,
            Constants.HoodConstants.HOOD_CANCODER_ID,
            Mode.SIM);
    assertNotNull(hood, "Hood should be instantiated successfully");
  }

  @Test
  public void testInitialTargetAngle() {
    Hood hood =
        new Hood(
            Constants.HoodConstants.HOOD_MOTOR_ID,
            Constants.HoodConstants.HOOD_CANCODER_ID,
            Mode.SIM);
    assertEquals(0.0, hood.targetAngle, 1e-6, "Initial targetAngle should be 0");
  }

  @Test
  public void testSetSpeed() {
    Hood hood =
        new Hood(
            Constants.HoodConstants.HOOD_MOTOR_ID,
            Constants.HoodConstants.HOOD_CANCODER_ID,
            Mode.SIM);
    hood.setSpeed(0.3);
    assertEquals(0.3, hood.speed, 1e-6, "Hood speed should be updated to 0.3");
  }

  @Test
  public void testSetPositionPIDUpdatesTargetAngle() {
    Hood hood =
        new Hood(
            Constants.HoodConstants.HOOD_MOTOR_ID,
            Constants.HoodConstants.HOOD_CANCODER_ID,
            Mode.SIM);
    hood.setPositionPID(45.0);
    assertEquals(45.0, hood.targetAngle, 1e-6, "targetAngle should be set to 45.0 degrees");
  }

  /**
   * Hood motor is configured Clockwise_Positive. A positive duty-cycle command drives the motor
   * clockwise (the "forward" direction), which the TalonFX achieves by applying a negative winding
   * voltage. After one simulation step the motor voltage is therefore negative.
   */
  @Test
  public void testSetForwardCommandDrivesMotor() {
    Hood hood =
        new Hood(
            Constants.HoodConstants.HOOD_MOTOR_ID,
            Constants.HoodConstants.HOOD_CANCODER_ID,
            Mode.SIM);
    hood.setSpeed(0.3);
    hood.motor.getSimState().setSupplyVoltage(12);
    hood.set(false);
    SimHooks.stepTiming(0.02);
    assertTrue(
        hood.motor.getSimState().getMotorVoltage() < 0,
        "Hood is Clockwise_Positive: forward command applies negative winding voltage for CW rotation");
  }

  /**
   * Reverse command sends a negative duty-cycle to the Clockwise_Positive motor, which results in
   * positive winding voltage (CCW rotation).
   */
  @Test
  public void testSetReverseCommandDrivesMotorOpposite() {
    Hood hood =
        new Hood(
            Constants.HoodConstants.HOOD_MOTOR_ID,
            Constants.HoodConstants.HOOD_CANCODER_ID,
            Mode.SIM);
    hood.setSpeed(0.3);
    hood.motor.getSimState().setSupplyVoltage(12);
    hood.set(true);
    SimHooks.stepTiming(0.02);
    assertTrue(
        hood.motor.getSimState().getMotorVoltage() > 0,
        "Hood is Clockwise_Positive: reverse command applies positive winding voltage for CCW rotation");
  }

  @Test
  public void testCommandsNotNull() {
    Hood hood =
        new Hood(
            Constants.HoodConstants.HOOD_MOTOR_ID,
            Constants.HoodConstants.HOOD_CANCODER_ID,
            Mode.SIM);
    assertNotNull(hood.moveCommand(false), "moveCommand(false) should not be null");
    assertNotNull(hood.moveCommand(true), "moveCommand(true) should not be null");
    assertNotNull(hood.stopCommand(), "stopCommand should not be null");
    assertNotNull(hood.setPositionPIDCommand(0.0), "setPositionPIDCommand should not be null");
    assertNotNull(hood.hold(), "hold should not be null");
  }
}
