package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class HopperTest {

  @BeforeAll
  static void halInit() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
  }

  @Test
  public void testInitialization() {
    Hopper hopper = new Hopper(Constants.HopperConstants.HOPPER_MOTOR_ID);
    assertNotNull(hopper, "Hopper should be instantiated successfully");
  }

  @Test
  public void testSetHopperSpeed() {
    Hopper hopper = new Hopper(Constants.HopperConstants.HOPPER_MOTOR_ID);
    hopper.setHopperSpeed(0.4);
    assertEquals(0.4, hopper.getHopperSpeed(), 1e-6, "Hopper speed should be updated to 0.4");
  }

  @Test
  public void testGetHopperSpeedAfterMultipleSets() {
    Hopper hopper = new Hopper(Constants.HopperConstants.HOPPER_MOTOR_ID);
    hopper.setHopperSpeed(0.1);
    assertEquals(0.1, hopper.getHopperSpeed(), 1e-6, "Hopper speed should reflect most recent set");
    hopper.setHopperSpeed(0.9);
    assertEquals(0.9, hopper.getHopperSpeed(), 1e-6, "Hopper speed should update to 0.9");
  }

  @Test
  public void testRunForwardDirectionIsPositive() {
    Hopper hopper = new Hopper(Constants.HopperConstants.HOPPER_MOTOR_ID);
    hopper.setHopperSpeed(0.5);
    hopper.hopperMotor.getSimState().setSupplyVoltage(12);
    hopper.run(false);
    assertTrue(
        hopper.hopperMotor.getSimState().getMotorVoltage() > 0,
        "Motor voltage should be positive when running forward");
  }

  @Test
  public void testRunInvertedDirectionIsNegative() {
    Hopper hopper = new Hopper(Constants.HopperConstants.HOPPER_MOTOR_ID);
    hopper.setHopperSpeed(0.5);
    hopper.hopperMotor.getSimState().setSupplyVoltage(12);
    hopper.run(true);
    assertTrue(
        hopper.hopperMotor.getSimState().getMotorVoltage() < 0,
        "Motor voltage should be negative when running inverted");
  }

  @Test
  public void testCommandsNotNull() {
    Hopper hopper = new Hopper(Constants.HopperConstants.HOPPER_MOTOR_ID);
    assertNotNull(hopper.runHopperCommand(false), "runHopperCommand(false) should not be null");
    assertNotNull(hopper.runHopperCommand(true), "runHopperCommand(true) should not be null");
    assertNotNull(hopper.stopCommand(), "stopCommand should not be null");
  }
}
