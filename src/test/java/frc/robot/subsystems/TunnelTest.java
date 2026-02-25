package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class TunnelTest {

  @BeforeAll
  static void halInit() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
  }

  @Test
  public void testInitialization() {
    Tunnel tunnel =
        new Tunnel(
            Constants.TunnelConstants.BOTTOM_TUNNEL_MOTOR_ID,
            Constants.TunnelConstants.TOP_TUNNEL_MOTOR_ID);
    assertNotNull(tunnel, "Tunnel should be instantiated successfully");
  }

  @Test
  public void testSetTunnelSpeed() {
    Tunnel tunnel =
        new Tunnel(
            Constants.TunnelConstants.BOTTOM_TUNNEL_MOTOR_ID,
            Constants.TunnelConstants.TOP_TUNNEL_MOTOR_ID);
    tunnel.setTunnelSpeed(0.6, 0.8);
    assertEquals(0.6, tunnel.getBottomTunnelSpeed(), 1e-6, "Bottom tunnel speed should be 0.6");
  }

  @Test
  public void testSetTunnelSpeedZero() {
    Tunnel tunnel =
        new Tunnel(
            Constants.TunnelConstants.BOTTOM_TUNNEL_MOTOR_ID,
            Constants.TunnelConstants.TOP_TUNNEL_MOTOR_ID);
    tunnel.setTunnelSpeed(0.0, 0.0);
    assertEquals(0.0, tunnel.getBottomTunnelSpeed(), 1e-6, "Bottom tunnel speed should be 0");
  }

  @Test
  public void testRunForwardDirectionIsPositive() {
    Tunnel tunnel =
        new Tunnel(
            Constants.TunnelConstants.BOTTOM_TUNNEL_MOTOR_ID,
            Constants.TunnelConstants.TOP_TUNNEL_MOTOR_ID);
    tunnel.setTunnelSpeed(0.5, 0.5);
    tunnel.bottomTunnelMotor.getSimState().setSupplyVoltage(12);
    tunnel.run(false);
    assertTrue(
        tunnel.bottomTunnelMotor.getSimState().getMotorVoltage() > 0,
        "Bottom motor voltage should be positive when running forward");
  }

  @Test
  public void testRunInvertedDirectionIsNegative() {
    Tunnel tunnel =
        new Tunnel(
            Constants.TunnelConstants.BOTTOM_TUNNEL_MOTOR_ID,
            Constants.TunnelConstants.TOP_TUNNEL_MOTOR_ID);
    tunnel.setTunnelSpeed(0.5, 0.5);
    tunnel.bottomTunnelMotor.getSimState().setSupplyVoltage(12);
    tunnel.run(true);
    assertTrue(
        tunnel.bottomTunnelMotor.getSimState().getMotorVoltage() < 0,
        "Bottom motor voltage should be negative when running inverted");
  }

  @Test
  public void testCommandsNotNull() {
    Tunnel tunnel =
        new Tunnel(
            Constants.TunnelConstants.BOTTOM_TUNNEL_MOTOR_ID,
            Constants.TunnelConstants.TOP_TUNNEL_MOTOR_ID);
    assertNotNull(tunnel.runTunnelCommand(false), "runTunnelCommand(false) should not be null");
    assertNotNull(tunnel.runTunnelCommand(true), "runTunnelCommand(true) should not be null");
    assertNotNull(tunnel.stopCommand(), "stopCommand should not be null");
  }
}
