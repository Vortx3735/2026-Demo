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
    tunnel.run(false);
    assertTrue(
        tunnel.lastBottomOutput > 0, "run(false) should command a positive bottom motor output");
  }

  @Test
  public void testRunInvertedDirectionIsNegative() {
    Tunnel tunnel =
        new Tunnel(
            Constants.TunnelConstants.BOTTOM_TUNNEL_MOTOR_ID,
            Constants.TunnelConstants.TOP_TUNNEL_MOTOR_ID);
    tunnel.setTunnelSpeed(0.5, 0.5);
    tunnel.run(true);
    assertTrue(
        tunnel.lastBottomOutput < 0, "run(true) should command a negative bottom motor output");
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
