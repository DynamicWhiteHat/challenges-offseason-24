package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsXbox implements DriverControls {
  private CommandXboxController m_controller;

  public DriverControlsXbox(int port) {
    m_controller = new CommandXboxController(port);
  }

  @Override
  public Trigger armExtend() {
    return m_controller.rightTrigger();
  }

  @Override
  public Trigger armRetract() {
    return m_controller.leftTrigger();
  }
}