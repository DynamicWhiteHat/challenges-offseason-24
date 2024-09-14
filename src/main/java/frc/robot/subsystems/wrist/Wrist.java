package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private WristIO m_io;
  private WristInputsAutoLogged m_inputs;
  private PIDController m_Controller;
  // For instructions on how to implement this class, refer to the README.md file

  public Wrist(WristIO io, PIDController controller) {
    // TODO: Implement the constructor
    m_io = io;
    m_Controller = controller;
    m_inputs = new WristInputsAutoLogged();
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    double PID = m_Controller.calculate(m_io.getAngle().getDegrees());
    m_io.setVoltage(PID);
    // TODO: Implement this method
  }

  public void setDesiredAngle(Rotation2d angle) {
    // TODO: Implement this method
    m_Controller.setSetpoint(angle.getDegrees());
  }

  public Command setDesiredAngleCommand(Rotation2d angle) {
    // TODO: Implement this method
    return Commands.runOnce(
        () -> {
          setDesiredAngle(angle);
        });
  }

  public boolean withinTolerance() {
    // TODO: Implement this method
    return m_Controller.atSetpoint();
  }

  public WristInputsAutoLogged getInputs() {
    // TODO: Implement this method
    return m_inputs;
  }
}
