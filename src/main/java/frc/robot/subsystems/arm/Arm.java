package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private static final double PIVOT_TOLERANCE_ROTATIONS = 0.02;

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public void setPosition(Angle position) {
    io.setPosition(position);
  }

  public void setSpeed(double speed) {
    io.setPivotSpeed(MathUtil.clamp(speed, -1.0, 1.0));
  }

  public void stopPivot() {
    io.stopPivot();
  }

  public double getLeftPivotPosition() {
    return inputs.leftPivotPosition;
  }

  public double getRightPivotPosition() {
    return inputs.rightPivotPosition;
  }

  public double getPivotPosition() {
    return (inputs.leftPivotPosition + inputs.rightPivotPosition) / 2.0;
  }

  public boolean isAtPosition(Angle position) {
    double target = position.in(edu.wpi.first.units.Units.Rotations);

    boolean leftAtPosition =
        Math.abs(inputs.leftPivotPosition - target) <= PIVOT_TOLERANCE_ROTATIONS;

    boolean rightAtPosition =
        Math.abs(inputs.rightPivotPosition - target) <= PIVOT_TOLERANCE_ROTATIONS;

    return leftAtPosition && rightAtPosition;
  }

  public void grabGamePiece() {
    io.grabGamePiece();
  }

  public void releaseGamePiece() {
    io.releaseGamePiece();
  }

  public void holdGamePiece() {
    io.holdGamePiece();
  }

  public void stopGrabber() {
    io.stopGrabber();
  }

  public void stop() {
    io.stopArm();
  }

  public Command setPositionCommand(Angle position) {
    return Commands.runEnd(() -> setPosition(position), () -> stop(), this);
  }

  public Command scoreL1Command() {
    return setPositionCommand(ArmConstants.L1_ANGLE);
  }

  public Command scoreL2Command() {
    return setPositionCommand(ArmConstants.L2_ANGLE);
  }

  public Command scoreL3Command() {
    return setPositionCommand(ArmConstants.L3_ANGLE);
  }

  public Command scoreOvenCommand() {
    return setPositionCommand(ArmConstants.OVEN_ANGLE);
  }

  public Command manualPivotCommand(DoubleSupplier joystickInput) {
    return Commands.runEnd(() -> setSpeed(joystickInput.getAsDouble()), this::stopPivot)
        .withName("ArmManualPivot");
  }

  // Runs the grabber while the command is active.
  public void grabGamePieceCommand() {
    // return a command that runs the grabber while the command is active and stops the grabber when
    // the command ends, set name to "ArmGrabGamePiece"
  }

  // Releases the game piece while the command is active.
  public void releaseGamePieceCommand() {
    // return a command that releases the game piece while the command is active and stops the
    // grabber when the command ends, set name to "ArmReleaseGamePiece"
  }

  // Applies the grabber holding voltage.
  public void holdGamePieceCommand() {
    // return a command that applies the grabber holding voltage while the command is active, set
    // name to "ArmHoldGamePiece"

  }

  public Command stopCommand() {
    return Commands.runOnce(() -> io.stopArm(), this).withName("ArmStop");
  }
}
