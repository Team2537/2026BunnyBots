package frc.robot.subsystems.drive.arm;

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

  // Pivot Actions

  public void setPosition(Angle position) {
    io.setPosition(position);
  }

  public void setSpeed(double speed) {
    // Clamp the speed between -1.0 and 1.0 and set it to the pivot motors

  }

  public void stopPivot() {
    // stop the pivot motors

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

  // Grabber Actions

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

  // General Actions

  public void stop() {
    io.stopArm();
  }

  // Factory Commands
  // Moves the pivot to a position and holds it there
  public Command setPositionCommand(Angle position) {
    // returns a command that sets the arm position to the given position and waits until it reaches
    // that position
    return Commands.runEnd(() -> setPosition(position), () -> stop(), this);
  }

  // Moves the arm to the L1 scoring position
  public Command scoreL1Command() {
    // returns a command that sets the arm position to L1_ANGLE, set name to "ArmScoreL1"
    return setPositionCommand(ArmConstants.L1_ANGLE);
  }

  // Moves the arm to the L2 scoring position
  public Command scoreL2Command() {
    // returns a command that sets the arm position to L2_ANGLE
    return setPositionCommand(ArmConstants.L2_ANGLE);
  }

  // Moves the arm to the L3 scoring position
  public Command scoreL3Command() {
    // returns a command that sets the arm position to L3_ANGLE
    return setPositionCommand(ArmConstants.L3_ANGLE);
  }

  // Moves the arm to the oven scoring position
  public void scoreOvenCommand() {}

  // Manual pivot control using a joystick
  public Command manualPivotCommand(DoubleSupplier joystickInput) {
    return runEnd(() -> setSpeed(joystickInput.getAsDouble()), this::stopPivot)
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

  // Stops the entire arm.
  public void stopCommand() {
    // return a command that stops the entire arm, set name to "ArmStop"
  }
}
