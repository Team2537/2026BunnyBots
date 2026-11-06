// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.arm.Arm;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmCommands extends Command {

  // Factory Commands
  // Moves the pivot to a position and holds it there
  public static Command setPositionCommand(Arm arm, Angle position) {
    return Commands.runEnd(() -> arm.setPosition(position), arm::stopPivot, arm);
  }

  // Moves the arm to the L1 scoring position
  public static void scoreL1Command() {
    // returns a command that sets the arm position to L1_ANGLE, set name to "ArmScoreL1"

  }

  // Moves the arm to the L2 scoring position
  public static void scoreL2Command() {
    // returns a command that sets the arm position to L2_ANGLE

  }

  // Moves the arm to the L3 scoring position
  public static void scoreL3Command() {
    // returns a command that sets the arm position to L3_ANGLE

  }

  // Moves the arm to the oven scoring position
  public static void scoreOvenCommand() {}

  // Manual pivot control using a joystick
  public static Command manualPivotCommand(Arm arm, DoubleSupplier joystickInput) {
    return Commands.runEnd(() -> arm.setSpeed(joystickInput.getAsDouble()), arm::stopPivot, arm)
        .withName("ArmManualPivot");
  }

  // Runs the grabber while the command is active.
  public static void grabGamePieceCommand() {
    // return a command that runs the grabber while the command is active and stops the grabber when
    // the command ends, set name to "ArmGrabGamePiece"
  }

  // Releases the game piece while the command is active.
  public static void releaseGamePieceCommand() {
    // return a command that releases the game piece while the command is active and stops the
    // grabber when the command ends, set name to "ArmReleaseGamePiece"
  }

  // Applies the grabber holding voltage.
  public static void holdGamePieceCommand() {
    // return a command that applies the grabber holding voltage while the command is active, set
    // name to "ArmHoldGamePiece"
  }

  // Stops the entire arm.
  public static void stopCommand() {
    // return a command that stops the entire arm, set name to "ArmStop"
  }
}
