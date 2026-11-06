package frc.robot.subsystems.drive.arm;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public final class ArmConstants {
  private ArmConstants() {}

  public static final double ARM_MAX_VOLTS = 12.0;

  public static final int LEFT_PIVOT_MOTOR_ID = 0;
  public static final int RIGHT_PIVOT_MOTOR_ID = 0;
  public static final int LEFT_GRABBER_MOTOR_ID = 0;
  public static final int RIGHT_GRABBER_MOTOR_ID = 0;

  public static final boolean LEFT_PIVOT_MOTOR_INVERTED = false;
  public static final boolean RIGHT_PIVOT_MOTOR_INVERTED = true;
  public static final boolean LEFT_GRABBER_MOTOR_INVERTED = false;
  public static final boolean RIGHT_GRABBER_MOTOR_INVERTED = true;

  public static final double STATUS_UPDATE_HZ = 50.0;

  public static final double PIVOT_GEAR_RATIO = 1.0;
  public static final double PIVOT_MOMENT_OF_INERTIA = 0.025; // kg*m^2

  public static final double PIVOT_KP = 0.0;
  public static final double PIVOT_KI = 0.0;
  public static final double PIVOT_KD = 0.0;

  public static final double PIVOT_CRUISE_VELOCITY = 0.0;
  public static final double PIVOT_ACCELERATION = 0.0;

  public static final double PIVOT_STATOR_CURRENT_LIMIT_AMPS = 40.0;
  public static final double PIVOT_SUPPLY_CURRENT_LIMIT_AMPS = 30.0;

  public static final double GRABBER_INTAKE_VOLTS = 6.0;
  public static final double GRABBER_RELEASE_VOLTS = -6.0;
  public static final double GRABBER_HOLD_VOLTS = 3.0;

  public static final double GRABBER_STATOR_CURRENT_LIMIT_AMPS = 40.0;
  public static final double GRABBER_SUPPLY_CURRENT_LIMIT_AMPS = 30.0;

  public static final double GRABBER_GEAR_RATIO = 1.0;
  public static final double GRABBER_MOMENT_OF_INERTIA = 0.01; // kg*m^2

  public static final Angle L1_ANGLE = Rotations.of(0.10);
  public static final Angle L2_ANGLE = Rotations.of(0.20);
  public static final Angle L3_ANGLE = Rotations.of(0.30);
  public static final Angle OVEN_ANGLE = Rotations.of(0.40);
}
