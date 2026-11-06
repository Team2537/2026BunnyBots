package frc.robot.subsystems.drive.arm;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public final class ArmConstants {
  private ArmConstants() {}

  public static final double ARM_MAX_VOLTS = 12.0;

  public static final int LEFT_PIVOT_MOTOR_ID = 40;
  public static final int RIGHT_PIVOT_MOTOR_ID = 41;
  public static final int LEFT_GRABBER_MOTOR_ID = 42;
  public static final int RIGHT_GRABBER_MOTOR_ID = 43;

  public static final boolean LEFT_PIVOT_MOTOR_INVERTED = false;
  public static final boolean RIGHT_PIVOT_MOTOR_INVERTED = true;
  public static final boolean LEFT_GRABBER_MOTOR_INVERTED = false;
  public static final boolean RIGHT_GRABBER_MOTOR_INVERTED = true;

  public static final double STATUS_UPDATE_HZ = 50.0;

  public static final double PIVOT_GEAR_RATIO = 1.0;
  public static final double PIVOT_MOMENT_OF_INERTIA =
      0.025; // kg*m^2 (can also use SingleJointedArmSim.estimateMOI(m, kg))

  public static final double PIVOT_KP = 10.0;
  public static final double PIVOT_KI = 0.0;
  public static final double PIVOT_KD = 0.0;
  public static final double PIVOT_KG = 0.0;

  public static final double PIVOT_CRUISE_VELOCITY = 10.0;
  public static final double PIVOT_ACCELERATION = 1.0;

  public static final double PIVOT_STATOR_CURRENT_LIMIT_AMPS = 40.0;
  public static final double PIVOT_SUPPLY_CURRENT_LIMIT_AMPS = 30.0;

  public static final Voltage GRABBER_INTAKE_VOLTS = Volts.of(6.0);
  public static final Voltage GRABBER_RELEASE_VOLTS = Volts.of(-6.0);
  public static final Voltage GRABBER_HOLD_VOLTS = Volts.of(3.0);

  public static final double GRABBER_STATOR_CURRENT_LIMIT_AMPS = 40.0;
  public static final double GRABBER_SUPPLY_CURRENT_LIMIT_AMPS = 30.0;

  public static final double GRABBER_GEAR_RATIO = 1.0;
  public static final double GRABBER_MOMENT_OF_INERTIA = 0.01; // kg*m^2

  public static final Angle L1_ANGLE = Rotations.of(0.10);
  public static final Angle L2_ANGLE = Rotations.of(0.20);
  public static final Angle L3_ANGLE = Rotations.of(0.30);
  public static final Angle OVEN_ANGLE = Rotations.of(0.40);

  public static final TalonFXConfiguration LEFT_PIVOT_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(
                      InvertedValue.CounterClockwise_Positive)) // TODO: confirm if inverted
          .withFeedback(
              new FeedbackConfigs().withSensorToMechanismRatio(ArmConstants.PIVOT_GEAR_RATIO))
          .withSlot0(
              new Slot0Configs()
                  .withKP(ArmConstants.PIVOT_KP)
                  .withKI(ArmConstants.PIVOT_KI)
                  .withKD(ArmConstants.PIVOT_KD)
                  .withKG(ArmConstants.PIVOT_KG)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(ArmConstants.PIVOT_CRUISE_VELOCITY)
                  .withMotionMagicAcceleration(ArmConstants.PIVOT_ACCELERATION))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(ArmConstants.PIVOT_STATOR_CURRENT_LIMIT_AMPS)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(ArmConstants.PIVOT_SUPPLY_CURRENT_LIMIT_AMPS)
                  .withStatorCurrentLimitEnable(true));

  public static final TalonFXConfiguration RIGHT_PIVOT_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(
                      InvertedValue.CounterClockwise_Positive)) // TODO: confirm if inverted
          .withFeedback(
              new FeedbackConfigs().withSensorToMechanismRatio(ArmConstants.PIVOT_GEAR_RATIO))
          .withSlot0(
              new Slot0Configs()
                  .withKP(ArmConstants.PIVOT_KP)
                  .withKI(ArmConstants.PIVOT_KI)
                  .withKD(ArmConstants.PIVOT_KD))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(ArmConstants.PIVOT_CRUISE_VELOCITY)
                  .withMotionMagicAcceleration(ArmConstants.PIVOT_ACCELERATION))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(ArmConstants.PIVOT_STATOR_CURRENT_LIMIT_AMPS)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(ArmConstants.PIVOT_SUPPLY_CURRENT_LIMIT_AMPS)
                  .withStatorCurrentLimitEnable(true));

  public static final TalonFXConfiguration LEFT_GRABBER_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive)) // TODO: confirm if inverted
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(ArmConstants.PIVOT_STATOR_CURRENT_LIMIT_AMPS)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(ArmConstants.PIVOT_SUPPLY_CURRENT_LIMIT_AMPS)
                  .withStatorCurrentLimitEnable(true));

  public static final TalonFXConfiguration RIGHT_GRABBER_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive)) // TODO: confirm if inverted
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(ArmConstants.PIVOT_STATOR_CURRENT_LIMIT_AMPS)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(ArmConstants.PIVOT_SUPPLY_CURRENT_LIMIT_AMPS)
                  .withStatorCurrentLimitEnable(true));
}
