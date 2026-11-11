package frc.robot.subsystems.drive.arm;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.tryUntilOk;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public class ArmIOReal implements ArmIO {
  /** Creates a new ArmIOReal. */

  // random thoughts:
  // arm should be able to score l1, l2, l3, and in the oven.
  // should i assign button on controller for each?

  private static final CANBus MECHANISM_CAN_BUS = new CANBus(Constants.MECHANISM_CAN_BUS);

  private final TalonFX leftPivotMotor =
      new TalonFX(ArmConstants.LEFT_PIVOT_MOTOR_ID, MECHANISM_CAN_BUS);
  private final TalonFX rightPivotMotor =
      new TalonFX(ArmConstants.RIGHT_PIVOT_MOTOR_ID, MECHANISM_CAN_BUS);
  private final TalonFX leftGrabberMotor =
      new TalonFX(ArmConstants.LEFT_GRABBER_MOTOR_ID, MECHANISM_CAN_BUS);
  private final TalonFX rightGrabberMotor =
      new TalonFX(ArmConstants.RIGHT_GRABBER_MOTOR_ID, MECHANISM_CAN_BUS);

  private final MotionMagicVoltage leftPivotPositionRequest = new MotionMagicVoltage(0.0);
  private final MotionMagicVoltage rightPivotPositionRequest = new MotionMagicVoltage(0.0);

  private final VoltageOut leftPivotVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut rightPivotVoltageRequest = new VoltageOut(0.0);

  private final VoltageOut leftGrabberVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut rightGrabberVoltageRequest = new VoltageOut(0.0);

  private final NeutralOut neutralRequest = new NeutralOut();

  private final StatusSignal<?> leftPivotAppliedVolts;
  private final StatusSignal<?> leftPivotPosition;
  private final StatusSignal<?> leftPivotVelocity;
  private final StatusSignal<?> leftPivotSupplyCurrent;
  private final StatusSignal<?> leftPivotStatorCurrent;

  private final StatusSignal<?> rightPivotAppliedVolts;
  private final StatusSignal<?> rightPivotPosition;
  private final StatusSignal<?> rightPivotVelocity;
  private final StatusSignal<?> rightPivotSupplyCurrent;
  private final StatusSignal<?> rightPivotStatorCurrent;

  private final StatusSignal<?> leftGrabberAppliedVolts;
  private final StatusSignal<?> leftGrabberPosition;
  private final StatusSignal<?> leftGrabberVelocity;
  private final StatusSignal<?> leftGrabberSupplyCurrent;
  private final StatusSignal<?> leftGrabberStatorCurrent;

  private final StatusSignal<?> rightGrabberAppliedVolts;
  private final StatusSignal<?> rightGrabberPosition;
  private final StatusSignal<?> rightGrabberVelocity;
  private final StatusSignal<?> rightGrabberSupplyCurrent;
  private final StatusSignal<?> rightGrabberStatorCurrent;

  public ArmIOReal() {
    configureLeftPivotMotor();
    configureRightPivotMotor();
    configureLeftGrabberMotor();
    configureRightGrabberMotor();

    leftPivotMotor.setPosition(0.0);
    rightPivotMotor.setPosition(0.0);

    leftPivotAppliedVolts = leftPivotMotor.getMotorVoltage();
    leftPivotPosition = leftPivotMotor.getPosition();
    leftPivotVelocity = leftPivotMotor.getVelocity();
    leftPivotSupplyCurrent = leftPivotMotor.getSupplyCurrent();
    leftPivotStatorCurrent = leftPivotMotor.getStatorCurrent();

    rightPivotAppliedVolts = rightPivotMotor.getMotorVoltage();
    rightPivotPosition = rightPivotMotor.getPosition();
    rightPivotVelocity = rightPivotMotor.getVelocity();
    rightPivotSupplyCurrent = rightPivotMotor.getSupplyCurrent();
    rightPivotStatorCurrent = rightPivotMotor.getStatorCurrent();

    leftGrabberAppliedVolts = leftGrabberMotor.getMotorVoltage();
    leftGrabberPosition = leftGrabberMotor.getPosition();
    leftGrabberVelocity = leftGrabberMotor.getVelocity();
    leftGrabberSupplyCurrent = leftGrabberMotor.getSupplyCurrent();
    leftGrabberStatorCurrent = leftGrabberMotor.getStatorCurrent();

    rightGrabberAppliedVolts = rightGrabberMotor.getMotorVoltage();
    rightGrabberPosition = rightGrabberMotor.getPosition();
    rightGrabberVelocity = rightGrabberMotor.getVelocity();
    rightGrabberSupplyCurrent = rightGrabberMotor.getSupplyCurrent();
    rightGrabberStatorCurrent = rightGrabberMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        ArmConstants.STATUS_UPDATE_HZ,
        leftPivotAppliedVolts,
        leftPivotPosition,
        leftPivotVelocity,
        leftPivotSupplyCurrent,
        leftPivotStatorCurrent,
        rightPivotAppliedVolts,
        rightPivotPosition,
        rightPivotVelocity,
        rightPivotSupplyCurrent,
        rightPivotStatorCurrent,
        leftGrabberAppliedVolts,
        leftGrabberPosition,
        leftGrabberVelocity,
        leftGrabberSupplyCurrent,
        leftGrabberStatorCurrent,
        rightGrabberAppliedVolts,
        rightGrabberPosition,
        rightGrabberVelocity,
        rightGrabberSupplyCurrent,
        rightGrabberStatorCurrent);

    ParentDevice.optimizeBusUtilizationForAll(
        leftPivotMotor, rightPivotMotor, leftGrabberMotor, rightGrabberMotor);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftPivotAppliedVolts,
        leftPivotPosition,
        leftPivotVelocity,
        leftPivotSupplyCurrent,
        leftPivotStatorCurrent,
        rightPivotAppliedVolts,
        rightPivotPosition,
        rightPivotVelocity,
        rightPivotSupplyCurrent,
        rightPivotStatorCurrent,
        leftGrabberAppliedVolts,
        leftGrabberPosition,
        leftGrabberVelocity,
        leftGrabberSupplyCurrent,
        leftGrabberStatorCurrent,
        rightGrabberAppliedVolts,
        rightGrabberPosition,
        rightGrabberVelocity,
        rightGrabberSupplyCurrent,
        rightGrabberStatorCurrent);

    inputs.leftPivotAppliedVolts = leftPivotAppliedVolts.getValueAsDouble();
    inputs.leftPivotPosition = leftPivotPosition.getValueAsDouble();
    inputs.leftPivotVelocity = leftPivotVelocity.getValueAsDouble();
    inputs.leftPivotSupplyCurrent = leftPivotSupplyCurrent.getValueAsDouble();
    inputs.leftPivotStatorCurrent = leftPivotStatorCurrent.getValueAsDouble();

    inputs.rightPivotAppliedVolts = rightPivotAppliedVolts.getValueAsDouble();
    inputs.rightPivotPosition = rightPivotPosition.getValueAsDouble();
    inputs.rightPivotVelocity = rightPivotVelocity.getValueAsDouble();
    inputs.rightPivotSupplyCurrent = rightPivotSupplyCurrent.getValueAsDouble();
    inputs.rightPivotStatorCurrent = rightPivotStatorCurrent.getValueAsDouble();

    inputs.leftGrabberAppliedVolts = leftGrabberAppliedVolts.getValueAsDouble();
    inputs.leftGrabberPosition = leftGrabberPosition.getValueAsDouble();
    inputs.leftGrabberVelocity = leftGrabberVelocity.getValueAsDouble();
    inputs.leftGrabberSupplyCurrent = leftGrabberSupplyCurrent.getValueAsDouble();
    inputs.leftGrabberStatorCurrent = leftGrabberStatorCurrent.getValueAsDouble();

    inputs.rightGrabberAppliedVolts = rightGrabberAppliedVolts.getValueAsDouble();
    inputs.rightGrabberPosition = rightGrabberPosition.getValueAsDouble();
    inputs.rightGrabberVelocity = rightGrabberVelocity.getValueAsDouble();
    inputs.rightGrabberSupplyCurrent = rightGrabberSupplyCurrent.getValueAsDouble();
    inputs.rightGrabberStatorCurrent = rightGrabberStatorCurrent.getValueAsDouble();
  }

  // pivot control
  @Override
  public void setPosition(Angle position) {
    double targetRotations = position.in(Rotations);

    leftPivotMotor.setControl(
      leftPivotPositionRequest.withPosition(
        targetRotations));

    rightPivotMotor.setControl(
        rightPivotPositionRequest.withPosition(
            ArmConstants.RIGHT_PIVOT_MOTOR_INVERTED ? -targetRotations : targetRotations));
  }

  @Override
  public void setSpeed(double speed) {

    // create doubles for clampedSpeed (between -1 and 1) and voltage, assume 12V is max voltage
      double clampedSpeed = MathUtil.clamp(speed, -1, 1);
      double voltage = 12*clampedSpeed;

    // use control requests to set the voltage for both pivot motors, taking into account the inversion of the right pivot motor
    leftPivotMotor.setControl(leftGrabberVoltageRequest.withOutput(voltage));
    rightPivotMotor.setControl(
        rightPivotVoltageRequest.withOutput(
            ArmConstants.RIGHT_GRABBER_MOTOR_INVERTED ? -voltage : voltage));
  }

  public void stopPivot() {
    // stop the pivot motors by setting their control to neutral
      leftPivotMotor.setControl(new NeutralOut()/*?*/);
      rightPivotMotor.setControl(new NeutralOut()/*?*/);
  }

  // grabber control
  @Override
  public void grabGamePiece() {
    // set the grabber motors to intake voltage
      setGrabberVoltage(ArmConstants.GRABBER_INTAKE_VOLTS);

  }

  @Override
  public void releaseGamePiece() {
    // set the grabber motors to release voltage
      setGrabberVoltage(ArmConstants.GRABBER_RELEASE_VOLTS);


  }

  @Override
  public void holdGamePiece() {
    // set the grabber motors to hold voltage
      setGrabberVoltage(ArmConstants.GRABBER_HOLD_VOLTS);
    
  }

  private void setGrabberVoltage(double voltage) {
    double clampedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    leftGrabberMotor.setControl(leftGrabberVoltageRequest.withOutput(clampedVoltage));
    rightGrabberMotor.setControl(
        rightGrabberVoltageRequest.withOutput(
            ArmConstants.RIGHT_GRABBER_MOTOR_INVERTED ? -clampedVoltage : clampedVoltage));
  }

  public void stopGrabber() {
    // stop the grabber motors by setting their control to neutral
      leftGrabberMotor.setControl(new NeutralOut()/*?*/);
      rightGrabberMotor.setControl(new NeutralOut()/*?*/);
  }

  @Override
  public void stopArm() {
    // stop the entire arm by stopping both the pivot and grabber motors
    stopPivot();
    stopGrabber();
  }

  private void configureLeftPivotMotor() {
    // Configure the left pivot motor settings here
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted =
        ArmConstants.LEFT_PIVOT_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = ArmConstants.PIVOT_GEAR_RATIO;

    config.Slot0 =
        new Slot0Configs()
            .withKP(ArmConstants.PIVOT_KP)
            .withKI(ArmConstants.PIVOT_KI)
            .withKD(ArmConstants.PIVOT_KD);

    config.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ArmConstants.PIVOT_CRUISE_VELOCITY)
            .withMotionMagicAcceleration(ArmConstants.PIVOT_ACCELERATION);

    config.CurrentLimits.StatorCurrentLimit = ArmConstants.PIVOT_STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ArmConstants.PIVOT_SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> leftPivotMotor.getConfigurator().apply(config, 0.25));
  }

  private void configureRightPivotMotor() {
    // Configure the right pivot motor settings here
    // create talonfxconfiguration object and set the neutral mode to brake, 
    // inverted value based on the left grabber motor inverted constant, 
    // and current limits based on the grabber stator and supply current limit constants
    // apply configs using tryUntilOk with 5 retries and 0.25 timeout

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted =
        ArmConstants.RIGHT_PIVOT_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = ArmConstants.PIVOT_GEAR_RATIO;

    config.Slot0 =
        new Slot0Configs()
            .withKP(ArmConstants.PIVOT_KP)
            .withKI(ArmConstants.PIVOT_KI)
            .withKD(ArmConstants.PIVOT_KD);

    config.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ArmConstants.PIVOT_CRUISE_VELOCITY)
            .withMotionMagicAcceleration(ArmConstants.PIVOT_ACCELERATION);

    config.CurrentLimits.StatorCurrentLimit = ArmConstants.PIVOT_STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ArmConstants.PIVOT_SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> leftPivotMotor.getConfigurator().apply(config, 0.25));
    }

  private void configureLeftGrabberMotor() {
    // Configure the left grabber motor settings here
    // create talonfxconfiguration object and set the neutral mode to brake, 
    // inverted value based on the left grabber motor inverted constant, 
    // and current limits based on the grabber stator and supply current limit constants
    // apply configs using tryUntilOk with 5 retries and 0.25 timeout
    

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted =
        ArmConstants.LEFT_GRABBER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = ArmConstants.GRABBER_GEAR_RATIO;

    config.CurrentLimits.StatorCurrentLimit = ArmConstants.GRABBER_STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ArmConstants.GRABBER_SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> leftGrabberMotor.getConfigurator().apply(config, 0.25));

     }

  private void configureRightGrabberMotor() {
    // Configure the right grabber motor settings here
    // create talonfxconfiguration object and set the neutral mode to brake, 
    // inverted value based on the left grabber motor inverted constant, 
    // and current limits based on the grabber stator and supply current limit constants
    // apply configs using tryUntilOk with 5 retries and 0.25 timeout


    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted =
        ArmConstants.RIGHT_GRABBER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = ArmConstants.GRABBER_GEAR_RATIO;

    config.CurrentLimits.StatorCurrentLimit = ArmConstants.GRABBER_STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ArmConstants.GRABBER_SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> leftGrabberMotor.getConfigurator().apply(config, 0.25));

    }
}
