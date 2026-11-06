package frc.robot.subsystems.drive.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private static final Angle PIVOT_TOLERANCE = Degrees.of(0.5);

  protected final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private static final CANBus MECHANISM_CAN_BUS = new CANBus(Constants.MECHANISM_CAN_BUS);

  protected final TalonFX leftPivotMotor =
      new TalonFX(ArmConstants.LEFT_PIVOT_MOTOR_ID, MECHANISM_CAN_BUS);
  protected final TalonFX rightPivotMotor =
      new TalonFX(ArmConstants.RIGHT_PIVOT_MOTOR_ID, MECHANISM_CAN_BUS);
  protected final TalonFX leftGrabberMotor =
      new TalonFX(ArmConstants.LEFT_GRABBER_MOTOR_ID, MECHANISM_CAN_BUS);
  protected final TalonFX rightGrabberMotor =
      new TalonFX(ArmConstants.RIGHT_GRABBER_MOTOR_ID, MECHANISM_CAN_BUS);

  private final ArmSignalManager armSignalManager =
      new ArmSignalManager(leftPivotMotor, rightPivotMotor, leftGrabberMotor, rightGrabberMotor);

  private final MotionMagicVoltage leftPivotPositionRequest = new MotionMagicVoltage(0.0);
  private final MotionMagicVoltage rightPivotPositionRequest = new MotionMagicVoltage(0.0);

  private final DutyCycleOut leftPivotDutyCycleRequest = new DutyCycleOut(0.0);
  private final DutyCycleOut rightPivotDutyCycleRequest = new DutyCycleOut(0.0);

  private final VoltageOut leftGrabberVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut rightGrabberVoltageRequest = new VoltageOut(0.0);

  public Arm() {
    configureMotor(leftPivotMotor, ArmConstants.LEFT_PIVOT_CONFIG);
    configureMotor(rightPivotMotor, ArmConstants.RIGHT_PIVOT_CONFIG);
    configureMotor(leftGrabberMotor, ArmConstants.LEFT_GRABBER_CONFIG);
    configureMotor(rightGrabberMotor, ArmConstants.RIGHT_GRABBER_CONFIG);

    leftPivotMotor.setPosition(Rotations.of(0.0));
    rightPivotMotor.setPosition(Rotations.of(0.0));
  }

  private void configureMotor(TalonFX motor, TalonFXConfiguration config) {
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void periodic() {
    armSignalManager.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  // Pivot Actions

  public void setPosition(Angle position) {
    double targetRotations = position.in(Rotations);

    leftPivotMotor.setControl(leftPivotPositionRequest.withSlot(0).withPosition(targetRotations));

    // TODO: I don't think this is right. You should be able to make the angles matching using the
    // TalonFxConfiguration
    // rightPivotMotor.setControl(
    //     rightPivotPositionRequest
    //         .withSlot(0)
    //         .withPosition(
    //             ArmConstants.RIGHT_PIVOT_MOTOR_INVERTED ? -targetRotations : targetRotations));

    rightPivotMotor.setControl(rightPivotPositionRequest.withSlot(0).withPosition(targetRotations));
  }

  public void setSpeed(double speed) {
    var clampedSpeed = MathUtil.clamp(speed, -1.0, 1.0);
    leftPivotMotor.setControl(leftPivotDutyCycleRequest.withOutput(clampedSpeed));
    rightPivotMotor.setControl(rightPivotDutyCycleRequest.withOutput(clampedSpeed));
  }

  public void stopPivot() {
    leftPivotMotor.stopMotor();
    rightPivotMotor.stopMotor();
  }

  public Angle getLeftPivotPosition() {
    return inputs.leftPivotPosition;
  }

  public Angle getRightPivotPosition() {
    return inputs.rightPivotPosition;
  }

  public Angle getPivotPosition() {
    // TODO: might be better to just trust one side
    return getLeftPivotPosition().plus(getRightPivotPosition()).div(2.0);
  }

  public boolean isAtPosition(Angle position) {
    return getPivotPosition().isNear(position, PIVOT_TOLERANCE);
  }

  public Voltage getGrabberAppliedVolts() {
    return inputs.leftGrabberAppliedVolts;
  }

  // Grabber Actions

  public void grabGamePiece() {
    leftGrabberMotor.setControl(
        leftGrabberVoltageRequest.withOutput(ArmConstants.GRABBER_INTAKE_VOLTS));
    rightGrabberMotor.setControl(
        rightGrabberVoltageRequest.withOutput(ArmConstants.GRABBER_INTAKE_VOLTS));
  }

  public void releaseGamePiece() {
    leftGrabberMotor.setControl(
        leftGrabberVoltageRequest.withOutput(ArmConstants.GRABBER_RELEASE_VOLTS));
    rightGrabberMotor.setControl(
        rightGrabberVoltageRequest.withOutput(ArmConstants.GRABBER_RELEASE_VOLTS));
  }

  public void holdGamePiece() {
    leftGrabberMotor.setControl(
        leftGrabberVoltageRequest.withOutput(ArmConstants.GRABBER_HOLD_VOLTS));
    rightGrabberMotor.setControl(
        rightGrabberVoltageRequest.withOutput(ArmConstants.GRABBER_HOLD_VOLTS));
  }

  public void stopGrabber() {
    leftGrabberMotor.stopMotor();
    rightGrabberMotor.stopMotor();
  }

  // General Actions

  public void stop() {
    stopPivot();
    stopGrabber();
  }
}
