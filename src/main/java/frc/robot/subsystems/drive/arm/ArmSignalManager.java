package frc.robot.subsystems.drive.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

// TODO: this doesn't really need to be it's own class, but maybe it's nicer to keep Arm cleaner
public class ArmSignalManager {
  @AutoLog
  public static class ArmIOInputs {

    public Voltage leftPivotAppliedVolts = Volts.of(0.0);
    public Angle leftPivotPosition = Rotations.of(0.0);
    public AngularVelocity leftPivotVelocity = RotationsPerSecond.of(0.0);
    public Current leftPivotSupplyCurrent = Amps.of(0.0);
    public Current leftPivotStatorCurrent = Amps.of(0.0);

    public Voltage rightPivotAppliedVolts = Volts.of(0.0);
    public Angle rightPivotPosition = Rotations.of(0.0);
    public AngularVelocity rightPivotVelocity = RotationsPerSecond.of(0.0);
    public Current rightPivotSupplyCurrent = Amps.of(0.0);
    public Current rightPivotStatorCurrent = Amps.of(0.0);

    public Voltage leftGrabberAppliedVolts = Volts.of(0.0);
    public Angle leftGrabberPosition = Rotations.of(0.0);
    public AngularVelocity leftGrabberVelocity = RotationsPerSecond.of(0.0);
    public Current leftGrabberSupplyCurrent = Amps.of(0.0);
    public Current leftGrabberStatorCurrent = Amps.of(0.0);

    public Voltage rightGrabberAppliedVolts = Volts.of(0.0);
    public Angle rightGrabberPosition = Rotations.of(0.0);
    public AngularVelocity rightGrabberVelocity = RotationsPerSecond.of(0.0);
    public Current rightGrabberSupplyCurrent = Amps.of(0.0);
    public Current rightGrabberStatorCurrent = Amps.of(0.0);
  }

  private final StatusSignal<Voltage> leftPivotAppliedVolts;
  private final StatusSignal<Angle> leftPivotPosition;
  private final StatusSignal<AngularVelocity> leftPivotVelocity;
  private final StatusSignal<Current> leftPivotSupplyCurrent;
  private final StatusSignal<Current> leftPivotStatorCurrent;

  private final StatusSignal<Voltage> rightPivotAppliedVolts;
  private final StatusSignal<Angle> rightPivotPosition;
  private final StatusSignal<AngularVelocity> rightPivotVelocity;
  private final StatusSignal<Current> rightPivotSupplyCurrent;
  private final StatusSignal<Current> rightPivotStatorCurrent;

  private final StatusSignal<Voltage> leftGrabberAppliedVolts;
  private final StatusSignal<Angle> leftGrabberPosition;
  private final StatusSignal<AngularVelocity> leftGrabberVelocity;
  private final StatusSignal<Current> leftGrabberSupplyCurrent;
  private final StatusSignal<Current> leftGrabberStatorCurrent;

  private final StatusSignal<Voltage> rightGrabberAppliedVolts;
  private final StatusSignal<Angle> rightGrabberPosition;
  private final StatusSignal<AngularVelocity> rightGrabberVelocity;
  private final StatusSignal<Current> rightGrabberSupplyCurrent;
  private final StatusSignal<Current> rightGrabberStatorCurrent;

  public ArmSignalManager(
      TalonFX leftPivotMotor,
      TalonFX rightPivotMotor,
      TalonFX leftGrabberMotor,
      TalonFX rightGrabberMotor) {
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

    inputs.leftPivotAppliedVolts = leftPivotAppliedVolts.getValue();
    inputs.leftPivotPosition = leftPivotPosition.getValue();
    inputs.leftPivotVelocity = leftPivotVelocity.getValue();
    inputs.leftPivotSupplyCurrent = leftPivotSupplyCurrent.getValue();
    inputs.leftPivotStatorCurrent = leftPivotStatorCurrent.getValue();

    inputs.rightPivotAppliedVolts = rightPivotAppliedVolts.getValue();
    inputs.rightPivotPosition = rightPivotPosition.getValue();
    inputs.rightPivotVelocity = rightPivotVelocity.getValue();
    inputs.rightPivotSupplyCurrent = rightPivotSupplyCurrent.getValue();
    inputs.rightPivotStatorCurrent = rightPivotStatorCurrent.getValue();

    inputs.leftGrabberAppliedVolts = leftGrabberAppliedVolts.getValue();
    inputs.leftGrabberPosition = leftGrabberPosition.getValue();
    inputs.leftGrabberVelocity = leftGrabberVelocity.getValue();
    inputs.leftGrabberSupplyCurrent = leftGrabberSupplyCurrent.getValue();
    inputs.leftGrabberStatorCurrent = leftGrabberStatorCurrent.getValue();

    inputs.rightGrabberAppliedVolts = rightGrabberAppliedVolts.getValue();
    inputs.rightGrabberPosition = rightGrabberPosition.getValue();
    inputs.rightGrabberVelocity = rightGrabberVelocity.getValue();
    inputs.rightGrabberSupplyCurrent = rightGrabberSupplyCurrent.getValue();
    inputs.rightGrabberStatorCurrent = rightGrabberStatorCurrent.getValue();
  }
}
