package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ArmIOSim implements ArmIO {

  private static final double LOOP_PERIOD_SEC = 0.02;
  private static final double MAX_VOLTS = 12.0;

  private static final double PIVOT_GEAR_RATIO = ArmConstants.PIVOT_GEAR_RATIO;
  private static final double GRABBER_GEAR_RATIO = 1.0;

  private static final double LEFT_PIVOT_INVERSION =
      ArmConstants.LEFT_PIVOT_MOTOR_INVERTED ? -1.0 : 1.0;
  private static final double RIGHT_PIVOT_INVERSION =
      ArmConstants.RIGHT_PIVOT_MOTOR_INVERTED ? -1.0 : 1.0;
  private static final double LEFT_GRABBER_INVERSION =
      ArmConstants.LEFT_GRABBER_MOTOR_INVERTED ? -1.0 : 1.0;
  private static final double RIGHT_GRABBER_INVERSION =
      ArmConstants.RIGHT_GRABBER_MOTOR_INVERTED ? -1.0 : 1.0;

  private static final DCMotor LEFT_PIVOT_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor RIGHT_PIVOT_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor LEFT_GRABBER_GEARBOX = DCMotor.getKrakenX44Foc(1);
  private static final DCMotor RIGHT_GRABBER_GEARBOX = DCMotor.getKrakenX44Foc(1);

  private final DCMotorSim leftPivotSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(LEFT_PIVOT_GEARBOX, 0.025, PIVOT_GEAR_RATIO),
          LEFT_PIVOT_GEARBOX);

  private final DCMotorSim rightPivotSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(RIGHT_PIVOT_GEARBOX, 0.025, PIVOT_GEAR_RATIO),
          RIGHT_PIVOT_GEARBOX);

  private final DCMotorSim leftGrabberSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(LEFT_GRABBER_GEARBOX, 0.004, GRABBER_GEAR_RATIO),
          LEFT_GRABBER_GEARBOX);

  private final DCMotorSim rightGrabberSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(RIGHT_GRABBER_GEARBOX, 0.004, GRABBER_GEAR_RATIO),
          RIGHT_GRABBER_GEARBOX);

  private final PIDController leftPivotController =
      new PIDController(ArmConstants.PIVOT_KP, ArmConstants.PIVOT_KI, ArmConstants.PIVOT_KD);

  private final PIDController rightPivotController =
      new PIDController(ArmConstants.PIVOT_KP, ArmConstants.PIVOT_KI, ArmConstants.PIVOT_KD);

  private final TrapezoidProfile pivotProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              ArmConstants.PIVOT_CRUISE_VELOCITY, ArmConstants.PIVOT_ACCELERATION));

  private double targetPivotPositionRot = 0.0;

  private TrapezoidProfile.State leftProfiledSetpoint = new TrapezoidProfile.State(0.0, 0.0);
  private TrapezoidProfile.State rightProfiledSetpoint = new TrapezoidProfile.State(0.0, 0.0);

  private boolean pivotPositionClosedLoop = false;
  private boolean pivotPositionOpenLoop = false;

  private double leftPivotRequestedVolts = 0.0;
  private double rightPivotRequestedVolts = 0.0;

  private double leftGrabberRequestedVolts = 0.0;
  private double rightGrabberRequestedVolts = 0.0;

  private double leftPivotAppliedVolts = 0.0;
  private double rightPivotAppliedVolts = 0.0;
  private double leftGrabberAppliedVolts = 0.0;
  private double rightGrabberAppliedVolts = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    double leftPositionRot = getLeftPivotPositionRotations();
    double rightPositionRot = getRightPivotPositionRotations();

    if (pivotPositionOpenLoop) {
      leftPivotAppliedVolts = MathUtil.clamp(leftPivotRequestedVolts, -MAX_VOLTS, MAX_VOLTS);

      rightPivotAppliedVolts = MathUtil.clamp(rightPivotRequestedVolts, -MAX_VOLTS, MAX_VOLTS);

    } else if (pivotPositionClosedLoop) {

      leftProfiledSetpoint = advanceProfile(leftProfiledSetpoint, targetPivotPositionRot);

      rightProfiledSetpoint = advanceProfile(rightProfiledSetpoint, targetPivotPositionRot);

      leftPivotAppliedVolts =
          MathUtil.clamp(
              leftPivotController.calculate(leftPositionRot, leftProfiledSetpoint.position),
              -MAX_VOLTS,
              MAX_VOLTS);

      rightPivotAppliedVolts =
          MathUtil.clamp(
              rightPivotController.calculate(rightPositionRot, rightProfiledSetpoint.position),
              -MAX_VOLTS,
              MAX_VOLTS);

    } else {
      leftPivotAppliedVolts = 0.0;
      rightPivotAppliedVolts = 0.0;
    }

    leftGrabberAppliedVolts = MathUtil.clamp(leftGrabberRequestedVolts, -MAX_VOLTS, MAX_VOLTS);

    rightGrabberAppliedVolts = MathUtil.clamp(rightGrabberRequestedVolts, -MAX_VOLTS, MAX_VOLTS);

    double batteryVoltage = RobotController.getBatteryVoltage();

    leftPivotAppliedVolts = MathUtil.clamp(leftPivotAppliedVolts, -batteryVoltage, batteryVoltage);

    rightPivotAppliedVolts =
        MathUtil.clamp(rightPivotAppliedVolts, -batteryVoltage, batteryVoltage);

    leftGrabberAppliedVolts =
        MathUtil.clamp(leftGrabberAppliedVolts, -batteryVoltage, batteryVoltage);

    rightGrabberAppliedVolts =
        MathUtil.clamp(rightGrabberAppliedVolts, -batteryVoltage, batteryVoltage);

    leftPivotSim.setInputVoltage(leftPivotAppliedVolts * LEFT_PIVOT_INVERSION);
    rightPivotSim.setInputVoltage(rightPivotAppliedVolts * RIGHT_PIVOT_INVERSION);
    leftGrabberSim.setInputVoltage(leftGrabberAppliedVolts * LEFT_GRABBER_INVERSION);
    rightGrabberSim.setInputVoltage(rightGrabberAppliedVolts * RIGHT_GRABBER_INVERSION);

    leftPivotSim.update(LOOP_PERIOD_SEC);
    rightPivotSim.update(LOOP_PERIOD_SEC);
    leftGrabberSim.update(LOOP_PERIOD_SEC);
    rightGrabberSim.update(LOOP_PERIOD_SEC);

    double leftPivotCurrent = Math.abs(leftPivotSim.getCurrentDrawAmps());
    double rightPivotCurrent = Math.abs(rightPivotSim.getCurrentDrawAmps());
    double leftGrabberCurrent = Math.abs(leftGrabberSim.getCurrentDrawAmps());
    double rightGrabberCurrent = Math.abs(rightGrabberSim.getCurrentDrawAmps());

    inputs.leftPivotAppliedVolts = leftPivotAppliedVolts;
    inputs.leftPivotPosition = getLeftPivotPositionRotations();
    inputs.leftPivotVelocity = leftPivotSim.getAngularVelocityRPM() / 60.0 * LEFT_PIVOT_INVERSION;
    inputs.leftPivotSupplyCurrent = leftPivotCurrent;
    inputs.leftPivotStatorCurrent = leftPivotCurrent;

    inputs.rightPivotAppliedVolts = rightPivotAppliedVolts;
    inputs.rightPivotPosition = getRightPivotPositionRotations();
    inputs.rightPivotVelocity =
        rightPivotSim.getAngularVelocityRPM() / 60.0 * RIGHT_PIVOT_INVERSION;
    inputs.rightPivotSupplyCurrent = rightPivotCurrent;
    inputs.rightPivotStatorCurrent = rightPivotCurrent;

    inputs.leftGrabberAppliedVolts = leftGrabberAppliedVolts;
    inputs.leftGrabberPosition =
        Units.radiansToRotations(leftGrabberSim.getAngularPositionRad()) * LEFT_GRABBER_INVERSION;
    inputs.leftGrabberVelocity =
        leftGrabberSim.getAngularVelocityRPM() / 60.0 * LEFT_GRABBER_INVERSION;
    inputs.leftGrabberSupplyCurrent = leftGrabberCurrent;
    inputs.leftGrabberStatorCurrent = leftGrabberCurrent;

    inputs.rightGrabberAppliedVolts = rightGrabberAppliedVolts;
    inputs.rightGrabberPosition =
        Units.radiansToRotations(rightGrabberSim.getAngularPositionRad()) * RIGHT_GRABBER_INVERSION;
    inputs.rightGrabberVelocity =
        rightGrabberSim.getAngularVelocityRPM() / 60.0 * RIGHT_GRABBER_INVERSION;
    inputs.rightGrabberSupplyCurrent = rightGrabberCurrent;
    inputs.rightGrabberStatorCurrent = rightGrabberCurrent;
  }

  @Override
  public void setPosition(Angle position) {
    targetPivotPositionRot = position.in(Rotations);
    leftProfiledSetpoint =
        new TrapezoidProfile.State(
            leftPivotSim.getAngularPositionRotations(),
            leftPivotSim.getAngularVelocityRPM() * LEFT_PIVOT_INVERSION);
    rightProfiledSetpoint =
        new TrapezoidProfile.State(
            rightPivotSim.getAngularPositionRotations(),
            rightPivotSim.getAngularVelocityRPM() * RIGHT_PIVOT_INVERSION);
    leftPivotController.reset();
    rightPivotController.reset();
    pivotPositionClosedLoop = true;
    pivotPositionOpenLoop = false;
  }

  @Override
  public void setPivotSpeed(double speed) {
    double volts = MathUtil.clamp(speed, -1, 1) * MAX_VOLTS;
    leftPivotRequestedVolts = volts;
    rightPivotRequestedVolts = volts;
    pivotPositionClosedLoop = false;
    pivotPositionOpenLoop = true;
  }

  @Override
  public void grabGamePiece() {
    leftGrabberRequestedVolts = ArmConstants.GRABBER_INTAKE_VOLTS;
    rightGrabberRequestedVolts = ArmConstants.GRABBER_INTAKE_VOLTS;
  }

  @Override
  public void releaseGamePiece() {
    leftGrabberRequestedVolts = ArmConstants.GRABBER_RELEASE_VOLTS;
    rightGrabberRequestedVolts = ArmConstants.GRABBER_RELEASE_VOLTS;
  }

  @Override
  public void holdGamePiece() {
    leftGrabberRequestedVolts = ArmConstants.GRABBER_HOLD_VOLTS;
    rightGrabberRequestedVolts = ArmConstants.GRABBER_HOLD_VOLTS;
  }

  @Override
  public void stopPivot() {
    pivotPositionClosedLoop = false;
    pivotPositionOpenLoop = false;
    leftPivotRequestedVolts = 0;
    rightPivotRequestedVolts = 0;
    leftPivotAppliedVolts = 0;
    rightPivotAppliedVolts = 0;
    leftPivotController.reset();
    rightPivotController.reset();
  }

  @Override
  public void stopGrabber() {
    leftGrabberRequestedVolts = 0;
    leftGrabberAppliedVolts = 0;
    rightGrabberRequestedVolts = 0;
    rightGrabberAppliedVolts = 0;
  }

  @Override
  public void stopArm() {
    stopGrabber();
    stopPivot();
  }

  private TrapezoidProfile.State advanceProfile(
      TrapezoidProfile.State currentSetpoint, double targetPositionRot) {

    return pivotProfile.calculate(
        LOOP_PERIOD_SEC, currentSetpoint, new TrapezoidProfile.State(targetPositionRot, 0.0));
  }

  private double getLeftPivotPositionRotations() {
    return leftPivotSim.getAngularPositionRotations();
  }

  private double getRightPivotPositionRotations() {
    return rightPivotSim.getAngularPositionRotations();
  }
}
