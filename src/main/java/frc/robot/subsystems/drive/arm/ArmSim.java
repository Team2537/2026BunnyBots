package frc.robot.subsystems.drive.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Simulation implementation of arm IO. */
public class ArmSim extends Arm {

  private static final double LOOP_PERIOD_SEC = 0.02;

  private static final double PIVOT_GEAR_RATIO = ArmConstants.PIVOT_GEAR_RATIO;
  private static final double GRABBER_GEAR_RATIO = 1.0;

  private static final DCMotor PIVOT_GEARBOX = DCMotor.getKrakenX60Foc(2);
  private static final DCMotor LEFT_GRABBER_GEARBOX = DCMotor.getKrakenX44Foc(1);
  private static final DCMotor RIGHT_GRABBER_GEARBOX = DCMotor.getKrakenX44Foc(1);

  // If the pivot motors move together, we can sim them as part of the same model, which supports gravity
  private final SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          PIVOT_GEARBOX,
          PIVOT_GEAR_RATIO,
          ArmConstants.PIVOT_MOMENT_OF_INERTIA,
          1.0, // should be in ArmConstants
          Degrees.of(-50).in(Radians), // should be in ArmConstants
          Degrees.of(50).in(Radians), // should be in ArmConstants
          true, // enable gravity,
          Degrees.of(0).in(Radians));

  private final DCMotorSim leftGrabberSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(LEFT_GRABBER_GEARBOX, 0.004, GRABBER_GEAR_RATIO),
          LEFT_GRABBER_GEARBOX);

  private final DCMotorSim rightGrabberSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(RIGHT_GRABBER_GEARBOX, 0.004, GRABBER_GEAR_RATIO),
          RIGHT_GRABBER_GEARBOX);

  public void simulationInit() {
    var leftPivotTalonFXSim = leftPivotMotor.getSimState();
    leftPivotTalonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
    leftPivotTalonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    var rightPivotTalonFXSim = rightPivotMotor.getSimState();
    rightPivotTalonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
    rightPivotTalonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    var leftGrabberTalonFXSim = leftGrabberMotor.getSimState();
    leftGrabberTalonFXSim.Orientation = getSimChassisReference(ArmConstants.LEFT_GRABBER_CONFIG);
    leftGrabberTalonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);

    var rightGrabberTalonFXSim = rightGrabberMotor.getSimState();
    rightGrabberTalonFXSim.Orientation = getSimChassisReference(ArmConstants.RIGHT_GRABBER_CONFIG);
    rightGrabberTalonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);
  }

  @Override
  public void simulationPeriodic() {
    var leftPivotTalonFXSim = leftPivotMotor.getSimState();
    var rightPivotTalonFXSim = rightPivotMotor.getSimState();
    var leftGrabberTalonFXSim = leftGrabberMotor.getSimState();
    var rightGrabberTalonFXSim = rightGrabberMotor.getSimState();

    double batteryVoltage = RobotController.getBatteryVoltage();
    leftPivotTalonFXSim.setSupplyVoltage(batteryVoltage);
    rightPivotTalonFXSim.setSupplyVoltage(batteryVoltage);
    leftGrabberTalonFXSim.setSupplyVoltage(batteryVoltage);
    rightGrabberTalonFXSim.setSupplyVoltage(batteryVoltage);

    pivotSim.setInputVoltage(leftPivotTalonFXSim.getMotorVoltage());
    leftGrabberSim.setInputVoltage(leftGrabberTalonFXSim.getMotorVoltage());
    rightGrabberSim.setInputVoltage(rightGrabberTalonFXSim.getMotorVoltage());

    // Advance simulations
    pivotSim.update(LOOP_PERIOD_SEC);
    leftGrabberSim.update(LOOP_PERIOD_SEC);
    rightGrabberSim.update(LOOP_PERIOD_SEC);

    leftPivotTalonFXSim.setRawRotorPosition(
        Radians.of(pivotSim.getAngleRads()).times(PIVOT_GEAR_RATIO));
    leftPivotTalonFXSim.setRotorVelocity(
        RadiansPerSecond.of(pivotSim.getVelocityRadPerSec()).times(PIVOT_GEAR_RATIO));
    System.out.println(Radians.of(pivotSim.getAngleRads()).in(Degrees));

    rightPivotTalonFXSim.setRawRotorPosition(
        Radians.of(pivotSim.getAngleRads()).times(PIVOT_GEAR_RATIO));
    rightPivotTalonFXSim.setRotorVelocity(
        RadiansPerSecond.of(pivotSim.getVelocityRadPerSec()).times(PIVOT_GEAR_RATIO));

    leftGrabberTalonFXSim.setRawRotorPosition(
        leftGrabberSim.getAngularPosition().times(GRABBER_GEAR_RATIO));
    leftGrabberTalonFXSim.setRotorVelocity(
        leftGrabberSim.getAngularVelocity().times(GRABBER_GEAR_RATIO));

    rightGrabberTalonFXSim.setRawRotorPosition(
        rightGrabberSim.getAngularPosition().times(GRABBER_GEAR_RATIO));
    rightGrabberTalonFXSim.setRotorVelocity(
        rightGrabberSim.getAngularVelocity().times(GRABBER_GEAR_RATIO));
  }

  private ChassisReference getSimChassisReference(TalonFXConfiguration config) {
    return config.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive
        ? ChassisReference.CounterClockwise_Positive
        : ChassisReference.Clockwise_Positive;
  }
}
