package frc.robot.subsystems.drive.arm;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {

    public double leftPivotAppliedVolts = 0.0;
    public double leftPivotPosition = 0.0;
    public double leftPivotVelocity = 0.0;
    public double leftPivotSupplyCurrent = 0.0;
    public double leftPivotStatorCurrent = 0.0;

    public double rightPivotAppliedVolts = 0.0;
    public double rightPivotPosition = 0.0;
    public double rightPivotVelocity = 0.0;
    public double rightPivotSupplyCurrent = 0.0;
    public double rightPivotStatorCurrent = 0.0;

    public double leftGrabberAppliedVolts = 0.0;
    public double leftGrabberPosition = 0.0;
    public double leftGrabberVelocity = 0.0;
    public double leftGrabberSupplyCurrent = 0.0;
    public double leftGrabberStatorCurrent = 0.0;

    public double rightGrabberAppliedVolts = 0.0;
    public double rightGrabberPosition = 0.0;
    public double rightGrabberVelocity = 0.0;
    public double rightGrabberSupplyCurrent = 0.0;
    public double rightGrabberStatorCurrent = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void grabGamePiece() {}

  public default void releaseGamePiece() {}

  public default void holdGamePiece() {}

  public default void stopArm() {
    stopPivot();
    stopGrabber();
  }

  public default void stopPivot() {}

  public default void stopGrabber() {}

  public default void setPosition(Angle position) {}

  public default void setSpeed(double speed) {}
}
