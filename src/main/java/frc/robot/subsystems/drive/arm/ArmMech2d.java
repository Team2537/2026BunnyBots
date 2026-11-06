// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class ArmMech2d extends SubsystemBase {
  @AutoLogOutput(key = "Arm/ArmMech2D")
  LoggedMechanism2d m_armBase;

  LoggedMechanismRoot2d m_armRoot;
  LoggedMechanismLigament2d m_armLigament;

  Arm arm;

  /** Creates a new IntakeMech2D. */
  public ArmMech2d(Arm arm) {
    m_armBase = new LoggedMechanism2d(Inches.of(26), Inches.of(28.5));
    m_armRoot = m_armBase.getRoot("Arm", 0, 0.2);
    m_armLigament =
        m_armRoot.append(
            new LoggedMechanismLigament2d("ArmLigament", Inches.of(10), Degrees.of(90)));

    this.arm = arm;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_armLigament.setAngle(arm.getPivotPosition());
    m_armLigament.setColor(fromDutyCycle(arm.getGrabberAppliedVolts().div(12).in(Volts)));
  }

  /**
   * Returns a Color8Bit that shows scaled values for the duty cycle
   *
   * @param dutyCycle a percentage value [-1.0, 1.0] to convert to a scaled color
   */
  public static Color8Bit fromDutyCycle(double dutyCycle) {
    // If duty cycle is zero, show gray
    if (dutyCycle == 0) {
      return new Color8Bit(Color.kDimGray);
    }

    var h = dutyCycle < 0 ? 0 : 65; // red or green if negative or positive respectively
    var s = (int) Math.floor(Math.max(100.0, Math.abs(dutyCycle) * 255.0));

    return new Color8Bit(Color.fromHSV(h, s, 255));
  }
}
