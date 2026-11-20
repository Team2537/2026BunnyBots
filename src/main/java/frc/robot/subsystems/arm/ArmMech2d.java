// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class ArmMech2d {
  LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);
  LoggedMechanismRoot2d root = mech.getRoot("arm", 1.5, 0);

  LoggedMechanismLigament2d base =
      root.append(new LoggedMechanismLigament2d("base", 0.5, 90, 5, new Color8Bit(235, 137, 52)));
  LoggedMechanismLigament2d forearm =
      base.append(
          new LoggedMechanismLigament2d("forearm", 0.5, -60, 5, new Color8Bit(235, 137, 52)));

  public ArmMech2d() {
    Logger.recordOutput("Mech2d", mech);
  }

  public void changeArmAngle(Angle angle) {
    forearm.setAngle(angle);
  }
}
