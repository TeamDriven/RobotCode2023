package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LimeLight;

public final class SubsystemInstances {
  public static final PowerDistribution pdp = new PowerDistribution(30, ModuleType.kRev);

  public static final LED LED = new LED();
  public static final Arm arm = new Arm();
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final LimeLight limelight = new LimeLight();
  public static final Elevator elevator = new Elevator();
  public static final Intake intake = new Intake(pdp);
}
