package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

public class RunIntake extends SequentialCommandGroup {
    

    public RunIntake(Intake intake, double speed) {

        addCommands(
            new ParallelDeadlineGroup(
                new WaitCommand(0.5),
                new SetIntakePosition(intake, true),
                new SpinIntake(intake, -speed)
            ),
            new InstantCommand(intake::resetIntakePosition, intake),
            new SpinIntake(intake, speed)
            // new ParallelDeadlineGroup(
            //     new WaitCommand(3.0), 
            //     new spinIntake(intake, speed)
            // ),
            // new ParallelDeadlineGroup(
            //     new WaitCommand(2.0), 
            //     new spinIntake(intake, -speed)
            // )
        );
    }
}
