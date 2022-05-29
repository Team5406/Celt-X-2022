package frc.team5406.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.commands.AutoShoot;
import frc.team5406.robot.commands.AutoTurnTurret;
import frc.team5406.robot.commands.ResetHoodEncoder;
import frc.team5406.robot.commands.SpinUp;
import frc.team5406.robot.subsystems.ConveyorSubsystem;
import frc.team5406.robot.subsystems.DriveSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;



public class OneBall {

    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final ConveyorSubsystem convey;

    public OneBall (DriveSubsystem _drive, ShooterSubsystem _shooter, ConveyorSubsystem _convey) {
        drive = _drive;
        shooter = _shooter;
        convey = _convey;
      }

    public Command getAutonomousCommand() {
        DriveSubsystem.setHeading();
        drive.reset();
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.S_VOLTS,
                                       Constants.V_VOLTS,
                                       Constants.A_VOLTS),
            Constants.DRIVE_KINEMATICS,
            10);

    TrajectoryConfig config =
        new TrajectoryConfig(Constants.MAX_SPEED_METERS_PER_SECOND,
                             Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            .setKinematics(Constants.DRIVE_KINEMATICS)
            .addConstraint(autoVoltageConstraint);
            config.setReversed(false);

        /* Rotation2d endpoint = new Rotation2d();
        endpoint = Rotation2d.fromDegrees(45); */


        Trajectory drive1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(1.5, 0, new Rotation2d(0)),
            config
        );
    
        


        return   new SequentialCommandGroup(
            new InstantCommand(drive::reset, drive),
                new ParallelRaceGroup(
                    new WaitCommand(2),
                    new ParallelCommandGroup(
                        new ResetHoodEncoder(shooter),
                        new AutoTurnTurret(shooter, 108),
                        new SpinUp(shooter, 2200, 11.5) 
                    )
                ),
            
                 new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new WaitCommand(2),
                        new AutoShoot(shooter, convey)
                    ),
                    new WaitCommand(1.5)
                ),
                
    
        new RamseteCommand(
          
            drive1,
            DriveSubsystem::getPose,
            new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
            Constants.DRIVE_KINEMATICS,
            drive::outputSpeeds,
            drive
            ).andThen(() -> drive.tankDriveVolts(0, 0)
            )
        );



      }
    
}
