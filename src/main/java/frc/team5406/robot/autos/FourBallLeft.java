package frc.team5406.robot.autos;

import frc.team5406.robot.Constants;
import frc.team5406.robot.commands.AutoIntake;
import frc.team5406.robot.commands.AutoShoot;
import frc.team5406.robot.commands.AutoTurnTurret;
import frc.team5406.robot.commands.ResetHoodEncoder;
import frc.team5406.robot.commands.SpinUp;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team5406.robot.subsystems.ConveyorSubsystem;
import frc.team5406.robot.subsystems.DriveSubsystem;
import frc.team5406.robot.subsystems.IntakeSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;



public class FourBallLeft {

    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final ConveyorSubsystem convey;
    private final IntakeSubsystem intake;

    public FourBallLeft (DriveSubsystem _drive, ShooterSubsystem _shooter, ConveyorSubsystem _convey, IntakeSubsystem _intake) {
        drive = _drive;
        shooter = _shooter;
        convey = _convey;
        intake = _intake;
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
            new Pose2d(Units.inchesToMeters(36), 0, new Rotation2d(0)),
            config
        );



        TrajectoryConfig config2 =
        new TrajectoryConfig(Constants.MAX_SPEED_METERS_PER_SECOND,
                             Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            .setKinematics(Constants.DRIVE_KINEMATICS)
            .addConstraint(autoVoltageConstraint);
            config2.setReversed(false);

        Trajectory drive2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(36), 0, new Rotation2d(0)),
            List.of(
                new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(130))
            ),
            new Pose2d(Units.inchesToMeters(24), Units.inchesToMeters(228), new Rotation2d(60)),
            config2
        );

        TrajectoryConfig config3 =
        new TrajectoryConfig(Constants.MAX_SPEED_METERS_PER_SECOND,
                             Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            .setKinematics(Constants.DRIVE_KINEMATICS)
            .addConstraint(autoVoltageConstraint);
            config3.setReversed(false);

        Trajectory drive3 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(24), Units.inchesToMeters(200), new Rotation2d(-90)),
            List.of(),
            new Pose2d(Units.inchesToMeters(24), Units.inchesToMeters(-228), new Rotation2d(-90)),
            config3
        );
    
        
        return   new SequentialCommandGroup(
            new InstantCommand(drive::reset, drive),

            //setup hood and turret position
            new ParallelDeadlineGroup(
                //Drive Backwards to pickup the ball
                new RamseteCommand(
                drive1,
                DriveSubsystem::getPose,
                new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
                Constants.DRIVE_KINEMATICS,
                drive::outputSpeeds,
                drive
                ).andThen(() -> drive.tankDriveVolts(0, 0)
                ),
            new ResetHoodEncoder(shooter),
            new AutoTurnTurret(shooter, 103),
            new SpinUp(shooter, 2344, 15.6),
            new AutoIntake(intake, convey, true)
        ),
            new ParallelDeadlineGroup(
                new WaitCommand(3),
                new AutoShoot(shooter, convey)
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.5),
                new AutoIntake(intake, convey, false)
            ),
            new ParallelDeadlineGroup(
                new RamseteCommand(
                    drive2,
                    DriveSubsystem::getPose,
                    new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
                    Constants.DRIVE_KINEMATICS,
                    drive::outputSpeeds,
                    drive
                    ).andThen(() -> drive.tankDriveVolts(0, 0)
                    ),
                new AutoIntake(intake, convey, true)
            )/*
            new WaitCommand(0.5),
            new ParallelDeadlineGroup(
                new RamseteCommand(
                    drive3,
                    DriveSubsystem::getPose,
                    new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
                    Constants.DRIVE_KINEMATICS,
                    drive::outputSpeeds,
                    drive
                    ).andThen(() -> drive.tankDriveVolts(0, 0)
                    ),
                new SpinUp(shooter, 2000)
            ),
            new ParallelDeadlineGroup(
               new WaitCommand (2),
               new AutoShoot(shooter, convey)
            )*/
            );
                
    
        



      }
    
}
