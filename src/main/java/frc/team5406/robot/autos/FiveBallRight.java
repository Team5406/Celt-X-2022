package frc.team5406.robot.autos;

import frc.team5406.robot.Constants;
import frc.team5406.robot.commands.AutoIntake;
import frc.team5406.robot.commands.AutoShoot;
import frc.team5406.robot.commands.AutoStopBoosterTopConvey;
import frc.team5406.robot.commands.AutoTurnTurret;
import frc.team5406.robot.commands.ResetHoodEncoder;
import frc.team5406.robot.commands.SpinUp;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.subsystems.ConveyorSubsystem;
import frc.team5406.robot.subsystems.DriveSubsystem;
import frc.team5406.robot.subsystems.IntakeSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;



public class FiveBallRight {

    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final ConveyorSubsystem convey;
    private final IntakeSubsystem intake;

    public FiveBallRight (DriveSubsystem _drive, ShooterSubsystem _shooter, ConveyorSubsystem _convey, IntakeSubsystem _intake) {
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

    //Drive backwards and pickup ball one
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
            config2.setReversed(true);

        Trajectory drive2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(36), 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(36), new Rotation2d(Units.degreesToRadians(-45))),
            config2
        );

        TrajectoryConfig config3 =
        new TrajectoryConfig(Constants.MAX_SPEED_METERS_PER_SECOND,
                             Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            .setKinematics(Constants.DRIVE_KINEMATICS)
            .addConstraint(autoVoltageConstraint);
            config3.setReversed(false);

        Trajectory drive3 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(36), new Rotation2d(Units.degreesToRadians(-45))),
            List.of(),
            new Pose2d(Units.inchesToMeters(-4), Units.inchesToMeters(-84), new Rotation2d(Units.degreesToRadians(-90))),
            config3
        );
    
        TrajectoryConfig config4 =
        new TrajectoryConfig(Constants.MAX_SPEED_METERS_PER_SECOND,
                             Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            .setKinematics(Constants.DRIVE_KINEMATICS)
            .addConstraint(autoVoltageConstraint);
            config4.setReversed(false);

        Trajectory drive4 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-4), Units.inchesToMeters(-84), new Rotation2d(Units.degreesToRadians(-90))),
            List.of(),
            new Pose2d(Units.inchesToMeters(10), Units.inchesToMeters(-233), new Rotation2d(Units.degreesToRadians(-45))),
            config4
        );
            
        //Drive foward 
        TrajectoryConfig config5 =
        new TrajectoryConfig(Constants.MAX_SPEED_METERS_PER_SECOND,
        Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            .setKinematics(Constants.DRIVE_KINEMATICS)
            .addConstraint(autoVoltageConstraint);
            config5.setReversed(true);

        Trajectory drive5 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(10), Units.inchesToMeters(-233), new Rotation2d(Units.degreesToRadians(-45))),
            List.of(),
            new Pose2d(Units.inchesToMeters(4), Units.inchesToMeters(-108), new Rotation2d(Units.degreesToRadians(-90))),
            config5
        );
        
        return   
        new ParallelDeadlineGroup(
        new SequentialCommandGroup(
            new InstantCommand(drive::reset, drive),
            new ParallelCommandGroup(
                new ParallelRaceGroup(
                    new WaitCommand(1.5), 
                    new ParallelCommandGroup(
                        new ResetHoodEncoder(shooter),
                        new AutoTurnTurret(shooter, 103),
                        new SpinUp(shooter, 2344, 15.6)
                    )
                   
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
            ),
         
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new WaitCommand(1),
                    new AutoShoot(shooter, convey)
                ),
                new WaitCommand(.75),
                new AutoStopBoosterTopConvey(shooter, convey)
            ),
           

                new RamseteCommand(
                    drive2,
                    DriveSubsystem::getPose,
                    new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
                    Constants.DRIVE_KINEMATICS,
                    drive::outputSpeeds,
                    drive
                    ).andThen(
                          
            new ParallelCommandGroup(
                new ParallelRaceGroup(
                    new WaitCommand(1.5),
                    new ParallelCommandGroup(
                        new AutoTurnTurret(shooter, 48.8),
                        new SpinUp(shooter, 2391, 16.85)
                    )
                ),
                new RamseteCommand(
                    drive3,
                    DriveSubsystem::getPose,
                    new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
                    Constants.DRIVE_KINEMATICS,
                    drive::outputSpeeds,
                    drive
                    ).andThen(() -> drive.tankDriveVolts(0, 0)
                    )
            )
        ),
       
               

          
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new WaitCommand(1),
                    new AutoShoot(shooter, convey)
                ),
                new WaitCommand(0.75),
                new AutoStopBoosterTopConvey(shooter, convey)
            ),

            new ParallelDeadlineGroup(
                new RamseteCommand(
                drive4,
                DriveSubsystem::getPose,
                new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
                Constants.DRIVE_KINEMATICS,
                drive::outputSpeeds,
                drive
                ).andThen(() -> drive.tankDriveVolts(0, 0)
                ),
                new SpinUp(shooter, 2410, 17.5) //turns off upper conveyor
            ),
                new WaitCommand(0.6), //wait at terminal to miss ball
            new ParallelDeadlineGroup(
            
            new RamseteCommand(
                drive5,
                DriveSubsystem::getPose,
                new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
                Constants.DRIVE_KINEMATICS,
                drive::outputSpeeds,
                drive
                ).andThen(() -> drive.tankDriveVolts(0, 0)
                ),
                new AutoTurnTurret(shooter, 51.9),
                new SpinUp(shooter, 2500, 19.7)
            ),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new WaitCommand(1),
                    new AutoShoot(shooter, convey)
                ),
                new WaitCommand(1),
                new AutoStopBoosterTopConvey(shooter, convey)
            ),
            new SpinUp(shooter, 0, 0)
        ),
        new AutoIntake(intake, convey, true)
        );
            

      }
    
}
