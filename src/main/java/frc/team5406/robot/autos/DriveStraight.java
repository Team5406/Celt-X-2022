package frc.team5406.robot.autos;

import frc.team5406.robot.Constants;

import java.util.List;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;


import frc.team5406.robot.subsystems.DriveSubsystem;



public class DriveStraight {

    private final DriveSubsystem drive;
    XboxController driverGamepad = new XboxController(1);

    public DriveStraight (DriveSubsystem subsystem) {
        drive = subsystem;
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

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(

        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.5, 0, new Rotation2d(0)),
        config
    );

    

    
    
        RamseteCommand ramseteCommand = new RamseteCommand(
          
            exampleTrajectory,
            DriveSubsystem::getPose,
            new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
            Constants.DRIVE_KINEMATICS,
            drive::outputSpeeds,
            drive);



        return new SequentialCommandGroup(
            new InstantCommand(drive::reset, drive),
            ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0))
        );
      }
    
}
