package com.team841.dory;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.team841.dory.constants.RC;
import com.team841.dory.constants.TunerConstants;
import com.team841.dory.constants.SC;
import com.team841.dory.drive.Drivetrain;
import com.team841.dory.superstructure.Escalator;
import com.team841.dory.superstructure.Escalator.Position;
import com.team841.dory.superstructure.FlapSystemAndHang;
import com.team841.dory.superstructure.MoveCommand;
import com.team841.dory.superstructure.Shooter;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Control {

    public static double scoreTimeout = 0.5;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.9).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Drivetrain drivetrain;
    private final Escalator escalator;
    private final Shooter shooter;
    private final FlapSystemAndHang flapSystem;

    public final Command snapScoreL4;
    public final Command snapScoreL3;
    public final Command snapScoreL2;

    public final Command noSnapAutoScoreL4;
    public final Command noSnapAutoScoreL3;
    public final Command noSnapAutoScoreL2;
    public final Command noSnapAutoScoreL1;

    public final Command escalatorGoHome;

    public final Command intake;

    public final Command driveCommand;

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController cojoystick = new CommandXboxController(1);

    public Control(Drivetrain drivetrain, Escalator escalator, Shooter shooter, FlapSystemAndHang flapSystemAndHang){
        this.drivetrain = drivetrain;
        this.escalator = escalator;
        this.shooter = shooter;
        this.flapSystem = flapSystemAndHang;

        if (!AutoBuilder.isConfigured()) {
            drivetrain.configureAutoBuilder();
        }

        NamedCommands.registerCommand("L2", new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator, Escalator.Position.L2,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L2, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear));
      
        NamedCommands.registerCommand("L3", new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator, Escalator.Position.L3, 
                        this.shooter::shooterHasCoral, 
                        this.shooter::escalatorClear), 
                this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout), 
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear));

        NamedCommands.registerCommand("L4", new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator, Escalator.Position.L4,
                        this.shooter::shooterHasCoral, this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear));

        NamedCommands.registerCommand("AutoL4", new SequentialCommandGroup(
                new ParallelCommandGroup(
                        AutoSnapInline(),
                        new MoveCommand(
                                this.escalator,
                                Escalator.Position.L4,
                                this.shooter::shooterHasCoral,
                                this.shooter::escalatorClear)),
                this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))
                        .withTimeout(1))
                .onlyIf(this.shooter::escalatorClear)
        );
        NamedCommands.registerCommand("GoDown",
                new InstantCommand(() -> this.escalator.setPosition(Position.HomeAndIntake, false), escalator));

        NamedCommands.registerCommand("Intake", Intake());

        NamedCommands.registerCommand("PassiveRaise",
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.Hold, true)));

        NamedCommands.registerCommand("FlapDown",
                new RunCommand(() -> this.flapSystem.setFlapperDutyCycle(-0.5), flapSystem)
                        .withTimeout(1)
                        .finallyDo(flapSystem::stopFlapper));
        
                        this.snapScoreL4 = new SequentialCommandGroup(
                AutoSnapInline(),
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L4,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.snapScoreL3 = new SequentialCommandGroup(
                AutoSnapInline(),
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L3,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.snapScoreL2 = new SequentialCommandGroup(
                AutoSnapInline(),
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L2,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(
                        Escalator.Position.L2, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.noSnapAutoScoreL4 = new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L4,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(
                        Escalator.Position.L4, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.noSnapAutoScoreL3 = new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L3,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.noSnapAutoScoreL2 = new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L2,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);
        this.noSnapAutoScoreL1 = new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L1,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L1, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.intake = Intake();

        this.escalatorGoHome =
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.HomeAndIntake,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear)
                        .onlyIf(this.shooter::escalatorClear);

        this.driveCommand = Drive(
            this.drivetrain, 
            () -> -joystick.getLeftY(),
            () -> -joystick.getLeftX(),
            () -> -joystick.getRightX());
        configureBindings();
    }

    public Command Drive(
            Drivetrain drivetrain,
            DoubleSupplier velocityX,
            DoubleSupplier velocityY,
            DoubleSupplier angularVelocity){
        
        return Commands.run(
            () -> {
                double controlX = velocityX.getAsDouble();
                double controlY = velocityY.getAsDouble();
                double controlAngularVelocity = angularVelocity.getAsDouble();
                double velX = controlX * MaxSpeed;
                double velY = controlY * MaxSpeed;
                double angularVel = controlAngularVelocity * MaxAngularRate;
                double throttleFieldFrame = RC.isRedAlliance.get() ? -velX : velX;
                double strafeFieldFrame = RC.isRedAlliance.get() ? -velY : velY;
                drivetrain.setControl(drivetrain.drive.withVelocityX(throttleFieldFrame).withVelocityY(strafeFieldFrame).withRotationalRate(angularVel));
            }, drivetrain);
    }

    /**
     * Sequences intaking that runs shooter, passive holds down the elevator, and stops when we have a coral.
     * @return Command
     */
    public Command Intake() {
        return new ConditionalCommand(
                new ParallelRaceGroup(
                        this.shooter.runShooterIntake(), this.flapSystem.runIntake()
                ), new SequentialCommandGroup(
                        this.escalator.passiveHoldDown().withTimeout(0.2), new ParallelRaceGroup(
                                this.shooter.runShooterIntake(), this.flapSystem.runIntake()
                        )

                ).onlyIf(this.shooter::escalatorClear), // Make sure that the elevator is clear before running.
                // move elevator down if not at intaking position
                () -> this.escalator.atPosition(Escalator.Position.HomeAndIntake)
        );
    }

    /**
     * Pose to Pose autoAlign command that will drive to the closest scoring command.
     * It is two ProfiledPIDControllers, one for X and one for Y, that will close the robot pose to the target pose
     * We use SwerveRequest.FieldCentricFacingAngle so that we do not have to deal with rotation
     * First we wait for the headingController to point the robot at the right direction before moving.
     * This deals with issues of profiling turning with vx and vy which is not a simple control.
     * @return AutoAlign Command
     */
    public Command AutoSnapInline(){
        ProfiledPIDController vx = drivetrain.vxController;
        ProfiledPIDController vy = drivetrain.vyController;
        AtomicReference<Pose2d> target = new AtomicReference<>();

        return Commands.runOnce(
                // Sets everything up, reset the controllers because we have a non-zero kI
                () -> {
                    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                            this.drivetrain.getState().Speeds, this.drivetrain.getState().Pose.getRotation());
                    target.set(drivetrain.getPoseToScore(this.drivetrain.getAngleToReefPolar()));
                    DogLog.log("AutoSnap/PoseTarget", target.get());
                    Translation2d translation2d = this.drivetrain.getState().Pose.getTranslation();
                    vx.reset(translation2d.getX(), fieldRelativeSpeeds.vxMetersPerSecond);
                    vy.reset(translation2d.getY(), fieldRelativeSpeeds.vyMetersPerSecond);
                }
        ).andThen(
                Commands.run(
                        // Points the drivetrain in the right direction
                        () -> this.drivetrain.setControl(
                                drivetrain.driveHeading.withTargetDirection(target.get().getRotation()))
                ).until(
                        // Wait until we are facing the right direction
                        this.drivetrain.driveHeading.HeadingController::atSetpoint
                ).andThen(
                       Commands.run(
                               () -> {
                                   ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                                           this.drivetrain.getState().Speeds, this.drivetrain.getState().Pose.getRotation());
                                   Translation2d translation2d = this.drivetrain.getState().Pose.getTranslation();

                                   // Add the current setpoint velocity to the new velocity so that it is smoother.
                                   double outputX =
                                           vx.calculate(translation2d.getX(), target.get().getX()) +
                                                   vx.getSetpoint().velocity;
                                   double outputY =
                                           vy.calculate(translation2d.getY(), target.get().getY()) +
                                                   vy.getSetpoint().velocity;

                                   // If it is not scaled, then each controller can output speeds that
                                   // when added up to the final vector will exceed the maximum real speed of the robot.
                                   outputX *= 0.7;
                                   outputY *= 0.7;

                                   // Do some logging to help with debugging
                                   DogLog.log("AutoSnap/outputX", outputX);
                                   DogLog.log("AutoSnap/outputY", outputY);
                                   DogLog.log("AutoSnap/vxAtGoal", vx.atGoal());
                                   DogLog.log("AutoSnap/vyAtGoal", vy.atGoal());

                                   this.drivetrain.setControl(
                                           this.drivetrain.driveHeading
                                                   .withVelocityY(outputY)
                                                   .withVelocityX(outputX)
                                                   .withTargetDirection(target.get().getRotation())
                                   );
                               }
                       ).until(
                               () -> vx.atGoal() && vy.atGoal()
                       ).finallyDo(
                               () -> this.drivetrain.setControl(
                                       this.drivetrain.m_robotSpeeds.withSpeeds(new ChassisSpeeds()))
                       )
                )
        );
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(driveCommand);

        joystick.start().onTrue(new InstantCommand(drivetrain::seedFieldCentric));

        /* Zero Automation */
        cojoystick.leftBumper().and(cojoystick.y()).whileTrue(noSnapAutoScoreL4);

        cojoystick.leftBumper().and(cojoystick.x()).whileTrue(noSnapAutoScoreL3);

        cojoystick.leftBumper().and(cojoystick.b()).whileTrue(noSnapAutoScoreL2);

        cojoystick.leftBumper().and(cojoystick.a()).whileTrue(noSnapAutoScoreL1);

        cojoystick.leftBumper().onFalse(escalatorGoHome);

        /* Automated */
        cojoystick.rightBumper().and(cojoystick.y()).whileTrue(snapScoreL4);

        cojoystick.rightBumper().and(cojoystick.x()).whileTrue(snapScoreL3);

        cojoystick.rightBumper().and(cojoystick.b()).whileTrue(snapScoreL2);

        cojoystick.rightBumper().onFalse(escalatorGoHome);

        /* ###################################### */

        joystick.leftTrigger().and(() -> !this.shooter.shooterHasCoral()).whileTrue(intake);

        joystick.rightTrigger(0.1).and(() -> this.escalator.getTarget().equals(Position.L4)).whileTrue(new InstantCommand(() -> this.shooter.setDutyCycle(SC.Shooter.manualShootDutyCycleL4), shooter)).onFalse(new InstantCommand(() -> this.shooter.setDutyCycle(0), shooter));
        
        joystick.rightTrigger(0.1).and(() -> !this.escalator.getTarget().equals(Position.L4)).and(() -> !this.escalator.getTarget().equals(Position.L1)).whileTrue(new InstantCommand(() -> this.shooter.setDutyCycle(SC.Shooter.manualShootDutyCycleL3L2), shooter)).onFalse(new InstantCommand(() -> this.shooter.setDutyCycle(0), shooter));
        
        joystick.rightTrigger(0.1).and(() -> this.escalator.getTarget().equals(Position.L1)).whileTrue(new InstantCommand(() -> this.shooter.setDutyCycle(SC.Shooter.manualShootDutyCycleL1), shooter)).onFalse(new InstantCommand(() -> this.shooter.setDutyCycle(0), shooter));

        cojoystick.leftTrigger().and(() -> !this.shooter.shooterHasCoral()).whileTrue(intake);

        cojoystick.rightTrigger(0.1).and(() -> this.escalator.getTarget().equals(Position.L4)).whileTrue(new InstantCommand(() -> this.shooter.setDutyCycle(SC.Shooter.manualShootDutyCycleL4), shooter)).onFalse(new InstantCommand(() -> this.shooter.setDutyCycle(0), shooter));
        
        cojoystick.rightTrigger(0.1).and(() -> !this.escalator.getTarget().equals(Position.L4)).and(() -> !this.escalator.getTarget().equals(Position.L1)).whileTrue(new InstantCommand(() -> this.shooter.setDutyCycle(SC.Shooter.manualShootDutyCycleL3L2), shooter)).onFalse(new InstantCommand(() -> this.shooter.setDutyCycle(0), shooter));
        
        cojoystick.rightTrigger(0.1).and(() -> this.escalator.getTarget().equals(Position.L1)).whileTrue(new InstantCommand(() -> this.shooter.setDutyCycle(SC.Shooter.manualShootDutyCycleL1), shooter)).onFalse(new InstantCommand(() -> this.shooter.setDutyCycle(0), shooter));

        // joystick.a().whileTrue(new InstantCommand(() -> this.flapSystem.setIntakeDutyCycle(-0.5), flapSystem)).onFalse(new InstantCommand(() -> this.flapSystem.setIntakeDutyCycle(0), flapSystem));
        // Zjoystick.a().whileTrue(new InstantCommand(() -> this.shooter.setDutyCycle(-.5), shooter)).onFalse(new InstantCommand(() -> this.shooter.setDutyCycle(0), shooter));

        // joystick.start().onTrue(new InstantCommand(escalator::zero, escalator));

        cojoystick.povDown().whileTrue(this.escalator.goDown());

        cojoystick.povUp().whileTrue(this.escalator.goUp());

        cojoystick.povLeft().whileTrue(
                new InstantCommand(() -> this.flapSystem.setFlapperDutyCycle(0.25), flapSystem))
                .onFalse(new InstantCommand(() -> this.flapSystem.stopFlapper(), flapSystem));

        cojoystick.povRight().whileTrue(
                new InstantCommand(() -> this.flapSystem.setFlapperDutyCycle(-0.25), flapSystem))
                .onFalse(new InstantCommand(() -> this.flapSystem.stopFlapper(), flapSystem));

        joystick.rightBumper().onTrue(new InstantCommand(() -> this.flapSystem.startClimb(), flapSystem));

        cojoystick.leftStick().onTrue(new InstantCommand(() -> this.shooter.setDutyCycle(-0.25))).onFalse(new InstantCommand(() -> this.shooter.setDutyCycle(0)));
        cojoystick.leftStick().onTrue(new InstantCommand(() -> this.flapSystem.setIntakeDutyCycle(-0.25))).onFalse(new InstantCommand(() -> this.flapSystem.setIntakeDutyCycle(0)));

        //joystick.leftStick().whileTrue(new InstantCommand(() -> this.flapSystem.reverseClimb(), flapSystem)).onFalse(new InstantCommand(() -> this.flapSystem.stopClimb(), flapSystem));
    }
}
