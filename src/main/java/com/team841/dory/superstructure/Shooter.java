package com.team841.dory.superstructure;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team841.dory.constants.SC;
import com.team841.dory.superstructure.Shooter.ShooterSpeeds;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

    TalonFX motor;
    CANrange FrontCANrange;
    CANrange BackCANrange;
    
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    StatusCode latestStatusCode;

    public Shooter(){
        this.motor = new TalonFX(SC.Shooter.MotorId, "rio");
        this.FrontCANrange = new CANrange(SC.Shooter.CanRangeIdFront, "rio");
        this.BackCANrange = new CANrange(SC.Shooter.CanRangeIdBack, "rio");

        this.motor.getConfigurator().apply(SC.Shooter.configs);
        this.motor.setNeutralMode(NeutralModeValue.Brake);
        this.FrontCANrange.getConfigurator().apply(SC.Shooter.CanrangeConfigs);
        this.BackCANrange.getConfigurator().apply(SC.Shooter.CanrangeConfigs);
    }

    @Override
    public void periodic() {
        DogLog.log("Shooter/isclear", this.escalatorClear());
        DogLog.log("Shooter/hasCoral", this.shooterHasCoral());
    }

    public void setDutyCycle(double output) {
        this.latestStatusCode = setControl(dutyCycle.withOutput(output));
    }

    public void setDutyCycle(ShooterSpeeds speed) {
        this.latestStatusCode = setControl(dutyCycle.withOutput(speed.getDutyCycle()));
    }

    /**
     * This checks to see if there is a coral in the shooter and is not blocking the elevator from moving. It happens
     * if the coral is too far back and will hit the rail on the elevator.
     * @return if the elevator is clear, as in the coral is not too far back. Or if there is no coral in the shooter
     * all together.
     */
    public boolean escalatorClear() {
        return (this.FrontCANrange.getDistance().getValue().magnitude() < 0.08 && this.BackCANrange.getDistance().getValue().magnitude() > 0.08)
                || (this.FrontCANrange.getDistance().getValue().magnitude() > 0.08 && this.BackCANrange.getDistance().getValue().magnitude() > 0.08);
    }

    public boolean shooterHasCoral() {
        return (this.FrontCANrange.getDistance().getValue().magnitude() < 0.08 && this.BackCANrange.getDistance().getValue().magnitude() > 0.08);
    }

    public double shooterCurrent() {
        return this.motor.getStatorCurrent().getValueAsDouble();
    }

    public StatusCode setControl(DutyCycleOut control) {
        return this.motor.setControl(control);
    }

    public void stopMotor() {
        this.motor.stopMotor();
    }

    /**
     * Command that runs the shooter at a speed that is dependent on which location it is scoring at.
     * @param atPosition checks which position the elevator is at to shoot at the right speed.
     * @param timout how long to run the shooter at the specified speed before stopping it.
     * @return Command
     */
    public Command runShooterScore(Escalator.Position atPosition, double timout) {
        return new RunCommand(
                () -> {
                    if (atPosition == Escalator.Position.L2 || atPosition == Escalator.Position.L3) {
                        setDutyCycle(ShooterSpeeds.ShootL2AndL3);
                    } else if (atPosition == Escalator.Position.L4) {
                        setDutyCycle(ShooterSpeeds.ShootL4);
                    } else if (atPosition == Escalator.Position.L1) {
                        setDutyCycle(ShooterSpeeds.ShooterL1);
                    }
                }
        ).withName("runShooterScoreCommand")
                .withTimeout(timout)
                .finallyDo(this::stopMotor);
    }

    /**
     * Runs the shooter at the intake speed until a coral is detected in the shooter.
     * @return Command
     */
    public Command runShooterIntake() {
        return new RunCommand(
                () -> {
                    this.setDutyCycle(ShooterSpeeds.Intake);
                }
        ).withName("runShooterIntakeCommand")
                .until(this::shooterHasCoral)
                .finallyDo(this::stopMotor);
    }

    /**
     * Preset speeds for the shooter motor. These are tuned and the speeds are different for each scoring location.
     */
    public enum ShooterSpeeds {
        Intake(0.15),
        Stopped(0),
        ShootL2AndL3(0.4),
        ShootL4(0.8),
        ShooterL1(0.17);

        private final double dutyCycle;

        ShooterSpeeds(double dutyCycle) {
            this.dutyCycle = dutyCycle;
        }

        public double getDutyCycle() {
            return dutyCycle;
        }
    }
}
