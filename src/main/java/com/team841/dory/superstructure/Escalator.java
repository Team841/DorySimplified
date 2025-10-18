package com.team841.dory.superstructure;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team841.dory.constants.RC;
import com.team841.dory.constants.SC;

import dev.doglog.DogLog;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Escalator extends SubsystemBase{

    TalonFX leftMotor = new TalonFX(SC.Escalator.left, RC.canivoreCANBus);
    TalonFX rightMotor = new TalonFX(SC.Escalator.right, RC.canivoreCANBus);

    Follower leftFollower = new Follower(SC.Escalator.right, true);

    MotionMagicTorqueCurrentFOC withOutCoralControl = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
    MotionMagicTorqueCurrentFOC withCoralControl = new MotionMagicTorqueCurrentFOC(0).withSlot(1);
    DutyCycleOut dutyCycle = new DutyCycleOut(0);

    StatusCode[] latestStatus;

    Position targetPosition = Position.HomeAndIntake;

    public Escalator(){
        this.leftMotor.getConfigurator().apply(SC.Escalator.leftConfigs);
        this.rightMotor.getConfigurator().apply(SC.Escalator.rightConfigs);

        zero();

        this.leftMotor.setControl(leftFollower);
    }

    @Override
    public void periodic(){
        Angle rightMotorPos = rightMotor.getPosition().getValue();
        DogLog.log("Escalator/AtHome", this.atPosition(Position.HomeAndIntake));
        DogLog.log("Escalator/TargetPosition", this.targetPosition.toString());
        DogLog.log("Escalator/PositionRadian", rightMotorPos.in(Units.Rotation));
        DogLog.log("Escalator/Logical Check",
                this.targetPosition == Position.HomeAndIntake &&
                        Math.abs(rightMotorPos.in(Units.Rotation) - Position.HomeAndIntake.getPosition()) < 2
                        && !(rightMotorPos.in(Units.Rotation) < 0.01));
    }

     /**
     * The coral has non-negligible weight, there is two PID loops, one for with coral, and one without
     * @param position target preset location
     * @param hasCoral whether the shooter is using the coral control or not
     */
    public void setPosition(Position position, boolean hasCoral) {
        if (hasCoral) {
            this.latestStatus = setControl(withCoralControl.withPosition(position.getPosition()));
        } else {
            this.latestStatus = setControl(withOutCoralControl.withPosition(position.getPosition()));
        }

        this.targetPosition = position;
    }

    public void zero() {
        this.resetPositions(0.0, 0.0);
    }

    public boolean atPosition(Position position) {
        return Math.abs(rightMotor.getPosition().getValue().in(Units.Rotation) - position.getPosition()) < 0.5;
    }

    /**
     * Duty cycle holds down elevator command.
     * Because the PID turning is not the best, when intaking, we do a passive hold down
     * @return Command
     */
    public Command passiveHoldDown() {
        return new RunCommand(
                () -> this.setControl(this.dutyCycle.withOutput(-0.025)), this
        ).onlyIf(() -> (this.targetPosition == Position.HomeAndIntake
                && Math.abs(this.rightMotor.getPosition().getValue().in(Units.Rotation) - Position.HomeAndIntake.getPosition()) < 2
                && !(this.rightMotor.getPosition().getValue().in(Units.Rotation) < 0.01))).withName("passiveEscalatorHoldDown");
    }

    public Position getTarget() {
        return this.targetPosition;
    }

    /**
     * Manual escalator go up command
     * @return Command
     */
    public Command goUp() {
        return new RunCommand(
                () -> this.setControl(this.dutyCycle.withOutput(0.1)), this)
                .withName("EscalatorGoUp")
                .finallyDo(() -> this.setControl(this.dutyCycle.withOutput(0)));
    }

    /**
     * Manual escalator go down command
     * @return Command
     */
    public Command goDown() {
        return new RunCommand(() -> this.setControl(this.dutyCycle.withOutput(-0.1)), this).withName("EscalatorGoDown").finallyDo(() -> this.setControl(this.dutyCycle.withOutput(0)));
    }

    public StatusCode[] setControl(ControlRequest control) {
        return new StatusCode[]{this.rightMotor.setControl(control), this.leftMotor.setControl(leftFollower)};
    }

    public void resetPositions(double leftPosition, double rightPosition) {
        this.leftMotor.setPosition(leftPosition);
        this.rightMotor.setPosition(rightPosition);
    }

    /**
     * An enum that stores the positions of all the scoring locations. This way we can set the elevator to go to L1
     * with words as opposed to just pure numbers.
     */
    public enum Position {
        HomeAndIntake(0),
        L1(3),
        L2(5.118 - 0.26123),
        L3(11.5463 - 0.26123),
        L4(22.0844 - 0.26123 + 0.45),
        Hold(7.0), Other(-1);

        private final double position;

        // Constructor
        Position(double position) {
            this.position = position;
        }

        // Getter method
        public double getPosition() {
            return position;
        }
    }
}
