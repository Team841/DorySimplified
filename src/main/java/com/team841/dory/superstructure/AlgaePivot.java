package com.team841.dory.superstructure;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team841.dory.constants.SC;

import dev.doglog.DogLog;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaePivot extends SubsystemBase {
    
    TalonFX pivotMotor = new TalonFX(SC.AlgaePivot.pivotMotorId, "rio");

    MotionMagicTorqueCurrentFOC withOutAlgaeControl = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
    MotionMagicTorqueCurrentFOC withAlgaeControl = new MotionMagicTorqueCurrentFOC(0).withSlot(1);
    
    DutyCycleOut dutyCycle = new DutyCycleOut(0);

    StatusCode[] latestStatus;

    AlgaePivotPosition pivotTargetPosition = AlgaePivotPosition.Stow;

    public AlgaePivot() {
        this.pivotMotor.getConfigurator().apply(SC.AlgaePivot.pivotConfigs);
        this.resetPositions(0);
    }

    @Override
    public void periodic() {
        Angle pivotMotorPos = pivotMotor.getPosition().getValue();
        DogLog.log("AlgaePivot/AtSotow", this.atPosition(AlgaePivotPosition.Stow));
        DogLog.log("AlgaePivot/TargetPosition", this.pivotTargetPosition.toString());
        DogLog.log("AlgaePivot/PositionRadian", pivotMotorPos.in(Units.Rotation));
    }

    public void setPosition(AlgaePivotPosition position, boolean hasAlgae) {
        if (hasAlgae) {
            this.latestStatus = setControl(withAlgaeControl.withPosition(position.getPosition()));
        } else {
            this.latestStatus = setControl(withOutAlgaeControl.withPosition(position.getPosition()));
        }

        this.pivotTargetPosition = position;
    }

    public StatusCode[] setControl(ControlRequest control) {
        return new StatusCode[]{this.pivotMotor.setControl(control)};
    }

    public void resetPositions(double position) {
        this.pivotMotor.setPosition(position);
    }

    public AlgaePivotPosition getTarget() {
        return this.pivotTargetPosition;
    }

    public boolean atPosition(AlgaePivotPosition position) {
        return Math.abs(pivotMotor.getPosition().getValue().in(Units.Rotation) - position.getPosition()) < 0.5;
    }

    public enum AlgaePivotPosition {
        ReefPickup(0),
        GroundPickup(0),
        BargeScore(0),
        ProcessorScore(0),
        Stow(0);

        private final double rotationsPosition;

        AlgaePivotPosition(double rotationsPosition) {
            this.rotationsPosition = rotationsPosition;
        }

        public double getPosition() {
            return rotationsPosition;
        }
    }
}
