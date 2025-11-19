package com.team841.dory.superstructure;

import java.util.function.BooleanSupplier;

import com.team841.dory.superstructure.AlgaePivot;
import com.team841.dory.superstructure.AlgaePivot.AlgaePivotPosition;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotMoveCommand extends Command {
    
    AlgaePivot algaePivot;
    AlgaePivot.AlgaePivotPosition position;
    
    BooleanSupplier hasAlgaeSupplier;
    boolean hasAlgae;

    BooleanSupplier hasCoralSupplier, isClearSupplier;
    boolean hasCoral, isClear;
    
    public PivotMoveCommand(AlgaePivot algaePivot, 
                            AlgaePivot.AlgaePivotPosition position, 
                            BooleanSupplier hasAlgaeSupplier, 
                            BooleanSupplier hasCoralSupplier, 
                            BooleanSupplier isClearSupplier) {
        this.algaePivot = algaePivot;
        this.position = position;
        this.hasAlgaeSupplier = hasAlgaeSupplier;
        this.hasCoralSupplier = hasCoralSupplier;
        this.isClearSupplier = isClearSupplier;
        
        addRequirements(this.algaePivot);
        setName("PivotMove");
    }

    @Override
    public void initialize() {
        this.hasAlgae = this.hasAlgaeSupplier.getAsBoolean();
        this.hasCoral = this.hasCoralSupplier.getAsBoolean();
        this.isClear = this.isClearSupplier.getAsBoolean();

        this.algaePivot.setPosition(position, hasAlgae);
    }

    @Override
    public void execute() {
        this.algaePivot.setPosition(position, hasAlgae);
    }

    @Override
    public boolean isFinished() {
        return this.algaePivot.atPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        return;
    }
}
