package com.team841.dory.superstructure;

import java.util.function.BooleanSupplier;

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
        
    }
}
