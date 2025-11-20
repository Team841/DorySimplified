package com.team841.dory.superstructure;

import java.util.function.BooleanSupplier;

import com.team841.dory.superstructure.AlgaePivot;
import com.team841.dory.superstructure.AlgaePivot.AlgaePivotPosition;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotMoveCommand extends Command {
    
    AlgaePivot algaePivot;
    AlgaePivot.AlgaePivotPosition position;
    
    public PivotMoveCommand(AlgaePivot algaePivot, 
                            AlgaePivot.AlgaePivotPosition position) {
        this.algaePivot = algaePivot;
        this.position = position;
        
        addRequirements(this.algaePivot);
        setName("PivotMove");
    }

    @Override
    public void initialize() {
        this.algaePivot.setPosition(position);
    }

    @Override
    public void execute() {
        this.algaePivot.setPosition(position);
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
