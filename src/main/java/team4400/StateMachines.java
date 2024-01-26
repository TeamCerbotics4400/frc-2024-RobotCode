// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team4400;

/** Add your docs here. */
public class StateMachines {
    
    public enum IntakeState{
        INTAKING,
        IDLE,
        FULL,
        SHOOTING
    }

    public enum PositionState{
        CABLE,
        MIDDLE,
        LOADING,
        TELE
    }

    //POSITION
    public static PositionState currentPositionState = PositionState.TELE;

    public static void setPositionState(PositionState state){
        currentPositionState = state;
    }

    public static PositionState getPositionState(){
        return currentPositionState;
    }

    //INTAKE
    public static IntakeState currentIntakeState = IntakeState.IDLE;

    public static void setIntaking(){
        if(currentIntakeState != IntakeState.INTAKING) currentIntakeState = IntakeState.INTAKING;
    }

    public static void setShooting(){
        currentIntakeState = IntakeState.SHOOTING;
    }

    public static boolean isShooting(){
        return currentIntakeState == IntakeState.SHOOTING;
    }

    public static void setIntakeIdle(){
        if(currentIntakeState != IntakeState.IDLE){
            currentIntakeState = IntakeState.IDLE;
        }
    }

    public static void setIntakeFull(){
        if(currentIntakeState != IntakeState.FULL){
            currentIntakeState = IntakeState.FULL;
        }
    }

    public static void setIntakeState(IntakeState state){
        currentIntakeState = state;
    }

    public static IntakeState getIntakeState(){
        return currentIntakeState;
    }
}
