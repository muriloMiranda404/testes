package frc.robot.autonomous;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;

public class PathPlanner{

    public void configurarEventosIntake(){
        HashMap<String, Command> hashMap = new HashMap<>();

        hashMap.put("Waypoint 1", new InstantCommand(() ->{
            Robot.setpoint_intake = 72.0;
        })
        .andThen(new WaitUntilCommand(() ->
        Robot.setpoint_intake == 72
        ))
        .andThen(new InstantCommand(() ->{
            Robot.setpoint_elevador = 1480.0;
        }))
        .andThen(new WaitUntilCommand(() ->
        Robot.setpoint_elevador == 1480.0
        ))
        .andThen(new InstantCommand(() ->{
            Robot.motor_da_bola.set(0.5);
            Robot.timer.reset();
            Robot.timer.start();
        }))
        .andThen(new WaitUntilCommand(() ->
        Robot.timer.get() >= 2.0
        ))
        .andThen(new InstantCommand(() ->{
            Robot.motor_da_bola.set(0);
            Robot.setpoint_elevador = 0.0;
        })).andThen(new WaitUntilCommand(() ->
        Robot.setpoint_elevador == 0.0
        ))
        .andThen(new InstantCommand(() ->{
            Robot.setpoint_intake = 61.0;
        })));
    }
}