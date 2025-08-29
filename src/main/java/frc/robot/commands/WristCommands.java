package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.WristSubsystem;

public class WristCommands {
    public static Command Stowed(WristSubsystem wrist) {
        return wrist.setAngle(WristSubsystem.WristPosition.Stowed.angle());
    }

    public static Command AlgaeIntake(WristSubsystem wrist) {
        return wrist.setAngle(WristSubsystem.WristPosition.AlgaeGroundIntake.angle());
    }

    public static Command TestWrist(WristSubsystem wrist) {
        return wrist.setAngle(WristSubsystem.WristPosition.Test.angle());
    }
}
