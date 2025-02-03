package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;

public final class SplineTest extends OpMode {
    public Command command;

    @Override
    public void init() {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            command = drive.commandBuilder(beginPose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .build();
        } else {
            throw new RuntimeException("Tank Drive not yet supported!");
        }
    }

    @Override
    public void start() {
        command.schedule();
    }

    @Override
    public void loop() {
        if (!Mercurial.isScheduled(command)) {
            requestOpModeStop();
        }
    }
}
