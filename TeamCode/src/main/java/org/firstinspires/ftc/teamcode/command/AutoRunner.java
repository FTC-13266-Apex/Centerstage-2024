package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystem.Drive;

public abstract class AutoRunner {
    Drive drive;

    public AutoRunner() {

    }

    public abstract void initialize();

    public abstract void run();
}
