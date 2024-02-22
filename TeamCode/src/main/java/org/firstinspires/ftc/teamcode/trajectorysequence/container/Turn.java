package org.firstinspires.ftc.teamcode.trajectorysequence.container;

public class Turn extends PathSegment {
    public volatile double angle;
    public Turn(double angle) {
        this.angle = angle;
    }
}