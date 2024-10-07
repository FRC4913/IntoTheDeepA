package com.huskyteers.paths;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

public class Paths {
    public static TrajectoryActionBuilder examplePath(TrajectoryActionBuilder actionBuilder) {
        return actionBuilder.strafeTo(new Vector2d(10, 10));
    }

    public static TrajectoryActionBuilder driveToBasket(TrajectoryActionBuilder actionBuilder) {
        return actionBuilder.strafeTo(new Vector2d(0, 0));
    }
}
