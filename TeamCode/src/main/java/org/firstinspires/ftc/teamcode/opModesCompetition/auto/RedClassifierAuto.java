package org.firstinspires.ftc.teamcode.opModesCompetition.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.generalOpModes.Alliance;

@Autonomous(name="AutoRedClassifier")
public class RedClassifierAuto extends ClassifierAuto {
    public RedClassifierAuto() {
        super(Alliance.RED);
    }
}
