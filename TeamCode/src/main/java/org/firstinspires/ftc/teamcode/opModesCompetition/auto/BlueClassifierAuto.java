package org.firstinspires.ftc.teamcode.opModesCompetition.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.generalOpModes.Alliance;

@Autonomous(name="AutoBlueClassifier", group="Classifier")
public class BlueClassifierAuto extends ClassifierAuto {
    public BlueClassifierAuto() {
        super(Alliance.BLUE);
    }
}
