package org.firstinspires.ftc.teamcode.opModesCompetition.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.generalOpModes.Alliance;

@Autonomous(name="AutoBlueTwelveBall", group="Twelve")
public class BlueTwelveBallAuto extends TwelveBallAuto {
    public BlueTwelveBallAuto() {
        super(Alliance.BLUE);
    }
}
