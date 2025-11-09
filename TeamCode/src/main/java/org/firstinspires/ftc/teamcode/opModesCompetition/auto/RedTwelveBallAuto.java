package org.firstinspires.ftc.teamcode.opModesCompetition.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.generalOpModes.Alliance;

@Autonomous(name="AutoRedTwelveBall", group="Twelve")
public class RedTwelveBallAuto extends TwelveBallAuto {
    public RedTwelveBallAuto() {
        super(Alliance.RED);
    }
}
