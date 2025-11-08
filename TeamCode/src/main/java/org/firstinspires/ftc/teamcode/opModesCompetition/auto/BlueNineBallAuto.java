package org.firstinspires.ftc.teamcode.opModesCompetition.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.generalOpModes.Alliance;

@Autonomous(name="AutoBlueNineBall")
public class BlueNineBallAuto extends NineBallAuto {
    public BlueNineBallAuto() {
        super(Alliance.BLUE);
    }
}
