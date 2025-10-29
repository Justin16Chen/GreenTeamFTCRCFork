package org.firstinspires.ftc.teamcode.opModesCompetition.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.BallColorSensor;
import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.utils.generalOpModes.ParentOpMode;

@TeleOp(name="Sensor Test", group="Competition")
public class SensorTele extends ParentOpMode {
    public static String[] colorSensorNames = {Hardware.backColorSensorName, Hardware.middleColorSensorName, Hardware.frontColorSensorName };
    BallColorSensor[] colSensor;
    @Override
    public void initiation() {
        for (int i=0; i<3; i++) {
            colSensor[i] = new BallColorSensor(hardware, telemetry, colorSensorNames[i], true);
            colSensor[i].declareHardware();
        }
    }

    @Override
    public void updateLoop() {
        for (int i=0; i<3; i++) {
            colSensor[i].update();
            colSensor[i].printInfo();
        }
        telemetry.update();
    }
}
