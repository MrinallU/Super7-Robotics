package org.firstinspires.ftc.teamcode.T3_2021;

import org.firstinspires.ftc.teamcode.Utils.Point;

public class T3_OdometryConstants {
    private static Point[] box1 = {new Point(60, 5, -90), new Point(68.5, 20, 180)};
    private static Point[] box2 = {new Point(80.5, 23, -90), new Point(92.5, 45, 180)};
    private static Point[] box3 = {new Point(106, 3.5, -90), new Point(117, 23, 180)};

    public static Point[][] boxes = {box1, box2, box3};

    private static Point powerShotR = new Point(64, 44.5, 90);
    private static Point powerShotM = new Point(64, 51, 90);
    private static Point powerShotL = new Point(64, 57.5, 90);
    public static Point[] powerShot = {powerShotR, powerShotM, powerShotL};

    private static Point powerShotR0 = new Point(0, 0, -5);
    private static Point powerShotM0 = new Point(0, 0, 0);
    private static Point powerShotL0 = new Point(0, 0, 5);
    public static Point[] powerShot0 = {powerShotR0, powerShotM0, powerShotL0};

    public static Point beforeShoot = new Point(75, 50, 90);

    public static Point[] beforeOtherWobble = {new Point(40, 14, 90), new Point(40, 17, 90), new Point(40, 16, 90)};
    public static Point[] otherWobble = {new Point(33, 14, 90), new Point(33, 16.5, 90), new Point(32, 16, 90)};

    public static Point beforeStack1 = new Point(64, 28, 90);
    public static Point afterStack1 = new Point(30, 28, 90);

    public static Point beforeStack3 = new Point(64, 25.5, 90);
    public static Point afterStack3 = new Point(45, 25.5, 90);
    public static Point shooting = new Point(62.5, 28, 90);

    public static Point shootP2 = new Point(62.5, 33, 90);

    public static Point parkLine = new Point(75, 35, 90);

    public static double startAngle = 0;

    // TeleOp Positions
    public static Point teleResetPos = new Point(65, 51, -90);
    public static Point teleResetPosOpp = new Point(65, 52, 90);
    public static Point goal = new Point(140, 32, 90);

    public static Point telePowerShot = new Point(65, 55, 90);

    // test points

    public static Point testHorizontal = new Point(16, 10, 0);
    public static Point testVertical = new Point(80, 49, 55);
    public static Point testTurn = new Point(0, 0, -90);
    public static Point test = new Point(34, 14, 0);
    public static Point [] splineTest = {new Point(49, 33, 0), new Point(72, 60, 0), new Point(100, 33, 0)};

}