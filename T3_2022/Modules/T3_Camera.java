package org.firstinspires.ftc.teamcode.T3_2022.Modules;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;

public class T3_Camera {
    private static final String VUFORIA_KEY = "AWnPBRj/////AAABmaYDUsaxX0BJg7/6QOpapAl4Xf18gqNd7L9nALxMG8K2AF6lodTZQ78nnksFc2CMy/3KmeolDEFGmp0CQJ7c/5PKymmJYckCfsg16B6Vnw5OihuD2mE7Ky0tT1VGdit2KvolunYkjWKDiJpX15SFMX//Jclt+Xt8riZqh3edXpUdREIXxS9tmdF/O6Nc5mUI7FEfAJHq4xUaqSY/yta/38qirjy3tdqFjDGc9g4DmgPE6+6dGLiXeUJYu32AgoefA1iFRF+ZVNJEc1j4oyw3JYQgWwfziqyAyPU2t9k9UDgqEkyxGxl4xS70KN/SBEUZeq4CzYfyon2kSSvKK/6/Vt4maMzG3LXfLt0PMiEPI1z+";
    private VuforiaLocalizer vuforia;
    private HardwareMap hardwareMap;

    private int blueThreshold = 70;
    private int redThreshold = 70;

    public String outStr = "";


    // Rings Are YELLOW (255 R and 255 G)
    // Just look at blue value

    public T3_Camera(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        initVuforia();
    }

    public int readBarcode(String auto) throws InterruptedException{
        int position = 0;
        Bitmap bm;
        double[] posOne = new double[4];
        double[] posTwo = new double[4];
        double[] posThree = new double[4];

        VuforiaLocalizer.CloseableFrame closeableFrame = vuforia.getFrameQueue().take();
        bm = vuforia.convertFrameToBitmap(closeableFrame);
        if(auto == "redPrimary"){
            posOne = calculateAverageRGB(bm, 976, 31, 1087, 80);
            posTwo = calculateAverageRGB(bm, 439, 81, 558, 151);
            posThree = calculateAverageRGB(bm, 0, 0, 0, 0);
        }else if(auto == "redSecondary"){
            posOne = calculateAverageRGB(bm, 410, 61, 526, 137);
            posTwo = calculateAverageRGB(bm, 62, 92, 112, 170);
            posThree = calculateAverageRGB(bm, 0, 0, 0, 0);
        }else if(auto == "bluePrimary"){
            posOne = calculateAverageRGB(bm, 354, 267, 492, 298);
            posTwo = calculateAverageRGB(bm, 9 , 296, 43, 319);
            posThree = calculateAverageRGB(bm, 0, 0, 0, 0);
        }else if(auto == "blueSecondary"){
            posOne = calculateAverageRGB(bm, 840, 186, 940, 205);
            posTwo = calculateAverageRGB(bm, 457, 194, 552, 220);
            posThree = calculateAverageRGB(bm, 0, 0, 0, 0);
        }



        if(auto.contains("red")){
            if(posOne[3] < blueThreshold){
                return 0;
            }else if(posTwo[3] < blueThreshold){
                return 1;
            }else{
                return 2;
            }
        }else{
            if(posOne[1] < redThreshold){
                return 0;
            }else if(posTwo[1] < redThreshold){
                return 1;
            }else{
                return 2;
            }
        }



    }

    public int readBarcodeRed(String auto) throws InterruptedException{
        int position = 0;
        Bitmap bm;
        double[] posOne = new double[4];
        double[] posTwo = new double[4];
        double[] posThree = new double[4];

        VuforiaLocalizer.CloseableFrame closeableFrame = vuforia.getFrameQueue().take();
        bm = vuforia.convertFrameToBitmap(closeableFrame);
        if(auto == "redPrimary"){
            posOne = calculateAverageRGB(bm, 970, 233, 1052, 269);
            posTwo = calculateAverageRGB(bm, 489, 261, 662, 287);
            posThree = calculateAverageRGB(bm, 0, 0, 0, 0);
        }else if(auto == "redSecondary"){
            posOne = calculateAverageRGB(bm, 426, 278, 551, 307);
            posTwo = calculateAverageRGB(bm, 52, 284, 85, 313 );
            posThree = calculateAverageRGB(bm, 0, 0, 0, 0);
        }else if(auto == "bluePrimary"){
            posOne = calculateAverageRGB(bm, 400, 150, 540, 220);
            posTwo = calculateAverageRGB(bm, 22, 140, 60, 215);
            posThree = calculateAverageRGB(bm, 0, 0, 0, 0);
        }else if(auto == "blueSecondary"){
            posOne = calculateAverageRGB(bm, 840, 186, 940, 205);
            posTwo = calculateAverageRGB(bm, 457, 194, 552, 220);
            posThree = calculateAverageRGB(bm, 0, 0, 0, 0);
        }


        if(posOne[2] < 20){
            return 0;
        }else if(posTwo[2] < 20){
            return 1;
        }else{
            return 2;
        }
    }


    public void saveImage(){
        try{
            File outF = AppUtil.getInstance().getSettingsFile("Picture.png");
            FileOutputStream out = new FileOutputStream(outF);

            Bitmap bm;

            // Link to image visualizer: https://yangcha.github.io/iview/iview.html

            VuforiaLocalizer.CloseableFrame closeableFrame = vuforia.getFrameQueue().take();

            bm = vuforia.convertFrameToBitmap(closeableFrame);
            drawRectangle(bm, 345, 236, 365, 246);
            drawRectangle(bm, 345, 280, 365, 300);

            bm.compress(Bitmap.CompressFormat.PNG, 100, out);
            out.close();
        }
        catch(Exception e){
            e.printStackTrace();
        }
    }

    public void saveImage(int id){
        try{
            File outF = AppUtil.getInstance().getSettingsFile("Picture" +id +".png");
            FileOutputStream out = new FileOutputStream(outF);

            Bitmap bm;

            // Link to image visualizer: https://yangcha.github.io/iview/iview.html

            VuforiaLocalizer.CloseableFrame closeableFrame = vuforia.getFrameQueue().take();

            bm = vuforia.convertFrameToBitmap(closeableFrame);


            bm.compress(Bitmap.CompressFormat.PNG, 100, out);
            out.close();
        }
        catch(Exception e){
            e.printStackTrace();
        }
    }

    public void drawRectangle(Bitmap bm, int left, int top, int right, int bottom){
        for(int x = left; x <= right; x++){
            for(int y = top; y <= bottom; y++){
                if(x == left || x == right || y == top || y == bottom){
                    bm.setPixel(x, y, Color.BLACK);
                }
            }
        }
    }

    public double[] calculateAverageRGB(Bitmap bm, int left, int top, int right, int bottom){
        long numTotalPixels = 0;
        double[] colorValues = {0, 0, 0, 0};
        for(int x = left; x <= right; x++){
            for(int y = top; y <= bottom; y++){
                int pixel = bm.getPixel(x, y);
                colorValues[0] += Color.alpha(pixel);
                colorValues[1] += Color.red(pixel);
                colorValues[2] += Color.green(pixel);
                colorValues[3] += Color.blue(pixel);
                numTotalPixels++;
            }
        }

        for(int i = 0; i < colorValues.length; i++){
            colorValues[i] = colorValues[i] / numTotalPixels;
        }

        return colorValues;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        vuforia.setFrameQueueCapacity(1);

        vuforia.enableConvertFrameToBitmap();

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    public String argbToText(double[] arr){
        return String.format("A: %.2f, R: %.2f, G: %.2f, B: %.2f", arr[0], arr[1], arr[2], arr[3]);
    }
}
