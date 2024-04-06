package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.CvType;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    private double[][]  navCamIntrinsicsMatrix;
    private double[][] dockCamIntrinsicsMatrix;

    @Override
    protected void runPlan1() {
        api.startMission();
        initCalibMatrix();

        api.saveMatImage(api.getMatDockCam(),"nD");
        api.saveMatImage(getDockCamCalibrateMat(),"D");

        api.saveMatImage(api.getMatNavCam(),"nN");
        api.saveMatImage(getNavCamCalibrateMat(),"N");//testImg


        Mat img = Imgcodecs.imread("TEST.png");//wait for fix, imgload failed

        detectArucoFromMat(img);//wait for test

        api.reportRoundingCompletion();
    }

    @Override
    protected void runPlan2() {
        // write your plan 2 here
    }

    @Override
    protected void runPlan3() {
        // write your plan 3 here
    }

    private boolean moveToWithRetry(Point point, Quaternion quaternion, int loopMAX_time) {
        Result result;
        final int LOOP_MAX = loopMAX_time;

        result = api.moveTo(point, quaternion, false);

        int loopCounter = 0;
        while (!result.hasSucceeded() && loopCounter < LOOP_MAX) {
            result = api.moveTo(point, quaternion, false);
            ++loopCounter;
        }

        return result.hasSucceeded();
    }

    private Mat getNavCamCalibrateMat() {

        Mat navCameraMatrix = new Mat(3, 3 , CvType.CV_64F);
        Mat navDistortionCoefficients = new Mat(1 , 5 , CvType.CV_64F);
        setCamCalib(navCamIntrinsicsMatrix[0], navCamIntrinsicsMatrix[1], navCameraMatrix, navDistortionCoefficients);

        Mat originalImage = api.getMatNavCam();

        Mat calibrateImaged = new Mat();

        Calib3d.undistort(
                originalImage,
                calibrateImaged,
                navCameraMatrix,
                navDistortionCoefficients
        );
        return calibrateImaged;
    }

    private Mat getDockCamCalibrateMat(){

        Mat dockCameraMatrix = new Mat(3, 3 , CvType.CV_64F);
        Mat dockDistortionCoefficients = new Mat(1 , 5 , CvType.CV_64F);
        setCamCalib(dockCamIntrinsicsMatrix[0], dockCamIntrinsicsMatrix[1], dockCameraMatrix, dockDistortionCoefficients);

        Mat originalImage = api.getMatDockCam();

        Mat calibrateImaged = new Mat();

        Calib3d.undistort(
                originalImage,
                calibrateImaged,
                dockCameraMatrix,
                dockDistortionCoefficients
        );
        return calibrateImaged;
    }

    private void detectArucoFromMat(Mat mat){

        if(mat.empty()){
            Log.i("imgLoad", "imgLoad_FAILED");
            api.reportRoundingCompletion();
        }

        List<Mat> arucoCorners = new ArrayList<>();
        Mat arucoIDs = new Mat();

        Mat arucoDrawMat = mat;

        Aruco.detectMarkers(mat,  Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250), arucoCorners, arucoIDs);

        Aruco.drawDetectedMarkers(arucoDrawMat, arucoCorners, arucoIDs, new Scalar(0, 255, 0));

        if (!arucoCorners.isEmpty()){
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(arucoCorners, 0.05f, new MatOfDouble(), new MatOfDouble(), rvecs, tvecs);

            MatOfPoint3f itemBoardWorldPoint = new MatOfPoint3f(
                    new Point3(3.75, 3.75, 0),
                    new Point3(3.75, -11.25, 0),
                    new Point3(-24.25, -11.25, 0),
                    new Point3(-24.25, 3.75, 0));

            MatOfPoint2f itemBoardImagePoints = new MatOfPoint2f();

            Calib3d.projectPoints(itemBoardWorldPoint, rvecs, tvecs, new MatOfDouble(), new MatOfDouble(), itemBoardImagePoints);

            org.opencv.core.Point topRight = new org.opencv.core.Point(itemBoardImagePoints.get(0, 0));
            org.opencv.core.Point bottomLeft = new org.opencv.core.Point(itemBoardImagePoints.get(1, 0));
            org.opencv.core.Point bottomRight = new org.opencv.core.Point(itemBoardImagePoints.get(2, 0));
            org.opencv.core.Point topLeft = new org.opencv.core.Point(itemBoardImagePoints.get(3, 0));

            Mat test = mat;
            Imgproc.line(test, topRight, bottomRight, new Scalar(0, 255, 0), 2);
            Imgproc.line(test, bottomRight, bottomLeft, new Scalar(0, 255, 0), 2);
            Imgproc.line(test, bottomLeft, topLeft, new Scalar(0, 255, 0), 2);
            Imgproc.line(test, topLeft, topRight, new Scalar(0, 255, 0), 2);

            api.saveMatImage(test,"test");
        }
    }

    private void setCamCalib(double[] cameraDoubleMatrix, double[] distortionCoefficientsDoubleMatrix, Mat cameraMatrix, Mat distortionCoefficients) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cameraMatrix.put(i, j, cameraDoubleMatrix[i * 3 + j]);
            }
        }

        for (int i = 0; i < 1; i++) {
            for (int j = 0; j < 5; j++) {
                distortionCoefficients.put(i, j, distortionCoefficientsDoubleMatrix[j]);
            }
        }
    }

    private void initCalibMatrix(){
        navCamIntrinsicsMatrix = api.getNavCamIntrinsics();
        dockCamIntrinsicsMatrix = api.getDockCamIntrinsics();
    }
}
