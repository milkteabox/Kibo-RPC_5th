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
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.defaultapk.constants.Area;

import static jp.jaxa.iss.kibo.rpc.defaultapk.constants.Constants.*;

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

        moveToWithRetry(new Point(10.95, -9.0,5.195), astronautPointwithQuaternion.quaternion, 5);


        detectArucoFromMat(getDockCamCalibrateMat(), dockCamIntrinsicsMatrix[0]);

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

    private void detectArucoFromMat(Mat img, double[] CamDoubleMatrix){

        Mat cameraMatrix = new Mat(3, 3 , CvType.CV_64F);

        cameraMatrix.put(0,0, CamDoubleMatrix[0]);
        cameraMatrix.put(0,1, 0);
        cameraMatrix.put(0,2, CamDoubleMatrix[2]);
        cameraMatrix.put(1,0, 0);
        cameraMatrix.put(1,1, CamDoubleMatrix[4]);
        cameraMatrix.put(1,2, CamDoubleMatrix[5]);
        cameraMatrix.put(2,0, 0);
        cameraMatrix.put(2,1, 0);
        cameraMatrix.put(2,2, 1);


        Mat distCoeffs = new Mat(1 , 5 , CvType.CV_64F);
        distCoeffs.setTo(new Scalar(0.0));
        MatOfDouble doubleDistCoeffs = new MatOfDouble(0.0, 0.0, 0.0, 0.0, 0.0);

        List<Mat> arucoCorners = new ArrayList<>();
        Mat arucoIDs = new Mat();

        Aruco.detectMarkers(img, Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250), arucoCorners, arucoIDs);

        Mat arucoDrawMat = img;
        Aruco.drawDetectedMarkers(arucoDrawMat, arucoCorners, arucoIDs, new Scalar(0, 255, 0));
        api.saveMatImage(arucoDrawMat,"123.mat");

        if(!arucoIDs.empty()) {
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(arucoCorners, 5.0f, cameraMatrix, distCoeffs, rvecs, tvecs);

            MatOfPoint3f itemBoardWorldPoint = new MatOfPoint3f(
                    new Point3(-24.25, 3.75, 0),
                    new Point3(-24.25, -11.25, 0),
                    new Point3(3.75, -11.25, 0),
                    new Point3(3.75, 3.75, 0));

            MatOfPoint2f itemBoardImagePoints = new MatOfPoint2f();

            Calib3d.projectPoints(itemBoardWorldPoint, rvecs, tvecs, cameraMatrix, doubleDistCoeffs, itemBoardImagePoints);

            org.opencv.core.Point topLeft = new org.opencv.core.Point(itemBoardImagePoints.get(0, 0));
            org.opencv.core.Point bottomLeft = new org.opencv.core.Point(itemBoardImagePoints.get(1, 0));
            org.opencv.core.Point bottomRight = new org.opencv.core.Point(itemBoardImagePoints.get(2, 0));
            org.opencv.core.Point topRight = new org.opencv.core.Point(itemBoardImagePoints.get(3, 0));

            Mat test = img;
            Imgproc.line(test, topRight, bottomRight, new Scalar(0, 255, 0), 2);
            Imgproc.line(test, bottomRight, bottomLeft, new Scalar(0, 255, 0), 2);
            Imgproc.line(test, bottomLeft, topLeft, new Scalar(0, 255, 0), 2);
            Imgproc.line(test, topLeft, topRight, new Scalar(0, 255, 0), 2);

            api.saveMatImage(test,"test.mat");

            int cmpp = 30;
            Mat frontalView = new Mat(15 * cmpp, 27 * cmpp, CvType.CV_8UC3);

            MatOfPoint2f dstPoints = new MatOfPoint2f(
                    new org.opencv.core.Point(0, 0),
                    new org.opencv.core.Point(0, frontalView.rows() - 1),
                    new org.opencv.core.Point(frontalView.cols() - 1, frontalView.rows() - 1),
                    new org.opencv.core.Point(frontalView.cols() - 1, 0));


            Mat transformationMatrix = Imgproc.getPerspectiveTransform(itemBoardImagePoints, dstPoints);
            Imgproc.warpPerspective(img, frontalView, transformationMatrix, frontalView.size());

            api.saveMatImage(frontalView, "frontalView.mat");
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

    public static Mat resizeImage(Mat inputImage, double scaleFactor) {
        Mat resizedImage = new Mat();
        Imgproc.resize(inputImage, resizedImage, new Size(), scaleFactor, scaleFactor, Imgproc.INTER_LINEAR);
        return resizedImage;
    }
}
