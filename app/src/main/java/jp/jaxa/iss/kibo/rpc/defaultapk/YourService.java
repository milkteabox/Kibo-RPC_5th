package jp.jaxa.iss.kibo.rpc.defaultapk;

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

//    Mat navCameraMatrix = new Mat(3, 3 , CvType.CV_64F);
//    Mat navDistortionCoefficients = new Mat(1 , 5 , CvType.CV_64F);
//    Mat dockCameraMatrix = new Mat(3, 3 , CvType.CV_64F);
//    Mat dockDistortionCoefficients = new Mat(1 , 5 , CvType.CV_64F);

    @Override
    protected void runPlan1() {
        api.startMission();


        Mat img = Imgcodecs.imread("TEST.jpg");
        detectArucoFromMat(img);
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

//    private Mat getNavCamCalibrateMat() {
//        Mat originalImage = api.getMatNavCam();
//
//        Mat calibrateImaged = new Mat();
//
//        Calib3d.undistort(
//                originalImage,
//                calibrateImaged,
//                navCameraMatrix,
//                navDistortionCoefficients
//        );
//        return calibrateImaged;
//    }
//
//    private Mat getDockCamCalibrateMat(){
//        Mat originalImage = api.getMatDockCam();
//
//        Mat calibrateImaged = new Mat();
//
//        Calib3d.undistort(
//                originalImage,
//                calibrateImaged,
//                dockCameraMatrix,
//                dockDistortionCoefficients
//        );
//        return calibrateImaged;
//    }

    private void detectArucoFromMat(Mat mat){

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

//    private void initGetCalibMat(){
//        double[] navCamDoubleMatrix = api.getNavCamIntrinsics()[0];
//        double[] navDistortionCoefficientsDoubleMatrix = api.getNavCamIntrinsics()[1];
//
//        navCameraMatrix.put(0,0, navCamDoubleMatrix[0]);
//        navCameraMatrix.put(0,1, navCamDoubleMatrix[1]);
//        navCameraMatrix.put(0,2, navCamDoubleMatrix[2]);
//        navCameraMatrix.put(1,0, navCamDoubleMatrix[3]);
//        navCameraMatrix.put(1,1, navCamDoubleMatrix[4]);
//        navCameraMatrix.put(1,2, navCamDoubleMatrix[5]);
//        navCameraMatrix.put(2,0, navCamDoubleMatrix[6]);
//        navCameraMatrix.put(2,1, navCamDoubleMatrix[7]);
//        navCameraMatrix.put(2,2, navCamDoubleMatrix[8]);
//
//        navDistortionCoefficients.put(0,0, navDistortionCoefficientsDoubleMatrix[0]);
//        navDistortionCoefficients.put(0,1, navDistortionCoefficientsDoubleMatrix[1]);
//        navDistortionCoefficients.put(0,2, navDistortionCoefficientsDoubleMatrix[2]);
//        navDistortionCoefficients.put(0,3, navDistortionCoefficientsDoubleMatrix[3]);
//        navDistortionCoefficients.put(0,4, navDistortionCoefficientsDoubleMatrix[4]);
//
//        double[] dockCamDoubleMatrix = api.getDockCamIntrinsics()[0];
//        double[] dockDistortionCoefficientsDoubleMatrix = api.getDockCamIntrinsics()[1];
//
//        dockCameraMatrix.put(0,0, dockCamDoubleMatrix[0]);
//        dockCameraMatrix.put(0,1, dockCamDoubleMatrix[1]);
//        dockCameraMatrix.put(0,2, dockCamDoubleMatrix[2]);
//        dockCameraMatrix.put(1,0, dockCamDoubleMatrix[3]);
//        dockCameraMatrix.put(1,1, dockCamDoubleMatrix[4]);
//        dockCameraMatrix.put(1,2, dockCamDoubleMatrix[5]);
//        dockCameraMatrix.put(2,0, dockCamDoubleMatrix[6]);
//        dockCameraMatrix.put(2,1, dockCamDoubleMatrix[7]);
//        dockCameraMatrix.put(2,2, dockCamDoubleMatrix[8]);
//
//        dockDistortionCoefficients.put(0,0, dockDistortionCoefficientsDoubleMatrix[0]);
//        dockDistortionCoefficients.put(0,1, dockDistortionCoefficientsDoubleMatrix[1]);
//        dockDistortionCoefficients.put(0,2, dockDistortionCoefficientsDoubleMatrix[2]);
//        dockDistortionCoefficients.put(0,3, dockDistortionCoefficientsDoubleMatrix[3]);
//        dockDistortionCoefficients.put(0,4, dockDistortionCoefficientsDoubleMatrix[4]);
//    }
}
