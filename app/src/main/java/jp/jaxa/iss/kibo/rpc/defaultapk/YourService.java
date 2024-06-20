package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;
import android.util.Pair;

import org.opencv.aruco.Aruco;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.defaultapk.untils.AreasItemData;
import jp.jaxa.iss.kibo.rpc.defaultapk.untils.PointWithQuaternion;

import static jp.jaxa.iss.kibo.rpc.defaultapk.Constants.*;
import static org.opencv.calib3d.Calib3d.Rodrigues;

public class YourService extends KiboRpcService {

    private double[][]  navCamIntrinsicsMatrix;
    private double[][] dockCamIntrinsicsMatrix;
    private AreasItemData areasData = new AreasItemData();
    private String[] finalAreaData = new String[5];

    TFliteDetector tfliteDetector;

    @Override
    protected void runPlan1(){
        api.startMission();
        initCalibMatrix();
        tfliteDetector = new TFliteDetector(this);
        Thread threadVision = new Thread(new Vision());
        threadVision.start();
        scanMove();
        reportAreaInfoAndEndRounding();
        sleep(3500);
        threadVision.interrupt();
        api.notifyRecognitionItem();
        targetTask();
    }

    @Override
    protected void runPlan2(){}

    @Override
    protected void runPlan3(){}

    class Vision implements Runnable {

        @Override
        public void run() {
            Mat navImgPast = api.getMatNavCam();
            Mat dockImgPast = api.getMatDockCam();
            while (!Thread.currentThread().isInterrupted()) {
                Mat navImgNow = api.getMatNavCam();
                Mat dockImgNow = api.getMatDockCam();

                Kinematics robotNowKinematics = api.getRobotKinematics();
                if(areImgEqual(navImgPast, navImgNow)){
                    PointWithQuaternion navImgShotPQ = new PointWithQuaternion(robotNowKinematics.getPosition(), robotNowKinematics.getOrientation());
                    navImgPast = navImgNow;

                    Mat calibNavImg = calibImgWithMatrix(navImgNow, navCamIntrinsicsMatrix);
                    scanItemFromMat(calibNavImg, navCamIntrinsicsMatrix[0]);
                }//Task when new NavImg

                if(areImgEqual(dockImgPast, dockImgNow)){
                    PointWithQuaternion dockImgShotPQ = new PointWithQuaternion(robotNowKinematics.getPosition(), robotNowKinematics.getOrientation());//wait to fix
                    dockImgPast = dockImgNow;

                    Mat calibDockImg = calibImgWithMatrix(dockImgNow, dockCamIntrinsicsMatrix);
                    scanItemFromMat(calibDockImg, dockCamIntrinsicsMatrix[0]);
                }//Task when new dockImg

                try {
                    Thread.sleep(300);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    private Point calcArucoPos(Mat img, Integer targetAreaNum){
        double[] camDoubleMatrix = navCamIntrinsicsMatrix[0];

        Mat cameraMatrix = new Mat(3, 3 , CvType.CV_64F);//setup cameraMatrix for calibratedImg
        cameraMatrix.put(0,0, camDoubleMatrix[0]);
        cameraMatrix.put(0,1, 0);
        cameraMatrix.put(0,2, camDoubleMatrix[2]);
        cameraMatrix.put(1,0, 0);
        cameraMatrix.put(1,1, camDoubleMatrix[4]);
        cameraMatrix.put(1,2, camDoubleMatrix[5]);
        cameraMatrix.put(2,0, 0);
        cameraMatrix.put(2,1, 0);
        cameraMatrix.put(2,2, 1);


        Mat distCoeffs = new Mat(1 , 5 , CvType.CV_64F);//setup distCoeffs for calibratedImg
        distCoeffs.setTo(new Scalar(0.0));

        List<Mat> arucoCorners = new ArrayList<>();
        Mat arucoIDs = new Mat();

        Aruco.detectMarkers(img, Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250), arucoCorners, arucoIDs);

        if(!arucoIDs.empty()) {
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(arucoCorners, 0.05f, cameraMatrix, distCoeffs, rvecs, tvecs);

            for (int i = 0; i < arucoIDs.rows(); i++) {
                int id = (int) arucoIDs.get(i, 0)[0]-100;
                if(id != targetAreaNum){continue;}
                Mat tvec = tvecs.row(i);

                double[] tvecArray = tvec.get(0, 0);
                double tx = tvecArray[0];
                double ty = tvecArray[1];
                double tz = tvecArray[2];

                return new Point(tx, ty, tz);
            }
        }
        return null;
    }

    private void scanItemFromMat(Mat img, double[] camDoubleMatrix){
        Mat cameraMatrix = new Mat(3, 3 , CvType.CV_64F);//setup cameraMatrix for calibratedImg
        cameraMatrix.put(0,0, camDoubleMatrix[0]);
        cameraMatrix.put(0,1, 0);
        cameraMatrix.put(0,2, camDoubleMatrix[2]);
        cameraMatrix.put(1,0, 0);
        cameraMatrix.put(1,1, camDoubleMatrix[4]);
        cameraMatrix.put(1,2, camDoubleMatrix[5]);
        cameraMatrix.put(2,0, 0);
        cameraMatrix.put(2,1, 0);
        cameraMatrix.put(2,2, 1);


        Mat distCoeffs = new Mat(1 , 5 , CvType.CV_64F);//setup distCoeffs for calibratedImg
        distCoeffs.setTo(new Scalar(0.0));
        MatOfDouble doubleDistCoeffs = new MatOfDouble(0.0, 0.0, 0.0, 0.0, 0.0);

        List<Mat> arucoCorners = new ArrayList<>();
        Mat arucoIDs = new Mat();

        Aruco.detectMarkers(img, Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250), arucoCorners, arucoIDs);

        if(!arucoIDs.empty()) {
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(arucoCorners, 0.05f, cameraMatrix, distCoeffs, rvecs, tvecs);

            for (int i = 0; i < arucoIDs.rows(); i++) {
                int id = (int) arucoIDs.get(i, 0)[0]-100;
                if (id < 0 || id > 4) { continue; }
                Mat rvec = rvecs.row(i);
                Mat tvec = tvecs.row(i);

                Mat lostItemBoardImg = getWarpItemImg(img, rvec, tvec, cameraMatrix, doubleDistCoeffs);

                if(lostItemBoardImg != null){
                    Pair<String, Integer> itemData = tfliteDetector.DetectFromMat(lostItemBoardImg);
                    if(itemData != null){
                        areasData.putVisionData(id, itemData);
                        Log.i("TFLite",id+"  :" + itemData.toString());
                    }
                }
            }
        }
    }

    public Mat getWarpItemImg(Mat originImg, Mat rvec, Mat tvec, Mat cameraMatrix, MatOfDouble doubleDistCoeffs){

        MatOfPoint3f itemBoardWorldPoint = new MatOfPoint3f(
                new Point3(-0.2325, 0.0375, 0),
                new Point3(-0.2325, -0.1125, 0),
                new Point3(-0.0325, -0.1125, 0),
                new Point3(-0.0325, 0.0375, 0));

        MatOfPoint2f itemBoardImagePoints = new MatOfPoint2f();

        Calib3d.projectPoints(itemBoardWorldPoint, rvec, tvec, cameraMatrix, doubleDistCoeffs, itemBoardImagePoints);

        org.opencv.core.Point[] points = itemBoardImagePoints.toArray();
            for (org.opencv.core.Point point : points) {
                if (point.x < 0 || point.x >= originImg.cols() || point.y < 0 || point.y >= originImg.rows()) {
                return null;
                }
            }

        int cmpp = 30;
        Mat frontView = new Mat(15 * cmpp, 20 * cmpp, CvType.CV_8UC3);

        MatOfPoint2f dstPoints = new MatOfPoint2f(
                new org.opencv.core.Point(0, 0),
                new org.opencv.core.Point(0, frontView.rows() - 1),
                new org.opencv.core.Point(frontView.cols() - 1, frontView.rows() - 1),
                new org.opencv.core.Point(frontView.cols() - 1, 0));


        Mat transformationMatrix = Imgproc.getPerspectiveTransform(itemBoardImagePoints, dstPoints);
        Imgproc.warpPerspective(originImg, frontView, transformationMatrix, frontView.size());
        return frontView;
    }

    public static boolean areImgEqual(Mat image1, Mat image2) {
        if (image1.rows() == image2.rows() && image1.cols() == image2.cols() && image1.channels() == image2.channels()) {
            Mat diffImage = new Mat();
            Core.compare(image1, image2, diffImage, Core.CMP_NE);
            return Core.countNonZero(diffImage) != 0;
        } else {
            return true;
        }
    }

    //Img Calibrate Tasks
    private void initCalibMatrix(){
        navCamIntrinsicsMatrix = api.getNavCamIntrinsics();
        dockCamIntrinsicsMatrix = api.getDockCamIntrinsics();
    }

    private Mat calibImgWithMatrix(Mat originalImg, double[][] calibMatrix) {
        Mat CameraMatrix = new Mat(3, 3 , CvType.CV_64F);
        Mat DistortionCoefficients = new Mat(1 , 5 , CvType.CV_64F);
        setCamCalib(calibMatrix[0], calibMatrix[1], CameraMatrix, DistortionCoefficients);

        Mat calibrateImaged = new Mat();

        Calib3d.undistort(
                originalImg,
                calibrateImaged,
                CameraMatrix,
                DistortionCoefficients
        );
        return calibrateImaged;
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

    //Move Tasks
    private boolean moveToWithRetry(PointWithQuaternion pq, int loopMAX_time) {
        Point point = pq.point;
        Quaternion quaternion = pq.quaternion;
        Result result;
        final double MAX_THRESHOLD_Angle = 8.75;
        result = api.moveTo(point, quaternion, false);
        Quaternion currentQuaternion = api.getRobotKinematics().getOrientation();
        int loopCounter = 0;
        while (calculateAngle(currentQuaternion, quaternion) <= MAX_THRESHOLD_Angle && loopCounter < loopMAX_time) {
            result = api.moveTo(point, quaternion, false);
            ++loopCounter;
        }
        return result.hasSucceeded();
    }

    public static double calculateAngle(Quaternion q1, Quaternion q2) {
        double dotProduct = q1.getW() * q2.getW() + q1.getX() * q2.getX() + q1.getY() * q2.getY() + q1.getZ() * q2.getZ();
        double q1Magnitude = Math.sqrt(q1.getW() * q1.getW() + q1.getX() * q1.getX() + q1.getY() * q1.getY() + q1.getZ() * q1.getZ());
        double q2Magnitude = Math.sqrt(q2.getW() * q2.getW() + q2.getX() * q2.getX() + q2.getY() * q2.getY() + q2.getZ() * q2.getZ());
        return Math.acos(dotProduct / (q1Magnitude * q2Magnitude));
    }

    private void reportAreaInfoAndEndRounding() {
        for(int areaNum = 1; areaNum <= 4; areaNum++){
            Pair<String, Integer> areaInfo = areasData.getMaxFreqItemData(areaNum);
            finalAreaData[areaNum] = areaInfo.first;
            if (areaInfo.first != null && areaInfo.second != null) {
                api.setAreaInfo(areaNum, areaInfo.first, areaInfo.second);
            } else { Log.i("Report", "areaInfo is null for areaNum: " + areaNum); }
        }
        api.reportRoundingCompletion();
    }

    public void sleep(long millis){
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void scanMove(){
        goPathPoint(scanPath_1);
        goPathPoint(scanPath_1_2);
        goPathPoint(scanPath_2);
        goPathPoint(scanPath_3);
        goPathPoint(scanPath_4);
        goPathPoint(scanPath_5);
        moveToWithRetry(astronautPointwithQuaternion, 5);
    }

    private void targetTask(){
        String targetItem = areasData.getMaxFreqItemData(0).first;
        Integer targetAreaNum = 4;
        for(int areaNum = 1; areaNum <= 4; areaNum++){
            if(finalAreaData[areaNum].equals(targetItem)){
                targetAreaNum = areaNum;
            }
        }
        if(areasData.getMaxFreqItemData(0)==null){
            Log.i("TARGET", "NULL");
            moveToWithRetry(targetPQ_area4, 5);
        }else{
            Log.i("TARGET", targetItem + targetAreaNum);
            switch (targetAreaNum){
                case 1:
                    goPathPoint(targetPQ_pathPoint1_1);
                    goPathPoint(targetPQ_pathPoint1_2);
                    goPathPoint(targetPQ_pathPoint1_3);
                    moveToWithRetry(targetPQ_area1, 5);
                    break;
                case 2:
                    goPathPoint(targetPQ_pathPoint2);
                    moveToWithRetry(targetPQ_area2, 5);
                    break;
                case 3:
                    goPathPoint(scanPath_4);
                    moveToWithRetry(targetPQ_area3, 5);
                    break;
                default:
                    moveToWithRetry(targetPQ_area4, 5);
                    break;
            }
        }
        Point robotPos = api.getRobotKinematics().getPosition();
        sleep(3500);
        Point error = calcArucoPos(calibImgWithMatrix(api.getMatNavCam(), navCamIntrinsicsMatrix), targetAreaNum);
        if(error != null){
            if(targetAreaNum == 2||targetAreaNum == 3){
                moveToWithRetry(new PointWithQuaternion(new Point(robotPos.getX() + error.getX(), robotPos.getY() - error.getY(), targetZ_area23), new Quaternion(0.5f, 0.5f, -0.5f, 0.5f)),5);
            }else if(targetAreaNum == 1){
                moveToWithRetry(new PointWithQuaternion(new Point(robotPos.getX() + error.getX(), targetY_area1, robotPos.getZ() + error.getY()), new Quaternion(0f, 0f, -0.707f, 0.707f)),5);
            }else {
                moveToWithRetry(new PointWithQuaternion(new Point(targetX_area4, robotPos.getY() - error.getX(), robotPos.getZ() + error.getY()), new Quaternion(0f,0f,-1f,0f)),5);
            }
        }
        api.takeTargetItemSnapshot();
    }

    private void goPathPoint(PointWithQuaternion pq){
        Point point = pq.point;
        Quaternion quaternion = pq.quaternion;
        Result result = api.moveTo(point, quaternion, false);
        while (!result.hasSucceeded()) {
            result = api.moveTo(point, quaternion, false);
        }
    }
}

