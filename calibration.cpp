#include "calibration.h"

//Calibration::Calibration()
//{
//    poseFlag = false;
//    calibrationFlag = false;
//}

Calibration::Calibration(QObject *parent)
    : QObject{parent}
{

}

cv::Mat Calibration::getUIImage()
{
    return uiImage;
}

void Calibration::intrinCalibration()
{

    std::vector<cv::String> fileNames;
    cv::glob("/home/phuc/images/my_photo*.jpg", fileNames, false);
    cv::Size patternSize( 10 - 1  , 7 - 1);
    std::vector<std::vector<cv::Point2f>> q(fileNames.size());

    std::vector<std::vector<cv::Point3f>> Q;
    // 1. Generate checkerboard (world) coordinates Q. The board has 25 x 18
    // fields with a size of 15x15mm

    int checkerBoard[2] = { 10 , 7 };
    // Defining the world coordinates for 3D points
      std::vector<cv::Point3f> objp;
      for(int i = 1; i<checkerBoard[1]; i++){
        for(int j = 1; j<checkerBoard[0]; j++){
          objp.push_back(cv::Point3f(j,i,0));
        }
      }

    std::vector<cv::Point2f> imgPoint;
    // Detect feature points
    std::size_t i = 0;
    for (auto const &f : fileNames) {
      std::cout << std::string(f) << std::endl;

      // 2. Read in the image an call cv::findChessboardCorners()
      cv::Mat img = cv::imread(fileNames[i]);
      cv::Mat gray;

      cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

      bool patternFound = cv::findChessboardCorners(gray, patternSize, q[i], cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

      // 2. Use cv::cornerSubPix() to refine the found corner detections
      if(patternFound){
          cv::cornerSubPix(gray, q[i],cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
          Q.push_back(objp);
      }

      // Display
      cv::drawChessboardCorners(img, patternSize, q[i], patternFound);
      cv::imshow("chessboard detection", img);
      cv::waitKey(0);

      i++;
    }

    cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
    cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
                cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
    cv::Size frameSize(640, 480);

    std::cout << "Calibrating..." << std::endl;
    // 4. Call "float error = cv::calibrateCamera()" with the input coordinates
    // and output parameters as declared above...

    float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, flags);

    std::cout << "Reprojection error = " << error << "\nK =\n"
              << K << "\nk=\n"
              << k << std::endl;

    // Precompute lens correction interpolation
    cv::Mat mapX, mapY;
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1,
                                mapX, mapY);

    // Show lens corrected images
    for (auto const &f : fileNames) {
      std::cout << std::string(f) << std::endl;

      cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);

      cv::Mat imgUndistorted;
      // 5. Remap the image using the precomputed interpolation maps.
      cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);
    }

}


bool Calibration::readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

void Calibration::arucoDetection(int camIndex)
{
    cv::VideoCapture cap;
    cv::Mat image;
    cap.open(camIndex);

    //---Camera Intrinsics Parameters:
    //-Focus Length = 2270.6 2260.6
    //-Central Point = 980.9766 394.1324

    cv::Mat cameraMatrix = cv::Mat( 3 , 3 , CV_64F, intriPara);

    std::cout << "Camera Matrix: " << cameraMatrix << endl;
    std::cout << "Distoration Co.: " << distCoeffs << endl;
//    float markerLength = 0.0005;
    float markerLength = 0.005;

    std::vector<cv::Vec3d> rvecs, tvecs;
    Mat rV_cv , tV_cv;
    //---Aruco Detector
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    //---Set coordinate system
    // Set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

//  ---Capture and get image
    //*with iteration = 5;
    while(true)
    {
        cap >> image;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        detector.detectMarkers(image, corners, ids);

        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(image, corners, ids);
            int nMarkers = corners.size();
            std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

            // Calculate pose for each marker
            for (int i = 0; i < nMarkers; i++)
            {
                //            solvePnP(objPoints, corners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
                solvePnP(objPoints, corners.at(i), cameraMatrix, distCoeffs, rV_cv, tV_cv);
//                cv::drawFrameAxes(image, cameraMatrix, distCoeffs, rV_cv, tV_cv, 0.01);
//                std::cout << "Translation Vector: " << endl;
//                std::cout << tV_cv.at<double>(0,0) * 1000<< endl;
//                std::cout << tV_cv.at<double>(1,0) * 1000<< endl;
//                std::cout << tV_cv.at<double>(2,0) * 1000<< endl;
//                std::cout << "Rotation Vector: " << endl;
//                std::cout << vpMath::deg(rV_cv.at<double>(0,0)) << endl;
//                std::cout << vpMath::deg(rV_cv.at<double>(1,0)) << endl;
//                std::cout << vpMath::deg(rV_cv.at<double>(2,0)) << endl;
            }
            break;
        }
    }
//    std::cout << "X: "<< tV_cv.at<double>(0,0) * 1000 << endl;
//    std::cout << "Y: "<< tV_cv.at<double>(1,0) * 1000 << endl;
//    std::cout << "Marker to image plane origin:" << endl;
//    std::cout << "X: "<< cx - abs(tV_cv.at<double>(0,0) * 1000) << endl;
//    std::cout << "Y: "<< cy - abs(tV_cv.at<double>(1,0) * 1000) << endl;

    //---Saving position---.
    xAruco = tV_cv.at<double>(0,0)*1000;
    yAruco = tV_cv.at<double>(1,0)*1000;
    std::cout << "xAruco: "<< xAruco << endl;
    std::cout << "yAruco: "<< yAruco << endl;
    cxAruco = central_x - abs(tV_cv.at<double>(0,0) * 1000);
    cyAruco = central_y - abs(tV_cv.at<double>(1,0) * 1000);
    std::cout << "cxAruco: "<< cxAruco << endl;
    std::cout << "cyAruco: "<< cyAruco << endl;
    //---Show resulting image and close window
//    imshow("ArUco", image);
    //---Output to UI
    uiImage = image;

//    while (cap.grab())
//    {
//        cv::Mat image;
//        cap >> image;

//        std::vector<int> ids;
//        std::vector<std::vector<cv::Point2f>> corners;
//        detector.detectMarkers(image, corners, ids);

//        // If at least one marker detected
//        if (ids.size() > 0)
//        {
//            cv::aruco::drawDetectedMarkers(image, corners, ids);
//            int nMarkers = corners.size();
//            std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

//            // Calculate pose for each marker
//            for (int i = 0; i < nMarkers; i++) {
//                solvePnP(objPoints, corners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
//            }
//            // Draw axis for each marker
//            for (int i = 0; i < rvecs.size(); ++i) {
//                auto rvec = rvecs[i];
//                auto tvec = tvecs[i];
//                cv::drawFrameAxes(image, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
//            }
//        }
//        // Show resulting image and close window
//        cv::imshow("Aruco Detection", image);
//        cv::waitKey(1);
//    }
    cap.release();
    //---End Aruco Detection.
}
void Calibration::getPose(Mat calibFrame)
{
    //---OpenCV Camera Grabber---
//    cv::VideoCapture cap;
//    cv::Mat calibFrame;
//    cap.open(camIndex);

//    poseFlag = false;
    //---Variables---


    vpRzyxVector rV;
    vpTranslationVector tV;
    // [Create camera parameters]
    vpCameraParameters cam;

    if(fullHDFlag)
    {
        cam.initPersProjWithoutDistortion(intriPara_FullHD[0], intriPara_FullHD[4], intriPara_FullHD[2], intriPara_FullHD[5]);
        central_x = intriPara_FullHD[2];
        central_y = intriPara_FullHD[5];
    }
    else
    {
        cam.initPersProjWithoutDistortion(intriPara[0], intriPara[4], intriPara[2], intriPara[5]);
        central_x = intriPara[2];
        central_y = intriPara[5];
    }

    //---GetPose---
//    while(!poseFlag)
//    {
        // Marco Define
        vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
        vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;

        // HOMOGRAPHY_VIRTUAL_VS
        vpImage<unsigned char> I;
        vpDisplayOpenCV d;


        // [Create AprilTag detector]
        vpDetectorAprilTag detector(tagFamily);

        // [AprilTag detector settings]
        detector.setAprilTagQuadDecimate(quad_decimate);
        detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
        detector.setAprilTagNbThreads(nThreads);
        detector.setDisplayTag(display_tag, color_id < 0 ? vpColor::none : vpColor::getColor(color_id), thickness);
        detector.setZAlignedWithCameraAxis(z_aligned);

        // [Detect and compute pose]
        vpImageConvert::convert(calibFrame, I);
        detector.detect(I, tagSize, cam, cMo_vec);
//        qDebug() << "Pose detection: " << cMo_vec.size();

        // [Stop while loop when detection succeed
        if(cMo_vec.size() != 0)
        {
            //---Storing cMo.

            cMo = cMo_vec.at(0);
            //---Displaying parameters.
            rM = cMo.getRotationMatrix();
            rV = rV.buildFrom(rM);
            tV = cMo.getTranslationVector();

            cMo.buildFrom(tV, rM);
            std::cout << "Detections: " << cMo_vec.size() << std::endl;
            std::cout << "Translation Vector: " << std::endl;
            std::cout << "X: " << tV[0]*1000 << std::endl;
            std::cout << "Y: " << tV[1]*1000 << std::endl;
            std::cout << "Z: " << tV[2]*1000 << std::endl;
            std::cout << "Rzyx Vector: " << std::endl;
            std::cout << "Rz: " << vpMath::deg(rV[0]) << std::endl;
            std::cout << "Ry: " << vpMath::deg(rV[1]) << std::endl;
            std::cout << "Rx: " << vpMath::deg(rV[2]) << std::endl;
//          [Display]
//---------------------------------------------------------------------
//            for (size_t i = 0; i < cMo_vec.size(); i++)
//            {
//                try {
//#if defined(VISP_HAVE_X11)
//                    vpDisplayX d(I);
//#elif defined(VISP_HAVE_GDI)
//                    vpDisplayGDI d(I);
//#endif
//                    vpDisplay::display(I);
//                    vpDisplay::displayFrame(I, cMo_vec[i], cam, tagSize / 2, vpColor::none, 3);
//                    vpDisplay::flush(I);
//                    vpDisplay::getClick(I);

//                } catch (const vpException &e) {
//                    qDebug() << "Catch an exception!";
//                }
//            }
//-----------------------------------------------------------------------

            poseFlag = true;
//            break;
//        }
    }
}

vector<vpHomogeneousMatrix> Calibration::getcMo_vec()
{
    return cMo_vec;
}

vpHomogeneousMatrix Calibration::getOnecMo()
{
    return cMo_vec.at(0);
}

vpHomogeneousMatrix Calibration::getbTc()
{
    return bTc;
}

void Calibration::handEyeCalibration(QVector<double> robot_position,vpHomogeneousMatrix cTo)
{
#if (defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_OPENCV))
  try {

        //--------------------------------------------
        //---Calculating eTb
        qDebug() << "Robot Position: " << robot_position ;
        vpTranslationVector etc(robot_position[0]/1000, robot_position[1]/1000, robot_position[2]/1000);
        vpRzyxVector erc(vpMath::rad(robot_position[5]),vpMath::rad(robot_position[4]) ,vpMath::rad(robot_position[3]));
        vpRotationMatrix ERC;
        ERC.buildFrom(erc);

        vpHomogeneousMatrix bTe, eTb;
        bTe.buildFrom(etc, ERC);

        tV = bTe.getTranslationVector(); //Translation Vector
        rV = rV.buildFrom(bTe.getRotationMatrix()); //Rotation Matrix

        cout << "-----" << endl;
        cout << "bTe" << endl;

        cout << "Translation Vector: " << endl;
        cout << "X: " << tV[0]*1000 << endl;
        cout << "Y: " << tV[1]*1000 << endl;
        cout << "Z: " << tV[2]*1000 << endl;
        cout << "***" << endl;
        cout << "Rzyx Vector: " << endl;
        cout << "Rz: " << vpMath::deg(rV[0]) << endl;
        cout << "Ry: " << vpMath::deg(rV[1]) << endl;
        cout << "Rx: " << vpMath::deg(rV[2]) << endl;

        //---Inverse base to e.o.f matrix
        bTe.inverse(eTb);
        eTb_vec.push_back(eTb);

        tV = eTb.getTranslationVector();
        rV = rV.buildFrom(eTb.getRotationMatrix());

        cout << "-----" << endl;
        cout << "eTb" << endl;
        cout << "Translation Vector: " << endl;
        cout << "X: "<< tV[0]*1000  << endl;
        cout << "Y: " << tV[1]*1000 << endl;
        cout << "Z: " << tV[2]*1000 << endl;
        cout << "***" << endl;
        cout << "Rzyx Vector: " << endl;
        cout << "Rz: " << vpMath::deg(rV[0]) << endl;
        cout << "Ry: " << vpMath::deg(rV[1]) << endl;
        cout << "Rx: " << vpMath::deg(rV[2]) << endl;

        //--------------------------------------------
        //---Calculating cTo
        tV = cTo.getTranslationVector();
        rV = rV.buildFrom(cTo.getRotationMatrix());

        cTo_vec.push_back(cTo);
        cout << "-----" << endl;
        cout << "cTo " << endl;
        cout << "Translation Vector: " << endl;
        cout << "X: " << tV[0]*1000 << endl;
        cout << "Y: " << tV[1]*1000 << endl;
        cout << "Z: " << tV[2]*1000 << endl;
        cout << "***" << endl;
        cout << "Rzyx vector: " << endl;
        cout << "Rz: " << vpMath::deg(rV[0]) << endl;
        cout << "Ry: " << vpMath::deg(rV[1]) << endl;
        cout << "Rx: " << vpMath::deg(rV[2]) << endl;

        // Compute the eMc hand to eye transformation from six poses
        // - cTo[6]: camera to object poses as six homogeneous transformations
        // - cTo_vec: camera to object poses vector
        // - bTe_vec (wMe) : base to hand (end-effector) poses as six homogeneous
        // transformations :
        // Calculation: int ret = vpHandEyeCalibration::calibrate(cMo, wMe, eMc);

        //---Compute Extrinsics Matrix---
        // Calculation: int calibrate(
        //   const std::vector<vpHomogeneousMatrix> &cMo
        //  ,const std::vector<vpHomogeneousMatrix> &rMe
        //  ,vpHomogeneousMatrix &eMc);

        if(calibrationFlag == true)
        {
            //Saving eTb and cTo for later check

            vpHomogeneousMatrix bMc; // hand (end-effector) to eye (camera) transformation
            bMc.eye();
            std::cout << "Calculating %HandEyeCalibration% ..." << endl;
            int ret = vpHandEyeCalibration::calibrate(cTo_vec, eTb_vec, bMc);
            std::cout << "ret: " << ret << endl;

            bTc = bMc;
            std::cout << "bTc: " << std::endl;
            std::cout << bTc << std::endl;

            //---Showing bTc
            tV = bMc.getTranslationVector();
            rV = rV.buildFrom(bMc.getRotationMatrix());

            std::cout << "-----" << endl;
            std::cout << "bMc " << endl;
            std::cout << "Translation Vector: " << endl;
            std::cout << "X: " << tV[0]*1000 << endl;
            std::cout << "Y: " << tV[1]*1000 << endl;
            std::cout << "Z: " << tV[2]*1000 << endl;
            std::cout << "***" << endl;
            std::cout << "Rzyx vector: " << endl;
            std::cout << "Rz: " << vpMath::deg(rV[0]) << endl;
            std::cout << "Ry: " << vpMath::deg(rV[1]) << endl;
            std::cout << "Rx: " << vpMath::deg(rV[2]) << endl;

            calibrationFlag = false;
        }
    }
        //---Notify any errors---
        catch (const vpException &e) {
        std::cout << "Catch an exception: " << e << std::endl;
    }
#else
    std::cout << "Cannot run this example: install Lapack, Eigen3 or OpenCV" << std::endl;
    return EXIT_SUCCESS;
#endif
}


void Calibration::getPose_RealSense()
{
    //-------------------Marco Defined----------------------------
    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;

    vpRealSense2 g;

    rs2::config config;
    unsigned int width = 640, height = 480;
    config.enable_stream(RS2_STREAM_COLOR, static_cast<int>(width), static_cast<int>(height), RS2_FORMAT_RGBA8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, static_cast<int>(width), static_cast<int>(height), RS2_FORMAT_Z16, 30);
    config.enable_stream(RS2_STREAM_INFRARED, static_cast<int>(width), static_cast<int>(height), RS2_FORMAT_Y8, 30);

    g.open(config);
    vpImage<unsigned char> I;
    vpImage<vpRGBa> I_color(height, width);
    vpImage<uint16_t> I_depth_raw(height, width);
    vpImage<vpRGBa> I_depth;

    const float depth_scale = g.getDepthScale();

    rs2::align align_to_color = RS2_STREAM_COLOR;

    g.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap), reinterpret_cast<unsigned char *>(I_depth_raw.bitmap),
              NULL, NULL, &align_to_color);
    vpCameraParameters cam;
    cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);

    //! [Construct grabber]
    std::cout << cam << std::endl;
//    std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
//    std::cout << "tagFamily: " << tagFamily << std::endl;
//    std::cout << "nThreads : " << nThreads << std::endl;
//    std::cout << "Z aligned: " << align_frame << std::endl;

    vpImage<vpRGBa> I_color2 = I_color;
    vpImage<float> depthMap;
    vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

//    vpDisplay *d1 = NULL;
//    vpDisplay *d2 = NULL;
//    vpDisplay *d3 = NULL;

//    d1 = new vpDisplayX(I_color, 100, 30, "Pose from Homography");
//    d3 = new vpDisplayX(I_depth, 100, I_color.getHeight() + 70, "Depth");
//    d2 = new vpDisplayX(I_color2, I_color.getWidth() + 120, 30, "Pose from RGBD fusion");

    //! [Create AprilTag detector]
    vpDetectorAprilTag detector(tagFamily);

    //! [Create AprilTag detector]

    //! [AprilTag detector settings]
    detector.setAprilTagQuadDecimate(quad_decimate);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setAprilTagNbThreads(nThreads);
    detector.setDisplayTag(display_tag, color_id < 0 ? vpColor::none : vpColor::getColor(color_id), thickness);
    detector.setZAlignedWithCameraAxis(align_frame);

    //! [AprilTag detector settings]

//    std::vector<double> time_vec;

//    for (;;)
//    {
//      double t = vpTime::measureTimeMs();

//      //! [Acquisition]
//      g.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap),
//                reinterpret_cast<unsigned char *>(I_depth_raw.bitmap), NULL, NULL, &align_to_color);
//      //! [Acquisition]

//      I_color2 = I_color;
//      vpImageConvert::convert(I_color, I);
//      vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

//      depthMap.resize(I_depth_raw.getHeight(), I_depth_raw.getWidth());
//#ifdef VISP_HAVE_OPENMP
//#pragma omp parallel for
//#endif
//      for (int i = 0; i < static_cast<int>(I_depth_raw.getHeight()); i++) {
//        for (int j = 0; j < static_cast<int>(I_depth_raw.getWidth()); j++) {
//          if (I_depth_raw[i][j]) {
//            float Z = I_depth_raw[i][j] * depth_scale;
//            depthMap[i][j] = Z;
//          } else {
//            depthMap[i][j] = 0;
//          }
//        }
//      }

//      vpDisplay::display(I_color);
//      vpDisplay::display(I_color2);
//      vpDisplay::display(I_depth);

      g.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap),
                reinterpret_cast<unsigned char *>(I_depth_raw.bitmap), NULL, NULL, &align_to_color);
      I_color2 = I_color;
      vpImageConvert::convert(I_color, I);


      detector.detect(I, tagSize, cam, cMo_vec);
      std::cout << "Detections: " << cMo_vec.size() << std::endl;
      // Display camera pose for each tag
      if(cMo_vec.size() != 0)
      {
          //---Storing cMo.

          cMo = cMo_vec.at(0);
          //---Displaying parameters.
          rM = cMo.getRotationMatrix();
          rV = rV.buildFrom(rM);
          tV = cMo.getTranslationVector();
          //---Offset x value
//            tV[0] += -0.02;
          cMo.buildFrom(tV, rM);
//          std::cout << "Detections: " << cMo_vec.size() << std::endl;
//          std::cout << "Translation Vector: " << std::endl;
//          std::cout << tV[0]*1000 << std::endl;
//          std::cout << tV[1]*1000 << std::endl;
//          std::cout << tV[2]*1000 << std::endl;
//          std::cout << "RzyxVector: " << std::endl;
//          std::cout << vpMath::deg(rV[0]) << std::endl;
//          std::cout << vpMath::deg(rV[1]) << std::endl;
//          std::cout << vpMath::deg(rV[2]) << std::endl;
          poseFlag = true;
      }
//      for (size_t i = 0; i < cMo_vec.size(); i++)
//      {

//      }

//      //! [Pose from depth map]
//      std::vector<std::vector<vpImagePoint> > tags_corners = detector.getPolygon();
//      std::vector<int> tags_id = detector.getTagsId();
//      std::map<int, double> tags_size;
//      tags_size[-1] = tagSize; // Default tag size

//      std::vector<std::vector<vpPoint> > tags_points3d = detector.getTagsPoints3D(tags_id, tags_size);
//      for (size_t i = 0; i < tags_corners.size(); i++)
//      {
//        vpHomogeneousMatrix cMo;
//        double confidence_index;
//        if (vpPose::computePlanarObjectPoseFromRGBD(depthMap, tags_corners[i], cam, tags_points3d[i], cMo,
//                                                    &confidence_index)) {
//          if (confidence_index > 0.5) {
//            vpDisplay::displayFrame(I_color2, cMo, cam, tagSize / 2, vpColor::none, 3);
//          } else if (confidence_index > 0.25) {
//            vpDisplay::displayFrame(I_color2, cMo, cam, tagSize / 2, vpColor::orange, 3);
//          } else {
//            vpDisplay::displayFrame(I_color2, cMo, cam, tagSize / 2, vpColor::red, 3);
//          }
//          std::stringstream ss;
//          ss << "Tag id " << tags_id[i] << " confidence: " << confidence_index;
//          vpDisplay::displayText(I_color2, 35 + i * 15, 20, ss.str(), vpColor::red);
//        }
//      }

//      //! [Pose from depth map]

//      vpDisplay::displayText(I_color, 20, 20, "Pose from homography + VVS", vpColor::red);
//      vpDisplay::displayText(I_color2, 20, 20, "Pose from RGBD fusion", vpColor::red);
//      vpDisplay::displayText(I_color, 35, 20, "Click to quit.", vpColor::red);
//      t = vpTime::measureTimeMs() - t;
//      time_vec.push_back(t);

////      std::stringstream ss;
////      ss << "Detection time: " << t << " ms for " << detector.getNbObjects() << " tags";
////      vpDisplay::displayText(I_color, 50, 20, ss.str(), vpColor::red);

//      if (vpDisplay::getClick(I_color, false))
//        break;

//      vpDisplay::flush(I_color);
//      vpDisplay::flush(I_color2);
//      vpDisplay::flush(I_depth);
//    }
}

