#include "detector.h"

Detector::Detector(QObject *parent)
    : QObject{parent}
{

}
void Detector::draw_label(Mat& input_image, string label, int left, int top)
{
    // Display the label at the top of the bounding box.
    int baseLine;
    Size label_size = getTextSize(label, FONT_FACE, FONT_SCALE, THICKNESS, &baseLine);
    top = max(top, label_size.height);
    // Top left corner.
    Point tlc = Point(left, top);
    // Bottom right corner.
    Point brc = Point(left + label_size.width, top + label_size.height + baseLine);
    // Draw white rectangle.
    rectangle(input_image, tlc, brc, BLACK, FILLED);
    // Put the label on the black rectangle.
    putText(input_image, label, Point(left, top + label_size.height), FONT_FACE, FONT_SCALE, YELLOW, THICKNESS);
}

vector<Mat> Detector::pre_process(Mat &input_image, Net &net)
{
    // Convert images to blob.
    Mat blob;
    blobFromImage(input_image, blob, 1./255., Size(INPUT_WIDTH, INPUT_HEIGHT), Scalar(), true, false);

    net.setInput(blob);
    // Forward propagate.
    vector<Mat> outputs;

    net.forward(outputs, net.getUnconnectedOutLayersNames());
    return outputs;
}

Mat Detector::post_process(Mat &input_image, vector<Mat> &outputs, const vector<string> &class_name)
{
    //   Initialize vectors to hold respective outputs while unwrapping detections.
    vector<int> class_ids;
    vector<float> confidences;
    vector<Rect> boxes;

    //   Resizing factor.
    float x_factor = input_image.cols / INPUT_WIDTH;
    float y_factor = input_image.rows / INPUT_HEIGHT;
    float *data = (float *)outputs[0].data;

    // const int dimensions = 6;
    // 25200 for default size 640.
    const int rows = 25200;

    // Iterate through 25200 detections // all 25200 bboxes
    for (int i = 0; i < rows; ++i)
    {
        float confidence = data[4];
        // Discard bad detections and continue.

        if (confidence >= CONFIDENCE_THRESHOLD)
        {
            float *classes_scores = data + 5;

            // Create a 1x85 Mat and store class scores of 1 classes.
            Mat scores(1, class_name.size(), CV_32FC1, classes_scores);

            // Perform minMaxLoc and acquire the index of best class score.
            Point class_id;
            double max_class_score;
            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

            // Continue if the class score is above the threshold.
            if (max_class_score > SCORE_THRESHOLD)
            {
                // Store class ID and confidence in the pre-defined respective vectors.
                confidences.push_back(confidence);
                class_ids.push_back(class_id.x);

                // Center of bbox.
                double cx = data[0];
                double cy = data[1];
                std::cout << "cx " << cx << std::endl;
                std::cout << "cy " << cy << std::endl;
                // Box dimension.
                double w = data[2];
                double h = data[3];

                // Bounding box coordinates.
                double left = double((cx - 0.5 * w) * x_factor);
                double top = double((cy - 0.5 * h) * y_factor);
                double width = double(w * x_factor);
                double height = double(h * y_factor);

                // Store good detections in the boxes vector.
                boxes.push_back(Rect(left, top, width, height));
            }
        }
        // Jump to the next row.
        // data += 6; 5 para + 1 class
        data += 6;
    }
    // Perform Non-Maximum Suppression and draw predictions.
    vector<int> indices;
    NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, indices);
    qDebug() << indices.size() ;

    for (int i = 0; i < indices.size(); i++)
    {
        int idx = indices[i];
        Rect box = boxes[idx];
        double left = box.x;
        double top = box.y;
        double width = box.width;
        double height = box.height;

        //--Get to central point of bounding box
        double x = (left + 0.5*width)/x_factor;
        //---Y has to multiply by y_factor YOLO 640x640 -> Camera 640x480.
        double y = ((top + 0.5*height)/y_factor)*y_factor;
        x_vec.push_back(x);
        y_vec.push_back(y);

        //--Draw bounding box.
        rectangle(input_image, Point(left, top), Point(left + width, top + height), BLUE, 3*THICKNESS);

        // Get the label for the class name and its confidence.
//        string label = format("%.2f", confidences[idx]);
//        label = class_name[class_ids[idx]] + ":" + label;
          string label = std::to_string(idx);
        // Draw class labels.
        draw_label(input_image, label, left, top);
    }
    return input_image;
}

//---Background substraction algorithm-------------
void Detector::bgSubtraction(Mat bg_img, Mat fg_img)
{

    //-------------------------------
    cvtColor( bg_img, bg_img , cv::COLOR_BGR2GRAY );
    cvtColor( fg_img, fg_img , cv::COLOR_BGR2GRAY );
    //---Select ROI
    Rect2d roi = selectROI(fg_img);

    bg_img = bg_img(roi);
    fg_img = fg_img(roi);

    //---Crop Img------

    std::cout << "Channels: " << fg_img.channels() << std::endl;

//    bg_img = bg_img(Range(405,825), Range(980,1385)); // 1080 - 1920 y - x
//    fg_img = fg_img(Range(405,825), Range(980,1385)); //
//    imshow("FG_IMG CROPPED", fg_img);
    //---Blur the image with 3x3 Gaussian kernel

    cv::GaussianBlur( bg_img, bg_img, Size(5, 5), 0);
    cv::GaussianBlur( fg_img, fg_img, Size(5, 5), 0);
    cv::Mat mask = fg_img - bg_img;
    mask = cv::abs(mask);

    //---Thresholding
    threshold( mask, mask, threshold_value, max_BINARY_value,threshold_type );
    //---Median Filter
    cv::medianBlur( mask, mask, 3);
    //---Opening
    int operation = morph_operator + 2;
    Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
    morphologyEx( mask, mask, operation, element );

//    for(int i = 0; i < mask.rows; i++)
//    {
//        for(int j = 0; j < mask.cols; j++)
//        {
//            if( mask.at<uchar>(i,j) <= 75)
//                mask.at<uchar>(i,j) = 0;
//            else
//                mask.at<uchar>(i,j) = 255;
//        }
//    }
    //---Median Blur

    imshow("MASK", mask);
    //---Labeling connected component
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    int connectivity = 8;

    int numOfLabels = cv::connectedComponentsWithStats(mask, labels, stats, centroids, connectivity);
    std::cout << "num: " << numOfLabels << std::endl;
    std::cout << centroids << std::endl;
    //---X first Y (V) second
    for(int i = 1; i < numOfLabels ; i++)
    {
        std::cout << "====================" << std::endl;
//        std::cout << centroids.at<double>(i,0) << std::endl;
//        std::cout << centroids.at<double>(i,1) << std::endl;
        std::cout << "i = " <<  i << std::endl;
        double cxPad, cyPad;
        cxPad = centroids.at<double>(i,0) + roi.x + 5; //Get to the right of pad-
                                                      //X axis plus 5 pixels for better positioning of the soldering tool
        cyPad = centroids.at<double>(i,1) + roi.y;
        std::cout << "cxPad: " << cxPad << " " ;
        std::cout << "cyPad: " << cyPad << std::endl;
        cxPad_vec.push_back(cxPad);
        cyPad_vec.push_back(cyPad);
    }
}

void Detector::blobDetection(Mat bg_img)
{
    //---Blob detections
    SimpleBlobDetector::Params params;
    std::vector<KeyPoint> keypoints;
    Mat im_with_keypoints;
    vector<Mat> im_with_keypoints_vec;
    cv::Mat win_mat(cv::Size(1280, 480), CV_8UC3);

    //---Select ROIs
    double cxPad;
    double cyPad;
    int pad_detections = 0;
    std::vector<Rect> rois;
    selectROIs("CROPPED", bg_img, rois);
    std::cout << "ROIs size: " << rois.size() << std::endl;
    //---Crop image
    vector<Mat> crop_img;

    for(uint8_t i = 0; i < rois.size(); i++)
    {
        crop_img.push_back(bg_img(rois.at(i)));

        //----blobDetections

        // Setup SimpleBlobDetector parameters.
        //----Change thresholds
        params.minThreshold = 50;
        params.maxThreshold = 200;

        //----Filter by Area.
        params.filterByArea = true;
        params.minArea = 35;
        params.maxArea = 200;

        //----Filter by Circularity
        params.filterByCircularity = true;
        params.minCircularity = 0.1;

        //----Filter by Convexity
        params.filterByConvexity = true;
        params.minConvexity = 0.75;

        //----Filter by Inertia
        params.filterByInertia = true;
        params.minInertiaRatio = 0.01;

        //----Detect blobs --- named keypoints.
        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        detector->detect(crop_img.at(i), keypoints);

        //---Get the results---------------------------------
        for(int j = keypoints.size() - 1 ; j >= 0; j--)
        {
            cv::KeyPoint pad;
            std::cout << "PAD --> " << j + 1  << std::endl;
            pad = keypoints.at(j);

            cxPad = pad.pt.x + rois.at(i).x + 5; //Get to the right of pad-
                                                //X axis plus 5 pixels for better positioning of the soldering tool
            cyPad = pad.pt.y + rois.at(i).y;

            std::cout << "xPad: " << cxPad << std::endl;
            std::cout << "yPad: " << cyPad << std::endl;
            cxPad_vec.push_back(cxPad);
            cyPad_vec.push_back(cyPad);
        }
        pad_detections += keypoints.size();

        //---Draw detected blobs as red circles.
        //---DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
        drawKeypoints(crop_img.at(i), keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        im_with_keypoints_vec.push_back(im_with_keypoints);
        //----Clear keypoints
        keypoints.clear();
    }

    std::cout << "Pad Detections: " << pad_detections << std::endl;

    for(int i = 0; i < im_with_keypoints_vec.size(); i++)
    {
        std::to_string(i);
        imshow("IMG " + i, im_with_keypoints_vec.at(i));
    }
}

vector<double> Detector::getcxPadVec()
{
    return cxPad_vec;
}

vector<double> Detector::getcyPadVec()
{
    return cyPad_vec;
}


void Detector::clearPadvec()
{
    cxPad_vec.clear();
    cyPad_vec.clear();
}
