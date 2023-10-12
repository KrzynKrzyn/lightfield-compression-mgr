#ifndef REGULAR_COMPRESSOR_HPP
#define REGULAR_COMPRESSOR_HPP

#include <filesystem>
#include <opencv2/opencv.hpp>

// void fixTarot(std::string path, std::string outputPath, int originalX, int originalY)
// {
//     int newX = 16 - originalX;
//     std::string sy = originalY < 10 ? "0"+std::to_string(originalY) : std::to_string(originalY);
//     std::string sx = newX < 10 ? "0"+std::to_string(newX) : std::to_string(newX);

//     std::string newFilename = "out_" + sy + "_" + sx + "_.png";
//     std::string newPath = outputPath + newFilename;

//     std::filesystem::copy_file(path, newPath);
// }

// for (int y = 0; y < lfModel.rows; y += 1)
// {
//     for (int x = 0; x < lfModel.cols; x += 1)
//     {
//         fixTarot(lfModel.getImagePath(x, y), "D:\\Files\\Studia\\mgr-compression-cversion2\\data\\stanford_tarot_fixed\\rectified\\", x, y);
//     }
// }
// return 0;

    // int minV = -25, maxV = 25;
    // int quantizationRange = 1;
    // int quantizationStep = 2 * quantizationRange + 1;
    // for (int value = minV; value < maxV; ++value)
    // {
    //     int qValue = value < 0 ? (value - quantizationRange) / quantizationStep : (value + quantizationRange) / quantizationStep;
    //     qValue *= quantizationStep;
    //     std::cout << value << "->" << qValue << std::endl;
    // }

    // return 0;


cv::Point2i getNearestPoint(const cv::Point2i p, const std::vector<cv::Point2i> points)
{

    double nearestDistance = 1000000;
    cv::Point2i nearestPoint;
    for (const auto &rp : points)
    {
        int xDist = p.x - rp.x, yDist = p.y - rp.y;
        double distance = sqrt(xDist * xDist + yDist * yDist);
        if (distance < nearestDistance)
        {
            nearestDistance = distance;
            nearestPoint = rp;
        }
    }

    return nearestPoint;
}

void multiRefModeling()
{
    cv::Mat distanceModel(cv::Size(17, 17), CV_32FC1);
    // std::vector<cv::Point2i> refPoints{{8,8}};
    // std::vector<cv::Point2i> refPoints{{4,4}, {12, 12}};
    // std::vector<cv::Point2i> refPoints{{4,4}, {12, 12}, {4, 12}, {12, 4}};
    std::vector<cv::Point2i> refPoints{{3, 3}, {13, 13}, {3, 13}, {13, 3}, {8, 8}};

    for (int i = 0; i < 17; ++i)
        for (int j = 0; j < 17; ++j)
        {
            auto refPoint = getNearestPoint(cv::Point(i, j), refPoints);
            int iDist = abs(refPoint.x - i), jDist = abs(refPoint.y - j);
            distanceModel.at<float>(i, j) = sqrt(iDist * iDist + jDist * jDist);
        }

    cv::Mat bppModel(cv::Size(17, 17), CV_32FC1);
    for (int i = 0; i < 17; ++i)
        for (int j = 0; j < 17; ++j)
        {
            bppModel.at<float>(i, j) = (distanceModel.at<float>(i, j) + 45.4191461) / 9.0556274;
        }

    for (const auto &rp : refPoints)
    {
        bppModel.at<float>(rp) = 14.000001;
    }

    // std::cout << distanceModel << std::endl << std::endl;
    std::cout << bppModel << std::endl
              << std::endl;
    std::cout << std::endl
              << "MEAN BPP: " << cv::mean(bppModel) << std::endl;

    cv::Mat bppToShow;
    bppModel.copyTo(bppToShow);
    bppToShow *= -1;
    bppToShow += 10;
    bppToShow *= 25.5;
    bppToShow.convertTo(bppToShow, CV_8UC1);
    bppToShow.convertTo(bppToShow, CV_8UC3);
    cv::cvtColor(bppToShow, bppToShow, cv::COLOR_GRAY2BGR);
    cv::Mat fuckignMask(bppToShow.size(), bppToShow.type());
    fuckignMask.setTo(cv::Scalar(255, 0, 0));
    bppToShow.setTo(cv::Scalar(0, 0, 0), fuckignMask);

    cv::resize(bppToShow, bppToShow, cv::Size(), 64, 64, 0);
    for (int x = 0; x < bppModel.cols; ++x)
        for (int y = 0; y < bppModel.rows; ++y)
        {
            float bpp = bppModel.at<float>(y, x);
            std::stringstream stream;
            stream << std::fixed << std::setprecision(2) << bpp;
            std::string bppText = stream.str();
            if (bppText.size() <= 4)
                cv::putText(bppToShow, bppText, {64 * x + 6, 64 * y + 40}, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.75, {0, 0, 0}, 1);
        }
    cv::imshow("BPP", bppToShow);
    cv::waitKey(0);
    cv::destroyAllWindows();
}


void resizeImages(std::string source, std::string destination) {
    std::vector<std::string> imageFilenames;

    for (const auto &entry : std::filesystem::directory_iterator(source))
    {
        if(entry.path().extension() == ".png" || entry.path().extension() == ".JPG") {
            auto path = entry.path();
            auto filename = entry.path().filename().string();
            auto sourcePath = path.string();
            auto filenameNoExt = entry.path().filename().replace_extension("png").string();
            auto destPath = destination + filenameNoExt;

            cv::Mat image = cv::imread(sourcePath);
            cv::Mat resizedImage;
            cv::resize(image, resizedImage, cv::Size(), 0.5, 0.5, 0);
            resizedImage = resizedImage(cv::Rect(0, 0, 1920, 1280));
            cv::imwrite(destPath, resizedImage);
            // imageFilenames.push_back(entry.path();
        }
    }
}

void savePPM(std::string source, std::string destination) {
    std::vector<std::string> imageFilenames;

    for (const auto &entry : std::filesystem::directory_iterator(source))
    {
        if(entry.path().extension() == ".png" || entry.path().extension() == ".JPG") {
            auto path = entry.path();
            auto filename = entry.path().filename().string();
            auto sourcePath = path.string();
            auto filenameNoExt = entry.path().filename().replace_extension("ppm").string();
            auto destPath = destination + filenameNoExt;

            cv::Mat image = cv::imread(sourcePath);
            // cv::resize(image, resizedImage, cv::Size(), 0.5, 0.5, 0);
            // resizedImage = resizedImage(cv::Rect(0, 0, 1920, 1280));
            cv::imwrite(destPath, image);
            // imageFilenames.push_back(entry.path();
        }
    }
}

void copyForHevc(std::string source, std::string destination) {
    std::vector<std::string> imageFilenames;

    int i=0;
    for (const auto &entry : std::filesystem::directory_iterator(source))
    {
        if(entry.path().extension() == ".png" || entry.path().extension() == ".JPG") {
            std::string numberSting;
            numberSting = std::to_string(i);
            if (numberSting.size() == 1) numberSting = "00" + numberSting;
            if (numberSting.size() == 2) numberSting = "0" + numberSting;

            auto path = entry.path();
            auto filename = entry.path().filename().string();
            auto sourcePath = path.string();
            auto filenameNoExt = entry.path().filename().replace_extension("png").string();
            auto destPath = destination + "image" + numberSting + ".png";

            std::cout << destPath << std::endl;

            cv::Mat image = cv::imread(sourcePath);
            cv::imwrite(destPath, image);

            ++i;
        }
    }
}


#endif