#define VISUALIZE false

#include <opencv2/opencv.hpp>

#include "src/lightfield_model/lightfield_model.hpp"
#include "src/lightfield_model/compression_cost_model.hpp"
#include "src/lightfield_compressor.hpp"

#include "src/visualization/lightfield_map_painter.hpp"

int main(int argc, char const *argv[])
{
    // const int lfCols = 4, lfRows = 4;
    // lfc::LightfieldModel lfModel(lfCols, lfRows,
    //     "D:\\Files\\Studia\\mgr-compression-cversion2\\data\\interdigital_hands\\", true, true);

    // const int lfCols = 9, lfRows = 9;
    // lfc::LightfieldModel lfModel(lfCols, lfRows,
    //  "N:\\StudiaMgr\\full_data_lightfield_analysis\\full_data\\additional\\rosemary\\", true, true);

    // lego, tarot, tarot_alt, bracelet
    const int lfCols = 17, lfRows = 17;
    lfc::LightfieldModel lfModel(lfCols, lfRows,
    "D:\\Files\\Studia\\mgr-compression-cversion2\\data\\stanford_lego\\rectified\\", false, false);

    // const int lfCols = 17, lfRows = 17;
    // lfc::LightfieldModel lfModel(lfCols, lfRows,
    // "D:\\Files\\Studia\\mgr-compression-cversion2\\data\\stanford_truck\\rectified\\", true, false);

    int epsilon = 3;     // 0 to infinity
    double theta = 0.08; // -1 to 1
    
    lfc::LightfieldCompressor compressor;
    lfc::CompressionCostModel costModel = compressor.compress(lfModel, epsilon, theta);

    // ============================================================== RESULT VISUALIZATION =============================================================

    const auto overallCost = costModel.getOverallCost();
    auto someImage = cv::imread(lfModel.getImagePath(0, 0));

    cv::Mat memoCostMat(lfCols, lfRows, CV_32SC1);
    cv::Mat timeCostMat(lfCols, lfRows, CV_32FC1);
    cv::Mat bppMat(lfCols, lfRows, CV_32FC1);
    cv::Mat predictionErrorMat(lfCols, lfRows, CV_32FC1);
    std::cout << "Overall compression results PER IMAGE: " << std::endl;
    for (int y = 0; y < lfModel.rows; y += 1)
    {
        for (int x = 0; x < lfModel.cols; x += 1)
        {
            memoCostMat.at<int32_t>(y, x) = costModel.getCostCell(x, y).overallMemoCost() / 1000;
            timeCostMat.at<float>(y, x) = (double)costModel.getCostCell(x, y).overallTimeCost() / (1000 * 60 * 60);
            bppMat.at<float>(y, x) = 8.0 * ((double)costModel.getCostCell(x, y).overallMemoCost() / (double)someImage.size().area());
            predictionErrorMat.at<float>(y, x) = ((double)costModel.getCostCell(x, y).predictionError / (someImage.size().area() * someImage.channels()));
            // std::cout << x << ", " << y << '\t' << "Memo cost: " << costModel.getCostCell(x, y).encodedDataMemoryCost / 1000 << " kb" << std::endl;
        }
    }

    std::cout << "Memo cost: " << std::endl;
    std::cout << memoCostMat << std::endl
              << std::endl;

    std::cout << "Time cost: " << std::endl;
    std::cout << timeCostMat << std::endl
              << std::endl;

    std::cout << "BPP: " << std::endl;
    std::cout << bppMat << std::endl
              << std::endl;

    std::cout << "Prediction error: " << std::endl;
    std::cout << predictionErrorMat << std::endl
              << std::endl;

    std::cout << std::endl
              << "Overall compression results MEMO: " << std::endl;
    std::cout << '\t' << "Partition memo cost: " << overallCost.partitionMemoryCost / 1000 << " kb"
              << " (" << 100 * (double)overallCost.partitionMemoryCost / (double)overallCost.overallMemoCost() << "%)" << std::endl;
    std::cout << '\t' << "Prediction memo cost: " << overallCost.blockMatchingMemoryCost / 1000 << " kb"
              << " (" << 100 * (double)overallCost.blockMatchingMemoryCost / (double)overallCost.overallMemoCost() << "%)" << std::endl;
    std::cout << '\t' << "Filtering memo cost: " << overallCost.filteringMemoryCost / 1000 << " kb"
              << " (" << 100 * (double)overallCost.filteringMemoryCost / (double)overallCost.overallMemoCost() << "%)" << std::endl;
    std::cout << '\t' << "Encoding memo cost: " << overallCost.encodingMemoryCost / 1000 << " kb"
              << " (" << 100 * (double)overallCost.encodingMemoryCost / (double)overallCost.overallMemoCost() << "%)" << std::endl;
    std::cout << '\t' << "Data memo cost: " << overallCost.encodedDataMemoryCost / 1000 << " kb"
              << " (" << 100 * (double)overallCost.encodedDataMemoryCost / (double)overallCost.overallMemoCost() << "%)" << std::endl;

    std::cout << "Overall compression results TIME: " << std::endl;
    std::cout << '\t' << "Adjustment time cost: " << overallCost.parameterAdjustmentTimeCost / 1000 << " ms"
              << " (" << 100 * (double)overallCost.parameterAdjustmentTimeCost / (double)overallCost.overallTimeCost() << "%)" << std::endl;
    std::cout << '\t' << "Partition time cost: " << overallCost.partitionTimeCost / 1000 << " ms"
              << " (" << 100 * (double)overallCost.partitionTimeCost / (double)overallCost.overallTimeCost() << "%)" << std::endl;
    std::cout << '\t' << "Prediction time cost: " << overallCost.blockMatchingTimeCost / 1000 << " ms"
              << " (" << 100 * (double)overallCost.blockMatchingTimeCost / (double)overallCost.overallTimeCost() << "%)" << std::endl;
    std::cout << '\t' << "Quantization time cost: " << overallCost.quantizationTimeCost / 1000 << " ms"
              << " (" << 100 * (double)overallCost.quantizationTimeCost / (double)overallCost.overallTimeCost() << "%)" << std::endl;
    std::cout << '\t' << "Filtering time cost: " << overallCost.filteringTimeCost / 1000 << " ms"
              << " (" << 100 * (double)overallCost.filteringTimeCost / (double)overallCost.overallTimeCost() << "%)" << std::endl;
    std::cout << '\t' << "Encoding time cost: " << overallCost.encodingTimeCost / 1000 << " ms"
              << " (" << 100 * (double)overallCost.encodingTimeCost / (double)overallCost.overallTimeCost() << "%)" << std::endl;

    std::cout << std::endl
              << "Memory: " << (double)overallCost.overallMemoCost() / (1000 * 1000) << "mb" << std::endl;
    std::cout << "Time: " << (double)overallCost.overallTimeCost() / (1000 * 1000) << "s" << std::endl;

    std::cout << std::endl
              << "Bit per pixel: " << 8.0 * ((double)overallCost.overallMemoCost() / (double)(lfModel.cols * lfModel.rows * someImage.size().area())) << std::endl;
    std::cout << "Compression rate: " << (double)(lfModel.cols * lfModel.rows * someImage.size().area() * someImage.channels()) / (double)overallCost.overallMemoCost() << std::endl;

    std::cout << std::endl
              << "Mean block count: " << ((double)overallCost.blockCount) / (lfModel.cols * lfModel.rows) << std::endl;
    std::cout
        << "Mean prediction error: " << ((double)overallCost.predictionError) / (lfModel.cols * lfModel.rows * someImage.size().area() * someImage.channels()) << std::endl;

    lfc::visualization::LightFieldMapPainter mapPainter;
    cv::Mat timeToShow = mapPainter.paint(timeCostMat, {0, 0, 255});
    cv::Mat bppToShow = mapPainter.paint(bppMat, {255, 0, 0});
    // cv::Mat perrorToShow = mapPainter.paint(predictionErrorMat, {255, 0, 255}, testResults.filteringThreshold);
    cv::Mat perrorToShow = mapPainter.paint(predictionErrorMat, {255, 0, 255});

    cv::imshow("Prediction Error", perrorToShow);
    cv::imshow("BPP", bppToShow);
    cv::imshow("Time", timeToShow);
    cv::waitKey(0);

    return 0;
}
