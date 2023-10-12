#ifndef LIGHTFIELD_MODEL_HPP
#define LIGHTFIELD_MODEL_HPP

#include <vector>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "lightfield_image.hpp"

namespace lfc
{
    class LightfieldModel
    {
    public:
        const int cols;
        const int rows;
    private:
        const std::string directoryPath;
        std::vector<std::string> imageFilenames;

        void fixTarot(std::string path, std::string outputPath, int originalX, int originalY)
        {
            int newX = 16 - originalX;
            std::string sy = originalY < 10 ? "0"+std::to_string(originalY) : std::to_string(originalY);
            std::string sx = newX < 10 ? "0"+std::to_string(newX) : std::to_string(newX);
        }

        void sortFilenames(bool leftRight, bool topBottom) {
            std::sort(imageFilenames.begin(), imageFilenames.end());

            if(leftRight && topBottom) return;

            std::vector<std::string> tempFilenames(imageFilenames.size(), "");
            for(int x=0; x<cols; ++x) {
                for(int y=0; y<rows; ++y) {
                    int newY = !topBottom ? (rows - y - 1) : y;
                    int newX = !leftRight ? (cols - x - 1) : x;
                    tempFilenames[newY * cols + newX] = imageFilenames[y * cols + x];
                }
            }

            imageFilenames = tempFilenames;
        }

    public:
        LightfieldModel(int cols, int rows, std::string directoryPath, bool leftRight = true, bool topBottom = true) 
        : cols(cols), rows(rows), directoryPath(directoryPath)
        {
            for (const auto &entry : std::filesystem::directory_iterator(directoryPath))
            {
                if(entry.path().extension() == ".png" || entry.path().extension() == ".JPG") {
                    imageFilenames.push_back(entry.path().string());
                }
            }
            
            sortFilenames(leftRight, topBottom);
        }

        std::string getImagePath(int col, int row) const
        {
            return imageFilenames[row * cols + col];
        }

        LightfieldImage getLightfieldImage(int col, int row) const {
            auto imagePath = getImagePath(col, row);
            cv::Mat image = cv::imread(imagePath);
            LightfieldImage lfImage{image, col, row};

            return lfImage;
        }
    };
}

#endif