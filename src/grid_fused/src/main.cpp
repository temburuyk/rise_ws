#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "bayesian.h"

#define GRIDROWS 25
#define GRIDCOLS 25

using namespace std;
using namespace cv;

void displayOccupancyGrid(vector<vector<bof::Cell> >& occGrid) {

    Mat mapImg = Mat::zeros(GRIDROWS, GRIDCOLS, CV_8UC3);

    for (int i = 0; i < GRIDROWS; ++i) {
        for (int j = 0; j < GRIDCOLS; ++j) {
            mapImg.at<Vec3b>(i, j)[0] = occGrid[i][j].getOccupiedProbability() * 255;
            mapImg.at<Vec3b>(i, j)[1] = occGrid[i][j].getOccupiedProbability() * 255;
            mapImg.at<Vec3b>(i, j)[2] = occGrid[i][j].getOccupiedProbability() * 255;
        }
    }

    //    while (waitKey(30) != 27) { // wait for ESC key press
    //        imshow("occupancyGrid", mapImg);
    //    }
    waitKey(1);
    imshow("occupancyGrid", mapImg);
}

int main(int argc, char **argv) {

    namedWindow("occupancyGrid", CV_WINDOW_NORMAL);

    bof::VelocityDistribution xVelDist(-6, 7, 2, 0);
    bof::VelocityDistribution yVelDist(-6, 7, 2, 0);

    yVelDist.setVelocityProbability(0, 0.1);
    yVelDist.setVelocityProbability(-2, 0.8);
    yVelDist.setVelocityProbability(-4, 0.1);

    xVelDist.setVelocityProbability(0, 0.1);
    xVelDist.setVelocityProbability(-2, 0.8);
    xVelDist.setVelocityProbability(-4, 0.1);

    /* Initialize occupancy grid */
    vector<vector<bof::Cell> > occGrid;
    for (int y = 0; y < GRIDROWS; ++y) {
        vector<bof::Cell> occRow;
        for (int x = 0; x < GRIDCOLS; ++x) {
            int occupancy = 0;
            if (x == GRIDCOLS - 1 && y == GRIDROWS - 1)
                occupancy = 1;
            bof::Cell cell(xVelDist, yVelDist, occupancy, x, y);
            occRow.push_back(cell);
        }
        occGrid.push_back(occRow);
    }

    for (int k = 0; k < GRIDROWS; ++k) {
        /* Update occupancy grid */
        vector<vector<bof::Cell> > prevOccGrid = occGrid;

        for (int i = 0; i < GRIDROWS; ++i) {
            for (int j = 0; j < GRIDCOLS; ++j) {
                occGrid[i][j].updateDistributions(prevOccGrid);
                // cout << "[" << i << "][" << j << "]: ";
                // occGrid[i][j].toString();
            }
        }

        displayOccupancyGrid(occGrid);
    }

    return 0;
}
