#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <assert.h>
#include <unistd.h>
#include <opencv.hpp>

using namespace cv;
using namespace std;

int main()
{
    //Initialise la webcam
    VideoCapture webcam(0);
    assert(webcam.isOpened());
    
    //Initialise la fenêtre graphique
    namedWindow("win");

    Mat img;
    while (true) {
        //Récupère la prochaine frame et l'affiche
        webcam >> img;
        assert(img.data != NULL);
        imshow("win", img);
        waitKey(20);
    }

    return EXIT_SUCCESS;
}

