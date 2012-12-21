#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <assert.h>
#include <unistd.h>
#include <opencv.hpp>
#include <GL/gl.h>
#include <GL/glu.h>
#include <SDL/SDL.h>

using namespace cv;
using namespace std;

int main()
{
    //Initialise la webcam
    VideoCapture webcam(0);
    assert(webcam.isOpened());
    
    //Initialise la fenêtre graphique
    namedWindow("win");

    //Initialisation de openGL et 
    //de ka fenêtre SDL
    int screenX = 400;
    int screenY = 300;
    SDL_Init(SDL_INIT_VIDEO);
    SDL_SetVideoMode(screenX, screenY, 32, SDL_OPENGL);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //gluOrtho2D(0, screenX, 0, screenY);
    gluPerspective(90, (double)screenX/screenY, 0.1, 1000);
    gluLookAt(-1, -10, 15, 0, 0, 0, 0, 0, 1);

    //Enregistrement d'images pour la calibration
    cout << "Select calibration images" << endl;
    vector<Mat> images;
    while (true) {
        //Récupère la prochaine frame et l'affiche
        Mat img;
        webcam >> img;
        assert(img.data != NULL);
        imshow("win", img);
        //Enregistre l'image si espace est pressé
        //Quite l'enregistrement si echap est pressé
        char key = waitKey(20);
        if (key == ' ') {
            images.push_back(img.clone());
            cout << "Image saved" << endl;
            waitKey(200);
        } else if (key == 27) {
            break;
        }
    }
    if (images.size() == 0) {
        cout << "No selected images" << endl;
        exit(0);
    }

    //Calibration variables
    //Nombre de coins internes du damier
    //horizontal et vertical
    size_t nbCornersX = 9;
    size_t nbCornersY = 6;
    Size sizeBoard = Size(nbCornersY, nbCornersX);

    //Conteneur
    //objPoints : positions 3D des coins du calibrateur
    //sur les images de calibration
    //imgPoints : position 2D sur les images de calibration
    //des coins du calibrateur
    vector< vector<Point3f> > objPoints;
    vector< vector<Point2f> > imgPoints;

    //Définition des coordonées des coins 3D
    //sur le calibrateur
    vector<Point3f> obj;
    for(size_t i=0;i<nbCornersX;i++) {
        for(size_t j=0;j<nbCornersY;j++) {
            Point3f pos(i, j, 0.0);
            obj.push_back(pos);
        }
    }

    //Détection des coins des images sélectionées
    for (size_t i=0;i<images.size();i++) {
        //Conteneurs des coins trouvés dans l'image
        vector<Point2f> corners;
        //Detection des coins
        bool founded = findChessboardCorners(images[i], sizeBoard, corners, 
            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_NORMALIZE_IMAGE);
        if (founded) {
            //Rafinement de la détection
            Mat img;
            cvtColor(images[i], img, CV_BGR2GRAY);
            cornerSubPix(img, corners, Size(11, 11), Size(-1, -1), 
                TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            //Enregistrement des associations de calibrations
            objPoints.push_back(obj);
            imgPoints.push_back(corners);
        }
    }
    assert(objPoints.size() == imgPoints.size());
    cout << "-> " << imgPoints.size() << " calibration associations" << endl;

    //Définition des matrices de la camera
    Mat intrinsicMatrix(3, 3, CV_64F);
    Mat distCoeffs(8, 1, CV_64F);
    vector<Mat> rvecs;
    vector<Mat> tvecs;

    //Calibration
    double rms = calibrateCamera(objPoints, imgPoints, images[0].size(), intrinsicMatrix, distCoeffs, rvecs, tvecs);
    cout << "=> Calibration error : " << rms << endl;

    //Précalcul de la carte des distorsions
    Mat distMap1;
    Mat distMap2;
    initUndistortRectifyMap(intrinsicMatrix, distCoeffs, Mat(), 
        getOptimalNewCameraMatrix(intrinsicMatrix, distCoeffs, images[0].size(), 1.0, images[0].size()), 
        images[0].size(), CV_16SC2, distMap1, distMap2);

    //Affichage live du rendu sans déformation
    /*
    while (true) {
        Mat img, tmp;
        webcam >> img;
        assert(img.data != NULL);
        remap(img, tmp, distMap1, distMap2, INTER_LINEAR);
        imshow("win", tmp);
        char key = waitKey(20);
        if (key == 27) {
            break;
        }
    }
    */

    int mouseClicked = false;
    double camAngleX = 0.0;
    double camAngleY = 0.0;
    while (true) {
        Mat img;
        webcam >> img;
        assert(img.data != NULL);
        vector<Point2f> corners;
        bool founded = findChessboardCorners(img, sizeBoard, corners, 
            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);
        if (founded) {
            //
            Mat tmp;
            cvtColor(img, tmp, CV_BGR2GRAY);
            cornerSubPix(tmp, corners, Size(11, 11), Size(-1, -1), 
                TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(img, sizeBoard, corners, founded);
            //
            Mat tmprvec;
            Mat tmptvec;
            solvePnP(obj, corners, intrinsicMatrix, distCoeffs, tmprvec, tmptvec);
            //
            //Controle de la camera
	    SDL_Event event;
	    while (SDL_PollEvent(&event)) {
                if (event.type == SDL_MOUSEMOTION && mouseClicked) {
                    camAngleX += 0.5*event.motion.xrel;
                    camAngleY += 0.5*event.motion.yrel;
                    glMatrixMode(GL_PROJECTION);
                    glLoadIdentity();
                    gluPerspective(90, (double)screenX/screenY, 0.1, 1000);
                    glRotated(camAngleY, 1, 0, 0);
                    gluLookAt(-1, -10, 15, 0, 0, 0, 0, 0, 1);
                    glRotated(camAngleX, 0, 0, 1);
                } else if (event.type == SDL_MOUSEBUTTONDOWN) {
                    mouseClicked = true;
                } else if (event.type == SDL_MOUSEBUTTONUP) {
                    mouseClicked = false;;
                }
            }
            //Initialisation du rendu d'une frame
            glClearColor(0.0, 0.0, 0.0, 0.0);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            //Dessin
            glBegin(GL_LINES);
                glColor3ub(255, 0, 0);
                glVertex3d(0, 0, 0);
                glVertex3d(1, 0, 0);
                glColor3ub(0, 255, 0);
                glVertex3d(0, 0, 0);
                glVertex3d(0, 1, 0);
                glColor3ub(0, 0, 255);
                glVertex3d(0, 0, 0);
                glVertex3d(0, 0, 1);
                for (size_t i=0;i<obj.size();i++) {
                    glColor3ub(255, 0, 0);
                    glVertex3d(obj[i].x, obj[i].y, obj[i].z);
                    glColor3ub(0, 0, 255);
                    glVertex3d(obj[i].x+1, obj[i].y, obj[i].z);
                    glColor3ub(255, 0, 0);
                    glVertex3d(obj[i].x, obj[i].y, obj[i].z);
                    glColor3ub(0, 0, 255);
                    glVertex3d(obj[i].x, obj[i].y+1, obj[i].z);
                }
            glEnd();
            //Reprojection
            Mat tmprot;
            Rodrigues(tmprvec, tmprot);
            Mat P(3, 4, CV_64F);
            P.at<double>(0,0) = tmprot.at<double>(0,0);
            P.at<double>(1,0) = tmprot.at<double>(1,0);
            P.at<double>(2,0) = tmprot.at<double>(2,0);
            P.at<double>(0,1) = tmprot.at<double>(0,1);
            P.at<double>(1,1) = tmprot.at<double>(1,1);
            P.at<double>(2,1) = tmprot.at<double>(2,1);
            P.at<double>(0,2) = tmprot.at<double>(0,2);
            P.at<double>(1,2) = tmprot.at<double>(1,2);
            P.at<double>(2,2) = tmprot.at<double>(2,2);
            P.at<double>(0,3) = tmptvec.at<double>(0,0);
            P.at<double>(1,3) = tmptvec.at<double>(1,0);
            P.at<double>(2,3) = tmptvec.at<double>(2,0);
            Mat KP = intrinsicMatrix*P;
            for (size_t i=0;i<obj.size();i++) {
                Mat X(4,1,CV_64F);
                X.at<double>(0,0) = obj[i].x;
                X.at<double>(1,0) = obj[i].y;
                X.at<double>(2,0) = obj[i].z;
                X.at<double>(3,0) = 1.0;
                Mat p = KP * X;
                Point2f pp;
                pp.x = p.at<double>(0,0) / p.at<double>(0,2);
                pp.y = p.at<double>(1,0) / p.at<double>(0,2);
                circle(img, pp, 10, Scalar(0,255,0));
            }
            //Traduction de la rotation openCV vers rotation openGL
            double rot[16];
            tmprot = tmprot.t();
            rot[0] = tmprot.at<double>(0,0);
            rot[1] = tmprot.at<double>(1,0);
            rot[2] = tmprot.at<double>(2,0);
            rot[3] = 0.0;
            rot[4] = tmprot.at<double>(0,1);
            rot[5] = tmprot.at<double>(1,1);
            rot[6] = tmprot.at<double>(2,1);
            rot[7] = 0.0;
            rot[8] = tmprot.at<double>(0,2);
            rot[9] = tmprot.at<double>(1,2);
            rot[10] = tmprot.at<double>(2,2);
            rot[11] = 0.0;
            rot[12] = tmptvec.at<double>(0,0);
            rot[13] = tmptvec.at<double>(1,0);
            rot[14] = tmptvec.at<double>(2,0);
            rot[15] = 1.0;
            glMultMatrixd(rot);
            glBegin(GL_LINES);
                glColor3ub(255, 0, 0);
                glVertex3d(0, 0, 0);
                glVertex3d(1, 0, 0);
                glColor3ub(0, 255, 0);
                glVertex3d(0, 0, 0);
                glVertex3d(0, 1, 0);
                glColor3ub(0, 0, 255);
                glVertex3d(0, 0, 0);
                glVertex3d(0, 0, 1);
            glEnd();
            //affichage du rendu
            glFlush();
            glFinish();
            SDL_GL_SwapBuffers();
        }
        imshow("win", img);
        char key = waitKey(20);
        if (key == 27) {
            break;
        }
    }

    //Libération des ressources
    webcam.release();
    destroyWindow("win");
    SDL_Quit();
    return EXIT_SUCCESS;
}

