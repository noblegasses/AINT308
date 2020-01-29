/*
Phil Culverhouse Oct 2016 (c) Plymouth University
James Rogers Jan 2020     (c) Plymouth University

This demo code will move eye and neck servos with kepresses.
Use this code as a base for your assignment.

*/

#include <iostream>
#include <fstream>
#include <string>
#include <sys/types.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include "owl-pwm.h"
#include "owl-comms.h"
#include "owl-cv.h"

#define pi 3.14

using namespace std;
using namespace cv;

void scanEye(ostringstream &CMDstream, string CMD, SOCKET u_sock, string RxPacket);
void chameleon(ostringstream &CMDstream, string CMD, SOCKET u_sock, string RxPacket);
void crossEye(ostringstream &CMDstream, string CMD, SOCKET u_sock, string RxPacket);
void rollEye(ostringstream &CMDstream, string CMD, SOCKET u_sock, string RxPacket);

int main(int argc, char *argv[])
{
    //Setup TCP coms
    ostringstream CMDstream; // string packet
    string CMD;
    string PiADDR = "10.0.0.10";
    int PORT=12345;
    SOCKET u_sock = OwlCommsInit(PORT, PiADDR);

    //Set servo positions to their center-points
    Rx = RxC; Lx = LxC;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;

    // move servos to centre of field
    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    CMD = CMDstream.str();
    string RxPacket= OwlSendPacket (u_sock, CMD.c_str());

    Mat Frame, Left, Right;

    //Open video feed
    string source = "http://10.0.0.10:8080/stream/video.mjpeg";
    VideoCapture cap (source);
    if (!cap.isOpened())
    {
        cout  << "Could not open the input video: " << source << endl;
        return -1;
    }

    //main program loop
    while (1){
        if (!cap.read(Frame))
        {
            cout  << "Could not open the input video: " << source << endl;
            break;
        }

        //flip input image as it comes in reversed
        Mat FrameFlpd;
        flip(Frame,FrameFlpd,1);

        // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
        Left= FrameFlpd(Rect(0, 0, 640, 480)); // using a rectangle
        Right=FrameFlpd(Rect(640, 0, 640, 480)); // using a rectangle

        //Draw a circle in the middle of the left and right image (usefull for aligning both cameras)
        circle(Left,Point(Left.size().width/2,Left.size().height/2),10,Scalar(255,255,255),1);
        circle(Right,Point(Right.size().width/2,Right.size().height/2),10,Scalar(255,255,255),1);

        //Display left and right images
        imshow("Left",Left);
        imshow("Right", Right);

        //Read keypress and move the corresponding motor
        int key = waitKey(10);
        switch (key){
        case 'w': //up
            Ry=Ry+5;
            break;
        case 's'://down
            Ry=Ry-5;
            break;
        case 'a'://left
            Rx=Rx-5;
            break;
        case 'd'://right
            Rx=Rx+5;
            break;
        case 'i': //up
            Ly=Ly-5;
            break;
        case 'k'://down
            Ly=Ly+5;
            break;
        case 'j'://left
            Lx=Lx-5;
            break;
        case 'l'://right
            Lx=Lx+5;
            break;
        case '1'://right
            scanEye(CMDstream, CMD, u_sock, RxPacket);
            break;
        case '2'://right
            chameleon(CMDstream, CMD, u_sock, RxPacket);
            break;
        case '3'://right
            crossEye(CMDstream, CMD, u_sock, RxPacket);
            break;
        case '4'://right
            rollEye(CMDstream, CMD, u_sock, RxPacket);
            break;
//        case 'e'://right
//            Neck=Neck+5;
//            break;
//        case 'q'://left
//            Neck=Neck-5;
//            break;
        }
//        for(int i=0; i<=2*pi*50; i++){
//            float scaledI=(float)i/50.0f;
//            float rawSin = sin(scaledI);
//            float neckVal = (rawSin*425)+1525;
//            //float neckVal = (rawSin-(-1))*((1950-1100)/((-1)-1))+1100;
//            Neck = (int)neckVal;

//            cout << Neck;

//            //Send new motor positions to the owl servos
//            CMDstream.str("");
//            CMDstream.clear();
//            CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
//            CMD = CMDstream.str();
//            RxPacket= OwlSendPacket (u_sock, CMD.c_str());

//            Sleep(10);
//        }
    } // END cursor control loop

    // close windows down
    destroyAllWindows();


#ifdef _WIN32
    RxPacket= OwlSendPacket (u_sock, CMD.c_str());
    closesocket(u_sock);
#else
    OwlSendPacket (clientSock, CMD.c_str());
    close(clientSock);
#endif
    exit(0); // exit here for servo testing only
}


void scanEye(ostringstream &CMDstream, string CMD, SOCKET u_sock, string RxPacket){
    Ry = RyC;
    Ly = LyC;
    for(int i=0; i<=2*pi*50; i++){
        float scaledI=(float)i/50.0f;
        float rawSin = sin(scaledI);
        float rEyeVal = (rawSin*((RxRm-RxLm)/2))+((RxRm+RxLm)/2);
        float lEyeVal = (rawSin*((LxRm-LxLm)/2))+((LxRm+LxLm)/2);
        Rx = (int)rEyeVal;
        Lx = (int)lEyeVal;

        //Send new motor positions to the owl servos
        CMDstream.str("");
        CMDstream.clear();
        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
        CMD = CMDstream.str();
        RxPacket= OwlSendPacket (u_sock, CMD.c_str());

        Sleep(10);
    }
}

void chameleon(ostringstream &CMDstream, string CMD, SOCKET u_sock, string RxPacket){


    for(int i = 0; i<=3; i++){
        int randEye = rand() %3;
        unsigned long randTime = rand() %1000 + 50;
        if(randEye == 0){
            Rx = rand() %(RxRm-RxLm) + RxLm;
            Ry = rand() %(RyTm-RyBm) + RyBm;
            Lx = rand() %(LxRm-LxLm) + LxLm;
            Ly = rand() %(LyBm-LyTm) + LyTm;
        } else if(randEye == 1){
            Rx = rand() %(RxRm-RxLm) + RxLm;
            Ry = rand() %(RyTm-RyBm) + RyBm;
        } else if(randEye == 2){
            Lx = rand() %(LxRm-LxLm) + LxLm;
            Ly = rand() %(LyBm-LyTm) + LyTm;
        }

        //Send new motor positions to the owl servos
        CMDstream.str("");
        CMDstream.clear();
        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
        CMD = CMDstream.str();
        RxPacket= OwlSendPacket (u_sock, CMD.c_str());

        Sleep(randTime);
    }
}

void crossEye(ostringstream &CMDstream, string CMD, SOCKET u_sock, string RxPacket){
    Ry = RyC;
    Ly = LyC;
    for(int i=0; i<=25; i++){
        float rEyeVal = (i*(-(RxRm-RxLm)/2))+((RxRm+RxLm)/2);
        float lEyeVal = (i*((LxRm-LxLm)/2))+((LxRm+LxLm)/2);
        Rx = (int)rEyeVal;
        Lx = (int)lEyeVal;

        //Send new motor positions to the owl servos
        CMDstream.str("");
        CMDstream.clear();
        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
        CMD = CMDstream.str();
        RxPacket= OwlSendPacket (u_sock, CMD.c_str());

        Sleep(10);
    }
    Sleep(500);
    for(int i=25; i<=0; i--){
        float rEyeVal = (i*(-(RxRm-RxLm)/2))+((RxRm+RxLm)/2);
        float lEyeVal = (i*((LxRm-LxLm)/2))+((LxRm+LxLm)/2);
        Rx = (int)rEyeVal;
        Lx = (int)lEyeVal;

        //Send new motor positions to the owl servos
        CMDstream.str("");
        CMDstream.clear();
        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
        CMD = CMDstream.str();
        RxPacket= OwlSendPacket (u_sock, CMD.c_str());

        Sleep(10);
    }
}

void rollEye(ostringstream &CMDstream, string CMD, SOCKET u_sock, string RxPacket){
    Ry = RyC;
    Ly = LyC;
    for(int i=pi*50; i<=2*pi*50; i++){
        float scaledI=(float)i/50.0f;
        float rawSin = sin(scaledI);
        float rxEyeVal = (rawSin*((RxRm-RxLm)/2))+(RxRm);
        float lxEyeVal = (rawSin*((LxRm-LxLm)/2))+(LxRm);
        float ryEyeVal = (rawSin*(-(RyTm-RyBm)/2))+((RyTm+RyBm)/2);
        float lyEyeVal = (rawSin*((LyBm-LyTm)/2))+((LyTm+LyBm)/2);
        Rx = (int)rxEyeVal;
        Lx = (int)lxEyeVal;
        Ry = (int)ryEyeVal;
        Ly = (int)lyEyeVal;

        //Send new motor positions to the owl servos
        CMDstream.str("");
        CMDstream.clear();
        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
        CMD = CMDstream.str();
        RxPacket= OwlSendPacket (u_sock, CMD.c_str());

        Sleep(10);
    }
    Rx = RxC;
    Lx = LxC;
    Ry = RyC;
    Ly = LyC;

    //Send new motor positions to the owl servos
    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    CMD = CMDstream.str();
    RxPacket= OwlSendPacket (u_sock, CMD.c_str());
}
