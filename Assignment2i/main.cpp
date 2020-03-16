/*
Phil Culverhouse Oct 2016 (c) Plymouth University
James Rogers Jan 2020     (c) Plymouth University

This demo code will move eye and neck servos with kepresses.
When 'c' is pressed, one eye will track the target currently within the target window.

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

#define pi 3.14159
#define IPD 66
#define pwmDeg 10.730

using namespace std;
using namespace cv;

double distCalc(double Rx, double Lx);

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
    //integral offsets
    double Lxoffi=0;
    double Lyoffi=0;
    double Rxoffi=0;
    double Ryoffi=0;
    //differential offsets
    double Lxoffold=0;
    double Lyoffold=0;
    double Rxoffold=0;
    double Ryoffold=0;
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
    bool Tracking=false;

    while (!Tracking){
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

        //Draw a rectangle to define the tracking window, this is drawn onto a copy of the right eye as to leave a clean version of the right image
        Mat RightCopy;
        Right.copyTo(RightCopy);
        rectangle(RightCopy,target,Scalar(255,255,255),2);


        //Display left and right images
        imshow("Left",Left);
        imshow("Right", RightCopy);

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
        case 'e'://right
            Neck=Neck+5;
            break;
        case 'q'://left
            Neck=Neck-5;
            break;
        case 'c'://left
            Tracking=true;
            std::cout<<"tracking";
            //Rect target= Rect(320-32, 240-32, 64, 64); //defined in owl-cv.h
            OWLtempl=Right(target); //set the tracking template to whatever is within the tracking window in the right eye
            break;
        }

        //Send new motor positions to the owl servos
        CMDstream.str("");
        CMDstream.clear();
        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
        CMD = CMDstream.str();
        RxPacket= OwlSendPacket (u_sock, CMD.c_str());

    }

    //tracking loop
    while(1){

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

        //match template within right eye
        OwlCorrel OWLL;
        OwlCorrel OWLR;
        OWLL = Owl_matchTemplate(Left, OWLtempl);
        OWLR = Owl_matchTemplate(Right, OWLtempl);

        //Draw a rectangle to define the tracking window
        rectangle(Right, target, Scalar::all(255), 2, 8, 0 );
        rectangle(Left, target, Scalar::all(255), 2, 8, 0 );
        rectangle(Left, OWLL.Match, Point( OWLL.Match.x + OWLtempl.cols , OWLL.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
        rectangle(Right, OWLR.Match, Point( OWLR.Match.x + OWLtempl.cols , OWLR.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );

        //Display images
        imshow("Target",OWLtempl);
        imshow("Left", Left);
        imshow("Right", Right);
        imshow("Correl",OWLL.Result );
        waitKey(10);

        //Control the left eye to track the target
        //try altering KPx & KPy to see the settling time/overshoot
        double factor = 1.0;
        double KPx=0.05*factor; // track rate X
        double KPy=0.05*factor; // track rate Y
        double KIx = 0.001*factor;//integral gain x
        double KIy = 0.001*factor;//integral gain y
        double KDx = 0.01*factor;//differential gain x
        double KDy = 0.01*factor;//differential gain y

        //Update x-axis value based on target pos
        double LxScaleV = LxRangeV/static_cast<double>(640);            //Calculate number of pwm steps per pixel
        double Lxoff= (OWLL.Match.x + OWLtempl.cols/2 -320)/LxScaleV ;    //Compare to centre of image
        Lxoffi += Lxoff;
        Lx=static_cast<int>(Lx+Lxoff*KPx+Lxoffi*KIx-(Lxoff-Lxoffold)*KDx); //Update Servo position
        Lxoffold = Lxoff;

        //Update y-axis value based on target pos
        double LyScaleV = LyRangeV/static_cast<double>(480);            //Calculate number of pwm steps per pixel
        double Lyoff= ((OWLL.Match.y + OWLtempl.rows/2 - 240)/LyScaleV) ; //Compare to centre of image
        Lyoffi += Lyoff;
        Ly=static_cast<int>(Ly-Lyoff*KPy-Lyoffi*KIy+(Lyoff-Lyoffold)*KDy);                               //Update Servo position
        Lyoffold = Lyoff;

        //Update x-axis value based on target pos
        double RxScaleV = RxRangeV/static_cast<double>(640);            //Calculate number of pwm steps per pixel
        double Rxoff= (OWLR.Match.x + OWLtempl.cols/2 -320)/RxScaleV ;    //Compare to centre of image
        Rxoffi += Rxoff;
        Rx=static_cast<int>(Rx+Rxoff*KPx+Rxoffi*KIx-(Rxoff-Rxoffold)*KDx);                               //Update Servo position
        Rxoffold = Rxoff;

        //Update y-axis value based on target pos
        double RyScaleV = RyRangeV/static_cast<double>(480);            //Calculate number of pwm steps per pixel
        double Ryoff= ((OWLR.Match.y + OWLtempl.rows/2 - 240)/RyScaleV) ; //Compare to centre of image
        Ryoffi += Ryoff;
        Ry=static_cast<int>(Ry-Ryoff*KPy-Ryoffi*KIy+(Ryoff-Ryoffold)*KDy);
        Ryoffold = Ryoff;

        if (Rx>RxRm){
            Rx = RxRm;
           }
        if (Rx<RxLm){
            Rx = RxLm;
        }
        if(Lx>LxRm){
            Lx = LxRm;
        }
        if(Lx<LxLm){
            Lx = LxLm;
        }
        if (Ry>RyTm){
            Ry = RyTm;
           }
        if (Ry<RyBm){
            Ry = RyBm;
        }
        if(Ly>LyBm){
            Ly = LyBm;
        }
        if(Ly<LyTm){
            Ly = LyTm;
        }

        //Send new motor positions to the owl servos
        CMDstream.str("");
        CMDstream.clear();
        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
        CMD = CMDstream.str();
        RxPacket= OwlSendPacket (u_sock, CMD.c_str());

        double Dist = distCalc(Rx, Lx);
        std::cout<<"  Distance: "<<Dist<<"\n\r";
    }
}

double distCalc(double Rx, double Lx){

   //std::cout<<"Right Convergence "<<Rx<< " Left Convergence " << Lx<<"\n\r";
   double rEyeAngle=((double(Rx-RxC)*pi)/(pwmDeg*180))*-1; // in radians
   double lEyeAngle=(double(Lx-LxC)*pi)/(pwmDeg*180);
   //double rEyeAngle = ((Rx-RxC)/(RxC)*(pi+pi)-pi);
   //double lEyeAngle = (Lx-LxC)*(pi+pi)-pi;
   double lDist = (IPD*cos(rEyeAngle))/sin(lEyeAngle+rEyeAngle);
   //std::cout<<"left Eye: "<<lEyeAngle<<" right Eye: "<<rEyeAngle<< " left dist: "<<lDist;
   double Dist = sqrt(pow(lDist,2.0)+(pow((IPD/2),2.0)) - (lDist*IPD*sin(lEyeAngle)));
   return Dist;
}

















