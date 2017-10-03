
#include <stitcher.h>
#include <line.h>
#include <warpers.h>

#include <opencv2/highgui/highgui.hpp>
#include <utils_for_delete.cpp>

#include <iomanip>
#include <ctime>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>

using namespace  std;

void stitch();

int main(int argc, char *argv[])
{
//   stitch();

    stitching::SphericalWarperCreator warpCreate;
    stitching::Camera cam;
    //stitching::SphericalWarper w =warpCreate.create(1,cv::Point3d(0,0,0),cam.K(),cam.R,cam.t);


    cam.Q.at<double>(0,0)=sqrt(0.25);
    cam.Q.at<double>(1,0)=sqrt(0.25);//x
    cam.Q.at<double>(2,0)=0;//y
    cam.Q.at<double>(3,0)=0;//z
    cam.calcR();

    cout<<cam.R.at<double>(0,0)<< ' ';
    cout<<cam.R.at<double>(0,1)<< ' ';
    cout<<cam.R.at<double>(0,2)<< endl;
    cout<<cam.R.at<double>(1,0)<< ' ';
    cout<<cam.R.at<double>(1,1)<< ' ';
    cout<<cam.R.at<double>(1,2)<< endl;
    cout<<cam.R.at<double>(2,0)<< ' ';
    cout<<cam.R.at<double>(2,1)<< ' ';
    cout<<cam.R.at<double>(2,2)<< endl;


    cam.Q.at<double>(0,0)=sqrt(0.25);
    cam.Q.at<double>(1,0)=-sqrt(0.25);//x
    cam.Q.at<double>(2,0)=0;//y
    cam.Q.at<double>(3,0)=0;//z
    cam.calcR();

    cout<<cam.R.at<double>(0,0)<< ' ';
    cout<<cam.R.at<double>(0,1)<< ' ';
    cout<<cam.R.at<double>(0,2)<< endl;
    cout<<cam.R.at<double>(1,0)<< ' ';
    cout<<cam.R.at<double>(1,1)<< ' ';
    cout<<cam.R.at<double>(1,2)<< endl;
    cout<<cam.R.at<double>(2,0)<< ' ';
    cout<<cam.R.at<double>(2,1)<< ' ';
    cout<<cam.R.at<double>(2,2)<< endl;




   return 0;
}

void stitch()
{

    string file="/home/skutukov/clouds/Cloud Mail.Ru/stitcher/stitcher/result/res.jpg";
    ofstream fout;
    fout.open("/home/skutukov/clouds/Cloud Mail.Ru/stitcher/stitcher/result/log.txt",ios::app);

    cv::Mat result;
    stitching::Stitcher stitcher = stitching::Stitcher::createDefault(stitching::Blender::NO);


   //prepare;
    stitcher.setCameras(retRoofCameras());
    printCameras(stitcher.cameras());
    stitcher.estimateWarpedImageScale();
    stitcher.estimateCenter();
    stitcher.calcWarps();

    //printCameras(stitcher.cameras());

    std::clock_t c_start = std::clock();

    stitching::Stitcher::Status status = stitcher.composePanorama(retRoofVImg(),result);

    std::clock_t c_end = std::clock();

    ulong clock=c_end-c_start;
    std::cout << "clocks: " << clock << std::endl;


    if(stitching::Stitcher::Status::OK == status)
    {
//        cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
//        cv::imshow( "Display window", result );
       cv::imwrite( file, result );

    }else{
       std::cout<<"error"<<std::endl;
    }

 //   fout<<file<<" displacedSphericalWarper NOBand "<<" clocks of compose pano:" <<clock<<endl;

    cv::waitKey(0);
     fout.close();

}
