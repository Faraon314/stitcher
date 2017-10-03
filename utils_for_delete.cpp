                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          #include <precomp.h>
    #include <iostream>
    #include <line.h>
    #include <camera.h>
    using namespace std;




    void printCameras(vector<stitching::Camera> cameras)
    {

        for (size_t i = 0; i < cameras.size(); ++i)
        {
            cout << "camera: "<<i<< endl;
            cout<< "focal: "<< cameras[0].focal<< endl;
            cout<< "aspect: "<< cameras[0].aspect<< endl;
            cout<< "ppx: "<< cameras[0].ppx<< endl;
            cout<< "ppy: "<< cameras[0].ppy<< endl;
            cout<< "cx: "<< cameras[0].cx<< endl;
            cout<< "cy: "<< cameras[0].cy<< endl;

            cout<<"intrinsics :"<<endl;
            cv::Mat_<float> K;
            cameras[i].K().convertTo(K, CV_32F);
            cout<<K(0,0)<< ' ';
            cout<<K(0,1)<< ' ';
            cout<<K(0,2)<< endl;
            cout<<K(1,0)<< ' ';
            cout<<K(1,1)<< ' ';
            cout<<K(1,2)<< endl;
            cout<<K(2,0)<< ' ';
            cout<<K(2,1)<< ' ';
            cout<<K(2,2)<< endl;

            cout<<"Rotation :"<<endl;
            cout<<cameras[i].R.at<double>(0,0)<< ' ';
            cout<<cameras[i].R.at<double>(0,1)<< ' ';
            cout<<cameras[i].R.at<double>(0,2)<< endl;
            cout<<cameras[i].R.at<double>(1,0)<< ' ';
            cout<<cameras[i].R.at<double>(1,1)<< ' ';
            cout<<cameras[i].R.at<double>(1,2)<< endl;
            cout<<cameras[i].R.at<double>(2,0)<< ' ';
            cout<<cameras[i].R.at<double>(2,1)<< ' ';
            cout<<cameras[i].R.at<double>(2,2)<< endl;
            cout<<"trasposition :"<<endl;
            cout<<cameras[i].t.at<double>(0,0)<< ' ';
            cout<<cameras[i].t.at<double>(1,0)<< ' ';
            cout<<cameras[i].t.at<double>(2,0)<< endl;
            cout<<endl;
        }
     }

    vector<stitching::Camera> retRoofCameras()
    {
        vector<stitching::Camera> cameras;
        stitching::Camera cam0;
        cam0.focal=2669.283856731386;
        cam0.aspect=1;
        cam0.ppx=2059.7633047373797;
        cam0.ppy=1610.3131060314458;
        cam0.cx=2974.9126600752825;
        cam0.cy=2974.9126600752825;


        cam0.Q.at<double>(0,0)=0.5014777682203287;
        cam0.Q.at<double>(1,0)=-0.49427617322611905;//x
        cam0.Q.at<double>(2,0)=0.49798566364342556;//y
        cam0.Q.at<double>(3,0)=0.506183159900958;//z
        cam0.calcR();


        cam0.t.at<double>(0,0)=-0.11052763183541797;
        cam0.t.at<double>(1,0)=-0.0000554373406533576;
        cam0.t.at<double>(2,0)= 0.00005837538903240044;
    //////////////////////////////////////////////////////
        stitching::Camera cam1;
        cam1.focal=2659.6785888188174;
        cam1.aspect=1;
        cam1.ppx=2141.6863623105787;
        cam1.ppy=1528.1366286402658;
        cam1.cx=2979.9918068387565;
        cam1.cy=2979.9918068387565;

        cam1.Q.at<double>(0,0)=0.6903821430849286;
        cam1.Q.at<double>(1,0)=-0.6727890956077942;//x
        cam1.Q.at<double>(2,0)=0.18579762648345802;//y
        cam1.Q.at<double>(3,0)=0.1902802442026522;//z
        cam1.calcR();

        cam1.t.at<double>(0,0)=-0.054653310110459014;
        cam1.t.at<double>(1,0)=-0.0958881395348833;
        cam1.t.at<double>(2,0)= 0.00010020768978155571;

    //////////////////////////////////////////////////////

        stitching::Camera cam2;
        cam2.focal=2675.747913779258;
        cam2.aspect=1;
        cam2.ppx=2100.596207513591;
        cam2.ppy=1575.0567156966117;
        cam2.cx=2970.0227509365614;
        cam2.cy=2970.0227509365614;

        cam2.Q.at<double>(0,0)=0.6856418764942244;
        cam2.Q.at<double>(1,0)=-0.6802500933430691;//x
        cam2.Q.at<double>(2,0)=-0.187414541882053;//y
        cam2.Q.at<double>(3,0)=-0.17897155415139282;//z
        cam2.calcR();

        cam2.t.at<double>(0,0)=0.05603465174249123;
        cam2.t.at<double>(1,0)=-0.09657082354589919;
        cam2.t.at<double>(2,0)= -0.0003361593307405558;
     /////////////////////////////////////////////


        stitching::Camera cam3;
        cam3.focal=2674.480841961668;
        cam3.aspect=1;
        cam3.ppx=2064.239790222484;
        cam3.ppy=1619.4392202591962;
        cam3.cx=2976.357334301288;
        cam3.cy=2976.357334301288;

        cam3.Q.at<double>(0,0)=0.49505410119646276;
        cam3.Q.at<double>(1,0)=-0.4942692678806468;//x
        cam3.Q.at<double>(2,0)=-0.5028718357561112 ;//y
        cam3.Q.at<double>(3,0)=-0.5076802581552396;//z
        cam3.calcR();

        cam3.t.at<double>(0,0)=0.11159259611349456;
        cam3.t.at<double>(1,0)=-1.4737518999238595e-12;
        cam3.t.at<double>(2,0)= 0.0004182874143630943;
     /////////////////////////////////////////////
        /////////////////////////////////////////////


           stitching::Camera cam4;
           cam4.focal=2743.007541618891;
           cam4.aspect=1;
           cam4.ppx=2167.3593286406845;
           cam4.ppy=1641.8108105203612;
           cam4.cx=2970.380475597225;
           cam4.cy=2970.380475597225;

           cam4.Q.at<double>(0,0)=0.1814783037631185;
           cam4.Q.at<double>(1,0)=-0.18311392701616283;//x
           cam4.Q.at<double>(2,0)=-0.6848932351149678;//y
           cam4.Q.at<double>(3,0)=-0.6815102137824013;//z
           cam4.calcR();

           cam4.t.at<double>(0,0)=0.05395962363701932;
           cam4.t.at<double>(1,0)=0.09509063928453929;
           cam4.t.at<double>(2,0)=-0.0002667921115136217;
        /////////////////////////////////////////////


              stitching::Camera cam5;
              cam5.focal=2604.58830773594;
              cam5.aspect=1;
              cam5.ppx=2031.3735835485174;
              cam5.ppy=1562.2300241421244;
              cam5.cx=2973.668570030094;
              cam5.cy=2973.668570030094;

              cam5.Q.at<double>(0,0)=0.19026407492587255;
              cam5.Q.at<double>(1,0)=-0.18576416386875558;//x
              cam5.Q.at<double>(2,0)=0.6717555700079412;//y
              cam5.Q.at<double>(3,0)=0.6914012665435679;//z
              cam5.calcR();

              cam5.t.at<double>(0,0)=-0.05814320748189296;
              cam5.t.at<double>(1,0)=0.0965537701528883;
              cam5.t.at<double>(2,0)=0.000026080949077117603;
           /////////////////////////////////////////////




        cameras.push_back(cam0);
        cameras.push_back(cam1);
        cameras.push_back(cam2);
        cameras.push_back(cam3);
        cameras.push_back(cam4);
        cameras.push_back(cam5);

         return cameras;
    }


    std::vector<cv::Mat > retRoofVImg()
    {
        std::vector<cv::Mat > vImg;
        vImg.push_back(cv::imread("/home/skutukov/clouds/Cloud Mail.Ru/stitcher/stitcher/res/roof_1_SPE0_0deg.jpg", CV_LOAD_IMAGE_COLOR));
        vImg.push_back(cv::imread("/home/skutukov/clouds/Cloud Mail.Ru/stitcher/stitcher/res/roof_1_SPE1_0deg.jpg", CV_LOAD_IMAGE_COLOR));
        vImg.push_back(cv::imread("/home/skutukov/clouds/Cloud Mail.Ru/stitcher/stitcher/res/roof_1_SPE2_0deg.jpg", CV_LOAD_IMAGE_COLOR));
        vImg.push_back(cv::imread("/home/skutukov/clouds/Cloud Mail.Ru/stitcher/stitcher/res/roof_1_SPE3_0deg.jpg", CV_LOAD_IMAGE_COLOR));
        vImg.push_back(cv::imread("/home/skutukov/clouds/Cloud Mail.Ru/stitcher/stitcher/res/roof_1_SPE4_0deg.jpg", CV_LOAD_IMAGE_COLOR));
        vImg.push_back(cv::imread("/home/skutukov/clouds/Cloud Mail.Ru/stitcher/stitcher/res/roof_1_SPE5_0deg.jpg", CV_LOAD_IMAGE_COLOR));
        return vImg;

    }

