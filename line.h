#ifndef LINE2_H
#define LINE2_H
#include <precomp.h>


using namespace  std;
using namespace cv;

class line2
{
 public:
 Point2d x_;
 Point2d a_;
 line2(Point2d x,Point2d a){x_=x; a_=a;}
 friend Point2d commonPoint(line2 L1,line2 L2);

};
Point2d commonPoint(line2 L1,line2 L2);

class line3
{
 public:
 Point3d x_;
 Point3d a_;
 line3(Point3d x,Point3d a){x_=x; a_=a;}
 // quaternion Q=(t,x,y,z) transposition t=(x,y,z)
 line3(Mat Q,Mat t);

 //project to z=0
 line2 project();
 //common poin on z=0;

 friend Point2d commonPoint(line3 L1,line3 L2 );
 Point3d nearestPoints(line3 L1,line3 L2,Point3d& x1,Point3d& x2 );
 friend vector<Point2d> commonPoints(vector<line3>);

 };

 void nearestPoints(line3 L1,line3 L2,Point3d& x1,Point3d& x2 );
 Point3d averagePoint(vector<line3>);

 vector<Point2d> commonPoints(vector<line3>);
 Point2d commonPoint(line3 L1,line3 L2 );
 Point2d averagePoint(vector<Point2d> points );

#endif // LINE2_H
