#include "line.h"




Point2d commonPoint(line2 L1,line2 L2 )
{
    double Dx=(L1.x_.x-L2.x_.x)*(-L2.a_.y)+(L2.x_.y-L2.x_.y)*(L2.a_.x);
    double D=L1.a_.x*L2.a_.y-L1.a_.y*L2.a_.x;
    double t=Dx/D;

//     cout<<Dx<<' '<<D<<endl;

    return L1.x_+t*L1.a_;

}


line2 line3::project()
{

    return line2(Point2d(x_.x,x_.y), Point2d(a_.x,a_.y));

}


line3::line3(Mat Q,Mat T)
{

    Mat_<double> q;
    Q.convertTo(q, CV_64F);
    double t1=q(0,0);
    double x1=q(1,0);
    double y1=q(2,0);
    double z1=q(3,0);

    double angle=acosf(t1); //alpha/2
    double sin_angle=sinf(angle);
    //set vector
    a_.x=x1/sin_angle;
    a_.y=y1/sin_angle;
    a_.z=z1/sin_angle;

    //set point on line
    Mat_<double> t;

    T.convertTo(t, CV_64F);
    x_.x=t(0,0);
    x_.y=t(1,0);
    x_.z=t(2,0);


}
Point2d commonPoint(line3 L1,line3 L2 )
{
    return commonPoint(L1.project(),L2.project());

}
vector<Point2d> commonPoints(vector<line3> lines)
{
    vector<Point2d> points;
    for (size_t i=0;i<lines.size();i++)
    {
        for (size_t j=i+1;j<lines.size();j++)
        {
            Point2d a=commonPoint(lines[i],lines[j]);
            points.push_back(a);

        }
    }
    return points;

}


Point2d averagePoint(vector<Point2d> points )
{
    double sumX=0,sumY=0;
    size_t size=points.size();

    for (size_t i=0;i<size;i++)
    {
        sumX+=points[i].x;
        sumY+=points[i].y;

    }
    return Point2d(sumX/size,sumY/size);

}
Point3d averagePoint(vector<line3> lines)
{
    double sumX=0,sumY=0,sumZ=0;
    int size=0;
    Point3d x1,x2;
    for (size_t i=0;i<lines.size();i++)
    {
        for (size_t j=i+1;j<lines.size();j++)
        {
            nearestPoints(lines[i],lines[j],x1,x2);
            sumX+=x1.x+x2.x;
            sumX+=x1.y+x2.y;
            sumZ+=x1.z+x2.z;
            size++;

        }
    }
    size>>1;// size*2
    return Point3d(sumX/size,sumY/size,sumZ/size);

}

Point3d vecMull(Point3d a,Point3d b);
double distanse(line3 L1,line3 L2);

void nearestPoints(line3 L1,line3 L2,Point3d & P1,Point3d& P2)
{
    //L1: X= L1.a_*t1+L1.x_  L2: X = L2.a_*t2+L2.x_
    //L1.a_*t1 - L2.a_*t2=|d|*(L1.a_ * L2.a_)+L1.x_-L2.x_
    double d=distanse(L1,L2);
    Point3d a_b=d*vecMull(L1.a_,L2.a_);

    double dx=(L1.x_.x-L2.x_.x+a_b.x);
    double dy=(L1.x_.y-L2.x_.y+a_b.y);
    double D_t1=dx*(L2.a_.y)-dy*(L2.a_.x);
    double D_t2=(L1.a_.x)*dy-(L1.a_.y)*dx;
    double D=L1.a_.x*L2.a_.y-L1.a_.y*L2.a_.x;

    double t1=-D_t1/D;
    double t2=D_t2/D;

    P1=L1.x_+t1*L1.a_;
    P2=L2.x_+t2*L2.a_;
}

double det(Point3d a,Point3d b,Point3d c)
{
    //a_x a_y a_z
    //b_x b_y b_zs
    //c_x c_y c_z
    return std::fabs(a.x*b.y*c.z + b.x*c.y*a.z+a.y*c.x*b.z
             -c.x*b.y*a.z-b.x*a.y*c.z-a.x*c.y*b.z);
}
double distanse(line3 L1,line3 L2)
{
     return det(L1.x_-L2.x_,L1.a_,L2.a_);
}
double Norm(Point3d a)
{
    return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z);
}

Point3d vecMull(Point3d a,Point3d b)
{
    a=a*(1/Norm(a));
    b=b*(1/Norm(b));
    return Point3d(a.y*b.z-b.y*a.z, -(a.x*b.z-a.z*b.x) ,a.y*b.z-b.y*a.z);
}
