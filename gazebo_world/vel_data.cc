#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>


// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

using namespace std;

#define PI 3.14159265
#define N 3

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

void multiplyVector(double mat1[][N], double mat2[N], double res[N]);
EulerAngles ToEulerAngles(Quaternion q);

string body_tags = ""; //all repeated tags are added in this global string variable
string points_coordinates = "";

int counter = 0;

void cb(ConstLaserScanStampedPtr &_msg)
{
    counter += 1;
    int i = 0;

    string xmlheader = "<?xml version=\"1.0\" enconding=\"utf-8\"?>\n<data>\n"; //xml tag header and data tag
    string xml_final_tag = "</data>"; //xml end data tag
    
    gazebo::msgs::LaserScan scan = _msg->scan();
    gazebo::msgs::Pose world_data = scan.world_pose();
    gazebo::msgs::Vector3d position = world_data.position();
    gazebo::msgs::Quaternion orientation = world_data.orientation();
    
    int ray_total_count = scan.count();

    string x_pos_tag = "\t\t\t\t<x>" + to_string(position.x()) + "</x>\n";
    string y_pos_tag = "\t\t\t\t<y>" + to_string(position.y()) + "</y>\n";
    string z_pos_tag = "\t\t\t\t<z>" + to_string(position.z()) + "</z>\n";

    string x_rot_tag = "\t\t\t\t<x>" + to_string(orientation.x()) + "</x>\n";
    string y_rot_tag = "\t\t\t\t<y>" + to_string(orientation.y()) + "</y>\n";
    string z_rot_tag = "\t\t\t\t<z>" + to_string(orientation.z()) + "</z>\n";
    string w_rot_tag = "\t\t\t\t<w>" + to_string(orientation.w()) + "</w>\n";

    string pos_tag = "\t\t\t<position>\n" + x_pos_tag + y_pos_tag + z_pos_tag + "\t\t\t</position>\n";
    string orientation_tag = "\t\t\t<orientation>\n" + x_rot_tag + y_rot_tag + z_rot_tag + w_rot_tag + "\t\t\t</orientation>\n";

    string world_pose_tag = "\t\t<world_pose>\n" + pos_tag + orientation_tag +"\t\t</world_pose>\n";

    string angle_min_tag = "\t\t<angle_min>" + to_string(scan.angle_min()) + "</angle_min>\n";
    string angle_max_tag = "\t\t<angle_max>" + to_string(scan.angle_max()) + "</angle_max>\n";
    string angle_step_tag = "\t\t<angle_step>" + to_string(scan.angle_step()) + "</angle_step>\n";
    string range_min_tag = "\t\t<range_min>" + to_string(scan.range_min()) + "</range_min>\n";
    string range_max_tag = "\t\t<range_max>" + to_string(scan.range_max()) + "</range_max>\n";
    string count_tag = "\t\t<count>" + to_string(ray_total_count) + "</count>\n";
    string vertical_angle_min_tag = "\t\t<vertical_angle_min>" + to_string(scan.vertical_angle_min()) + "</vertical_angle_min>\n";
    string vertical_angle_max_tag = "\t\t<vertical_angle_max>" + to_string(scan.vertical_angle_max()) + "</vertical_angle_max>\n";
    string vertical_angle_step_tag = "\t\t<vertical_angle_step>" + to_string(scan.vertical_angle_step()) + "</vertical_angle_step>\n";
    string vertical_count_tag = "\t\t<vertical_count>" + to_string(scan.vertical_count()) + "</vertical_count>\n";

    string angle_max_min_tags = angle_min_tag + angle_max_tag + angle_step_tag + range_min_tag + range_max_tag + count_tag + vertical_angle_min_tag + vertical_angle_max_tag + vertical_angle_step_tag + vertical_count_tag;

    string ranges_tags = "";
    string intensities_tags = "";


    double min_angle = scan.angle_min();
    double step_angle = scan.angle_step();

    double q0 = orientation.w();
    double q1 = orientation.x();
    double q2 = orientation.y();
    double q3 = orientation.z();

    double m11 = 2*((q0*q0) + (q1*q1))-1;
    double m12 = 2*((q1*q2) - (q0*q3));
    double m13 = 2*((q1*q3) + (q0*q2));
    double m21 = 2*((q1*q2) + (q0*q3));
    double m22 = 2*((q0*q0) + (q2*q2))-1;
    double m23 = 2*((q2*q3) - (q0*q1));
    double m31 = 2*((q1*q3) - (q0*q2));
    double m32 = 2*((q2*q3) + (q0*q1));
    double m33 = 2*((q0*q0) + (q3*q3))-1;
    

    // if(scan.angle_min() < 0)
    // {
    //     min_angle = (PI/2) + abs(scan.angle_min());
    // }
    // else
    // {
    //     min_angle = (PI/2) - scan.angle_min();
    // }

    double R[N][N] = { { m11, m12, m13 },
                       { m21, m22, m23 },
                       { m31, m32, m33 }};

    // double angle = 2*acos(orientation.w());
    // double vertical_angle = orientation.z()/sin(angle/2);
    
    // if(orientation.w() == 1.0)
    // {
    //     angle = 0;
    //     vertical_angle = 0;
    // }

    // cos_a  = W;
    // sin_a  = sqrt( 1.0 - cos_a * cos_a );
    // angle  = acos( cos_a ) * 2;
    // if ( fabs( sin_angle ) < 0.0005 ) sin_a = 1;
    // tx = X / sin_a;
    // ty = Y / sin_a;
    // tz = Z / sin_a;
    // latitude = -asin( ty );
    // if ( tx * tx + tz * tz < 0.0005 )
    //   longitude   = 0;
    // else
    //    longitude  = atan2( tx, tz );
    // if ( longitude < 0 )
    //   longitude += 360.0;
    double angle = 0;

    while(i<ray_total_count)
    {
        ranges_tags += "\t\t<ranges>" + to_string(scan.ranges(i)) + "</ranges>\n";
        intensities_tags += "\t\t<intensities>" + to_string(scan.intensities(i)) + "</intensities>\n";

        // double x1 = scan.ranges(i)*sin(min_angle)*cos(0);
        // double y1 = scan.ranges(i)*sin(min_angle)*sin(0);
        // double z1 = scan.ranges(i)*cos(min_angle);

        double x1 = scan.ranges(i)*cos(min_angle);
        double y1 = 0;
        double z1 = scan.ranges(i)*sin(min_angle);

        double v[N]={x1,y1,z1};

        double res[N];


        Quaternion qq;
        qq.w = orientation.w();
        qq.x = orientation.x();
        qq.y = orientation.y();
        qq.z = orientation.z();
        
        EulerAngles z_angle = ToEulerAngles(qq);

        double RZ[N][N] = { { cos(angle), -sin(angle), 0 },
                            { sin(angle), cos(angle), 0 },
                            { 0, 0, 1 }};

        multiplyVector(RZ, v, res);

        // double x = res[0] + position.x();
        // double y = res[1] + position.y();
        // double z = res[2] + position.z();

        double x = position.x();
        double y = position.y();
        double z = position.z();
        
        points_coordinates += to_string(x) + "\t" + to_string(y) + "\t" + to_string(z) + "\n";
        min_angle += step_angle;
        cout << "X: " + to_string(x1) + ";   Y: " + to_string(y1) + ";   Z: " + to_string(z1) + "; \n";

        i += 1;
    }
    
    angle += step_angle;

    string point_tag = "\t<point>\n" + world_pose_tag + angle_max_min_tags + ranges_tags + intensities_tags + "\t</point>\n";
    
    body_tags += point_tag;

    ofstream file;
    // file.open("../sensor_data/sensor_data.xml", fstream::app);
    file.open("../sensor_data/sensor_data.xml"); //open is the method of ofstream
    file << xmlheader + body_tags + xml_final_tag;
    file.close();

    ofstream pointCloud;
    // pointCloud.open("../sensor_data/sensor_data.xml", fstream::app);
    pointCloud.open("../sensor_data/sensor_data.xyz"); //open is the method of ofstream
    pointCloud << points_coordinates;
    pointCloud.close();

    Quaternion q;
    q.w = orientation.w();
    q.x = orientation.x();
    q.y = orientation.y();
    q.z = orientation.z();
    EulerAngles angles = ToEulerAngles(q);

    // cout << "Roll: " + to_string(angles.roll) + "; Pitch: " + to_string(angles.pitch) + "; Yaw: " + to_string(angles.yaw) + "; \n";

}

// Multiplication of matrices
// void multiply(double mat1[][N], double mat2[N][], double res[N][])
// {
//     int i, j, k;
//     for (i = 0; i < N; i++) {
//         for (j = 0; j < N; j++) {
//             res[i][j] = 0;
//             for (k = 0; k < N; k++)
//                 res[i][j] += mat1[i][k] * mat2[k][j];
//         }
//     }
// }

void multiplyVector(double mat1[][N], double mat2[N], double res[N])
{
    int i, j;
    for (int i=0;i<N;i++){
        for (int j=0;j<N;j++){
            res[i]+=(mat1[i][j]*mat2[j]);
        }
    }
}

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{

    // if(!isdigit(to_string(atoi(_argv[0]))))
    // {
    //     cout << "\nPlease enter a valid number as an argument for max_counter variable!!\n\n";
    //     return 0;
    // }

    // int max_counter = atoi(_argv[0]);

    // for (int i = 0; i < _argc; ++i) {
    //   cout << _argv[i] << endl;
    // }

    //int max_counter = 200;
    
    // Load gazebo as a client
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(_argc, _argv);
    #else
    gazebo::client::setup(_argc, _argv);
    #endif

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    
    // Publish to the  velodyne topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/my_velodyne/velodyne_hdl-32/top/sensor/scan", cb);

    while (true)
        gazebo::common::Time::MSleep(10);

    // Make sure to shut everything down.
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
    #else
    gazebo::client::shutdown();
    #endif
}