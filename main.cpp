#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //viewer->setCameraPose(2,-1,-3,0,3.14,3.14,0,0,0,0);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "Cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud");
    viewer->resetCameraViewpoint("Cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

boost::mutex updateModelMutex;
void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    while (!viewer->wasStopped ())
    {
        boost::mutex::scoped_lock updateLock(updateModelMutex);
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        updateLock.unlock();

    }
}


void createCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &point_cloud_ptr, float zmin,float zmax,float anglemin,float anglemax)
{

    // ------------------------------------
    // -----Create example point cloud-----
    // ------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    std::cout << "Genarating example point clouds.\n\n";
    // We're going to make an ellipse extruded along the z-axis. The colour for
    // the XYZRGB cloud will gradually go from red to green to blue.
    uint8_t r(255), g(15), b(15);
    for (float z=zmin; z <= zmax; z += 0.05)
    {
        for (float angle=anglemin; angle <= anglemax; angle += 5.0)
        {
            pcl::PointXYZ basic_point;
            basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
            basic_point.y = sinf (pcl::deg2rad(angle));
            basic_point.z = z;
            basic_cloud_ptr->points.push_back(basic_point);

            pcl::PointXYZRGBA point;
            point.x = basic_point.x;
            point.y = basic_point.y;
            point.z = basic_point.z;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                            static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            point_cloud_ptr->points.push_back (point);
        }
        if (z < 0.0)
        {
            r -= 12;
            g += 12;
        }
        else
        {
            g -= 12;
            b += 12;
        }
    }
    basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
    basic_cloud_ptr->height = 1;
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
    point_cloud_ptr->height = 1;
}

int
main (int argc, char** argv)
{

    cout<<"hello world"<<endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);

    // ------------------------------------
    // -----Create example point cloud-----
    // ------------------------------------
    createCloud(point_cloud_ptr,-1.0,1.0,0.0,360.0);

    // Creating PCL Viewer window and thread
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = rgbVis(point_cloud_ptr);
    //boost::thread vthread(&viewerRunner,viewer);
    boost::thread vthread(boost::bind(&viewerRunner,viewer));

    int	fcount =0;
    while (fcount<=3)
    {
        createCloud(point_cloud_ptr,-1.0-fcount,1.0+fcount,0.0,360.0);

        boost::mutex::scoped_lock updateLock(updateModelMutex);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(point_cloud_ptr);
        viewer->updatePointCloud<pcl::PointXYZRGBA>(point_cloud_ptr,rgb,"Cloud");
        updateLock.unlock();

        boost::this_thread::sleep (boost::posix_time::microseconds (200000));
        fcount++;
    }

    vthread.join();

}