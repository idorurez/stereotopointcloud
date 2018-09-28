/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   main.cpp
 * Author: Alfred Young
 *
 * Created on May 24, 2017, 10:44 PM
 */

#include <fstream>
#include <sstream>
#include <math.h>

#include <OpenEXR/ImfInputFile.h>
#include <OpenEXR/OpenEXRConfig.h>
#include <OpenEXR/ImfArray.h>
#include <OpenEXR/ImfRgbaFile.h>
#include <OpenEXR/ImfInputFile.h>
#include <OpenEXR/ImfRgba.h>
#include <OpenEXR/ImfThreading.h>
#include <OpenEXR/half.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/normal_space.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/registration/gicp.h>
#include <pcl/features/from_meshes.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

using namespace std;
using namespace Imf;
using namespace Imath;

void readRGBA(const char fileName[], Array2D <Rgba> &pixels, int &width, int &height);
float find_theta(int pt_x, int &width);
float find_phi(int pt_y, int &height);
half apply_sRGB(half color);
void cart_coords(float theta, float phi, half rho, float *coords);
void write_ply (const std::string &filename, const pcl::PCLPointCloud2 &cloud, bool binary, bool use_camera);
void register_point(pcl::PointXYZRGB &point, float* coords, Imf_2_2::Rgba colors);
void transform_cloud(float val, Eigen::Affine3f transform);
void fastBilateralFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);
void estimateNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normals);
void normalSpaceSampling(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_out);
void setOrganizedPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, int &width, int &height);
void setOrganizedPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int &width, int &height);
float estimate_depth(float val);

// command line arguments
bool binary = true;                             // use binary?
bool use_camera = false;                        // use camera
bool separate = false;                           // separate output into two separate point clouds
bool sparse = false;
bool rgbd = false;
bool straight = true;
string method = "straight";
float iod = .04;                                // iod default
float focal_length = .59;                        // 5.9 mm focal length of camera

void write_ply (const std::string &filename, const pcl::PCLPointCloud2 &cloud, bool binary, bool use_camera)
{
  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  pcl::PLYWriter writer;
  writer.write (filename, cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), binary, use_camera);

}

void readRGBA(const char fileName[],
                Array2D <Rgba> &pixels,
                int &width,
                int &height)
{
    RgbaInputFile file (fileName);
    Box2i dw = file.header().dataWindow();
    width  = dw.max.x - dw.min.x + 1;
    height = dw.max.y - dw.min.y + 1;
    pixels.resizeErase (height, width);

    file.setFrameBuffer (&pixels[0][0] - dw.min.x - dw.min.y * width, 1, width);
    file.readPixels (dw.min.y, dw.max.y);
}

float find_theta(int pt_x, int &width) {
    return (pt_x * 2 * M_PI) / width;
}

float find_phi(int pt_y, int &height) {
    return (pt_y * M_PI / height);
}

void cart_coords(float theta, float phi, half rho, float *coords) {
    
    if (rho > 0) {
        coords[0] = rho * cos(theta) * sin(phi);    // x
        coords[1] = rho * cos(phi);                 // y
        coords[2] = rho * sin(theta) * sin(phi);    // z
    } else {
        coords[0] = std::numeric_limits<float>::quiet_NaN();
        coords[1] = std::numeric_limits<float>::quiet_NaN();
        coords[2] = std::numeric_limits<float>::quiet_NaN();
    }
            
}

float estimate_depth(float val) {
//    cout << val << endl;
//    cout << (float) (1 / (val * (((float) 1/1000) - ((float) 1/50)) + ((float) 1/50))) << endl;
    return (float) (1 / (val * (((float) 1/1000) - ((float) 1/50)) + ((float) 1/50))); // estimation in cm
}

void register_point(pcl::PointXYZRGB &point, float *coords, Imf_2_2::Rgba colors) 
{     
    point.x = coords[0];
    point.y = coords[1];
    point.z = coords[2];
    point.r = (float) (apply_sRGB(colors.r) + 0.5);
    point.g = (float) (apply_sRGB(colors.g) + 0.5);
    point.b = (float) (apply_sRGB(colors.b) + 0.5);
}
 
 void transform_cloud(float val, Eigen::Affine3f transform) 
 {
  // Define a translation of val meters on the x axis.
  transform.translation() << val, 0.0, 0.0;
 }

void fastBilateralFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out) 
{
    FastBilateralFilter<PointXYZRGB> fbf;
    fbf.setInputCloud (cloud_in);
    fbf.setSigmaS (5);
    fbf.setSigmaR (0.005f);
    fbf.filter (*cloud_out);
}
 
void estimateNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output_cloud)
{
    // estimate normals
//    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
//    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
//    ne.setMaxDepthChangeFactor(0.0002f);
//    ne.setNormalSmoothingSize(.010f);
//    ne.setInputCloud(input_cloud);
//    ne.compute(*output_cloud);
    
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
    ne.setInputCloud (input_cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius
    ne.setRadiusSearch (0.005);

    // Compute the features
    ne.compute (*output_cloud);
}

void normalSpaceSampling(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_out)
{
    pcl::NormalSpaceSampling <PointXYZRGBNormal, PointXYZRGBNormal> nss;
    nss.setInputCloud (cloud_in);
    nss.setNormals (cloud_in);
    nss.setBins (4, 4, 4);
    nss.setSeed(0);
    nss.setSample (static_cast<unsigned int> (cloud_in->size ()) / 4);
    nss.setKeepOrganized (true);
    nss.filter (*cloud_out);
}

void setOrganizedPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int &width, int &height)
{
    cloud->width = width;
    cloud->height = height / 2;
    cloud->points.resize (cloud->width * cloud->height);
    
}

void setOrganizedPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, int &width, int &height)
{
    cloud->width = width;
    cloud->height = height / 2;
    cloud->points.resize (cloud->width * cloud->height);   
}

void reconstructMesh(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, PolygonMesh::Ptr mesh) 
{
    OrganizedFastMesh<pcl::PointXYZRGB> fast_mesh;
    fast_mesh.setInputCloud(input_cloud);
    fast_mesh.reconstruct(*mesh);
}

void printHelp (int, char **argv) {
    pcl::console::print_error ("Syntax is: \n");
    pcl::console::print_error ("    %s input_stereo.exr input_depthmap.exr output_left_eye.ply/combined.ply output_right.ply <options>\n", argv[0]);
    pcl::console::print_info ("Where options are:\n");
    pcl::console::print_info ("       -iod X = IOD of images (default: ");
    pcl::console::print_value ("%s", iod); pcl::console::print_info (")\n");
    pcl::console::print_info ("       -ascii = set output to ascii (default: ");
    pcl::console::print_value ("%s", binary ? "binary" : "ascii"); pcl::console::print_info (")\n");
    pcl::console::print_info ("       -use_camera 1/0 (default: ");
    pcl::console::print_value ("%s", use_camera ? "true" : "false"); pcl::console::print_info (")\n");
    pcl::console::print_info ("       -separate write out as separate or combined ply (default: ");
    pcl::console::print_value ("%s", separate ? "separate" : "combined"); pcl::console::print_info (")\n");
}

half apply_sRGB(half color) {
    half new_cc;
    if (color <= 0.0031308) {
        new_cc = color * 12.92;
    } else {
        new_cc = 1.055 * pow((double) color, 1.0 / 2.4) - 0.055;
    }
    new_cc *= 255;
    return new_cc;
}

int main(int argc, char * argv[])
{
    /**
     * Command line options: jpointcloud <latlong.exr> <depthmap.exr> <output.ply>
     */

    if (argc < 3)
    {
        printHelp (argc, argv);
        return (-1);
    }
    
    int latlong_width, latlong_height;    
    
    const char * latlong_file = argv[1];
    const char * depthmap_file = argv[2];
    const char * output_file_left = argv[3];
    const char * output_file_right = argv[4];
    const char * output_file_combined = argv[3];
    
    // moving eye points
    Eigen::Affine3f transform_left = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_right = Eigen::Affine3f::Identity();

    Eigen::Matrix4f initial_guess;
    initial_guess << 1, 0, 0, iod,
                     0, 1, 0, 0,
                     0, 0, 1, 0,
                     0, 0, 0, 1;

    int num_vertices;
    
    float phi, theta;                                               // spherical coordinates
    float rho_left, rho_right;                                      // spherical coordinates
    float coords_left[3], coords_right[3];                          // cartesian coordinates
    
    parse_argument (argc, argv, "-use_camera", use_camera);
    parse_argument (argc, argv, "-iod", iod);
    
    Array2D <Rgba> latlong_pixels, depthmap_pixels;
    
    // clouds and pointers oh my
    pcl::PCLPointCloud2 cloud2_left_ply, cloud2_right_ply;
    pcl::PCLPointCloud2::Ptr cloud2_combined (new pcl::PCLPointCloud2 ());
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp_no_normals;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_left (new pcl::PointCloud<pcl::PointXYZRGB>);    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_right (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_left_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_right_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_left_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_right_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_left_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_right_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_left_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_right_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_combined (new pcl::PointCloud<pcl::PointXYZRGBNormal>);


    PolygonMesh::Ptr cloud_left_mesh (new PolygonMesh);
    PolygonMesh::Ptr cloud_right_mesh (new PolygonMesh);
    
    // read and process entire image images
    readRGBA(latlong_file, latlong_pixels, latlong_width, latlong_height);
    readRGBA(depthmap_file, depthmap_pixels, latlong_width, latlong_height);

    num_vertices = latlong_width * latlong_height;
    
    // make sure the clouds are organized
    setOrganizedPointCloud(cloud_left, latlong_width, latlong_height);
    setOrganizedPointCloud(cloud_right, latlong_width, latlong_height);
    
    int mono_height = (latlong_height/2)-1;
        
    /* Parse command line flags */
//    
//    namespace po = boost::program_options;
//
//    // First of all, it is necessary to create an instance of po::option_description:
//
//    po::options_description description("MyTool Usage");
//
//    description.add_options()
//        ("help", "Display this help message")
//        ("version", "Display the version number")
//        ("ascii", "set ascii output")
//        ("separate", "skip registration and output separate pointclouds for eyes")
//        ("sparse", "perform sparse pointcloud registration method")
//        ("rgbd", "perform rgbd-based registration method");
//    

    
    // parse ascii flag
    if (parse_argument (argc, argv, "-ascii", binary) != -1) {
        binary = false;
    }
    
    // separate eyes?
    if (parse_argument (argc, argv, "-separate", separate) != -1) {
        separate = true;
        method = "separate";
    }
    
    if (parse_argument (argc, argv, "-sparse", sparse) != -1) {
        sparse = true;
        method = "sparse";
    }
    
    if (parse_argument (argc, argv, "-rgbd", rgbd) != -1) {
        rgbd = true;
        method = "rgbd";
    }
    
    if (parse_argument (argc, argv, "-straight", straight) != -1) {
        straight = true;
        method = "straight";
    }
    
    // Make sure we're working with mutually exclusive options
    if ((sparse && (rgbd || separate || straight)) || (rgbd && (separate || straight || sparse)) || 
    (separate && (rgbd || straight || sparse)) || (sparse && (rgbd || separate || straight))) {
        cout << "please select only -sparse, -rgbd, -separate, or -straight" << endl;
        exit(1);
    }
      
    
// set method
    
    // Print argument results
    print_info ("PLY output format: "); print_value ("%s, ", (binary ? "binary" : "ascii"));
    print_value ("%s, ", (separate ? "separate ply for each eye" : "combine both eyes for a ply"));
    
    if (method == "sparse") {
        print_value ("Using sparse registration method\n");
    } else if (method == "rgbd") {
        print_value ("using RGB-D registration method\n");
    } else {
        print_value ("performing straight-to-icp\n");
    }
    
            
            
    for (int row = 0; row < mono_height; row++) {
        for (int col = 0; col < latlong_width; col++) {
           
            pcl::PointXYZRGB p_left, p_right;
            int row_right = row+mono_height;           // "address" to the right eye's row
            float completed;
            
            // These spherical coordinates will always be the same for either eye
            theta = find_theta(col, latlong_width);
            phi = find_phi(row, mono_height);
            
            // find respective distances (rho) vals
            rho_left = estimate_depth(depthmap_pixels[row][col].r); 
            rho_right = estimate_depth(depthmap_pixels[row_right][col].r);
            
            // find cartesian coordinates for both eyes
            cart_coords(theta, phi, rho_left, coords_left);
            cart_coords(theta, phi, rho_right, coords_right);
            
            // store left eye info
            register_point(p_left, coords_left,  latlong_pixels[row][col]);           
            cloud_left->points[(row*latlong_width)+col] = p_left;
            
            // store right eye info
            register_point(p_right, coords_right, latlong_pixels[row_right][col]);
            cloud_right->points[(row*latlong_width)+col] = p_right;
            
            completed = (float) (((float) (row*latlong_width)+col) / (mono_height * latlong_width)) * 100;
            cout.precision(3);
            cout << "Writing points. %" << completed << " completed   " << std::flush << "\r";
        }
    }
    
    cout << "Transforming clouds based on iod\n";
    // translate clouds based on iod
    transform_cloud(iod/2 * -1, transform_left);
    transform_cloud(iod/2, transform_right);
    pcl::transformPointCloud (*cloud_left, *cloud_left_transformed, transform_left);            
    pcl::transformPointCloud (*cloud_right, *cloud_right_transformed, transform_right);            

    if (separate == true) {
        cout << "Attempting to convert and save separate point clouds for eyes\n";
        toPCLPointCloud2(*cloud_left_transformed, cloud2_left_ply);
        toPCLPointCloud2(*cloud_right_transformed, cloud2_right_ply);
        write_ply(output_file_left, cloud2_left_ply, binary, use_camera);
        write_ply(output_file_right, cloud2_right_ply, binary, use_camera);
        cout << "Done\n";
        
    } else if (method == "straight") { 
           const char * output_file_left_transformed = "cloud_left_transformed.ply";

           cout << "Writing temp intermediary cloud\n";
           toPCLPointCloud2(*cloud_left_transformed, cloud2_left_ply);
           write_ply(output_file_left_transformed, cloud2_left_ply, binary, use_camera);

           pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final_no_normals (new pcl::PointCloud<pcl::PointXYZRGB>);
           pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_combined_no_normals (new pcl::PointCloud<pcl::PointXYZRGB>);

           icp_no_normals.setInputSource (cloud_left_transformed);
           icp_no_normals.setInputTarget (cloud_right_transformed);
           icp_no_normals.setMaxCorrespondenceDistance (1);

           cout << "\nAligning left and right eye for straight registration\n";
           icp_no_normals.align(*cloud_final_no_normals, initial_guess);

           cout << "icp has converged:" << icp_no_normals.hasConverged() << " score: " <<
           icp_no_normals.getFitnessScore() << endl;
           cout << icp_no_normals.getFinalTransformation() << endl;

           *cloud_combined_no_normals = *cloud_left_transformed;
           *cloud_combined_no_normals += *cloud_final_no_normals;

           toPCLPointCloud2(*cloud_combined_no_normals, *cloud2_combined);

    } else {
    
        // 1) initial noise filter
        cout << "Applying bilateral filter\n";
        fastBilateralFilter (cloud_left_transformed, cloud_left_filtered);
        fastBilateralFilter (cloud_right_transformed, cloud_right_filtered);
        
        if (method == "sparse") { 
                
            cout << "Approximating mesh\n";
            // 2) Approximate mesh
            reconstructMesh(cloud_left_filtered, cloud_left_mesh);
            reconstructMesh(cloud_right_filtered, cloud_right_mesh);

            // 3) normal estimation
            cout << "Estimating normals and Covariances\n";
            boost::shared_ptr< std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > > left_covs (new std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >);
            pcl::features::computeApproximateNormals(*cloud_left_filtered, cloud_left_mesh->polygons, *cloud_left_normals);
            pcl::features::computeApproximateCovariances(*cloud_left_filtered, *cloud_left_normals, *left_covs);

            boost::shared_ptr< std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > > right_covs  (new std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >);
            pcl::features::computeApproximateNormals(*cloud_right_filtered, cloud_right_mesh->polygons, *cloud_right_normals);
            pcl::features::computeApproximateCovariances(*cloud_right_filtered, *cloud_right_normals, *right_covs);

            pcl::concatenateFields(*cloud_left_normals, *cloud_left_filtered, *cloud_left_with_normals);
            pcl::concatenateFields(*cloud_right_normals, *cloud_right_filtered, *cloud_right_with_normals);

            // write out current step
            const char * output_file_cloud_left_with_normals = "cloud_left_with_normals.ply";
            pcl::PCLPointCloud2 cloud2_left_with_normals;
            toPCLPointCloud2(*cloud_left_with_normals, cloud2_left_with_normals);
            cout << "Writing cloud left with normals\n";
            write_ply(output_file_cloud_left_with_normals, cloud2_left_with_normals, binary, use_camera);

            icp.setSourceCovariances(left_covs);
            icp.setTargetCovariances(right_covs);
            icp.setInputSource (cloud_left_with_normals);
            icp.setInputTarget (cloud_right_with_normals);

            cout << "Aligning left and right eye\n";
            icp.align(*cloud_final, initial_guess);

            cout << "icp has converged:" << icp.hasConverged() << " score: " <<
            icp.getFitnessScore() << endl;
            cout << icp.getFinalTransformation() << endl;
            
            cout << "Adjoining original and final clouds";
            *cloud_combined = *cloud_left_with_normals;
            *cloud_combined += *cloud_final;

            toPCLPointCloud2(*cloud_combined,*cloud2_combined);

        } else {
            
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_left_nss (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_right_nss (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

            // 2) normal estimation
            cout << "Estimating normals\n";
            estimateNormals (cloud_left_filtered, cloud_left_normals);
            estimateNormals (cloud_right_filtered, cloud_right_normals);

            pcl::concatenateFields(*cloud_left_normals, *cloud_left_filtered, *cloud_left_with_normals);
            pcl::concatenateFields(*cloud_right_normals, *cloud_right_filtered, *cloud_right_with_normals);

            // write out current step
            const char * output_file_cloud_left_with_normals = "cloud_left_with_normals.ply";
            pcl::PCLPointCloud2 cloud2_left_with_normals;
            toPCLPointCloud2(*cloud_left_with_normals, cloud2_left_with_normals);
            cout << "Writing cloud left with normals\n";
            write_ply(output_file_cloud_left_with_normals, cloud2_left_with_normals, binary, use_camera);

            // 3) downsampling with normal space sampling
            cout << "Normal space downsampling\n";
            normalSpaceSampling (cloud_left_with_normals, cloud_left_nss);
            normalSpaceSampling (cloud_right_with_normals, cloud_right_nss);

             // write out current step
            const char * output_file_cloud_nss = "cloud_left_nss.ply";
            pcl::PCLPointCloud2 cloud2_left_nss;
            toPCLPointCloud2(*cloud_left_nss, cloud2_left_nss);
            cout << "Writing cloud left with normals\n";
            write_ply(output_file_cloud_nss, cloud2_left_nss, binary, use_camera);

            // 4) Correspondence rejection w/median distance and surface normals. c-c-c-c-c-c-c-c-combo!
            pcl::registration::CorrespondenceRejectorMedianDistance::Ptr rej_md (new pcl::registration::CorrespondenceRejectorMedianDistance);
            pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rej_sn (new registration::CorrespondenceRejectorSurfaceNormal);
            rej_sn->setThreshold (0);
            rej_md->setMedianFactor (4.0);

            // add correspondence rejectors
            icp.addCorrespondenceRejector(rej_sn);
            icp.addCorrespondenceRejector(rej_md);      
            icp.setInputSource (cloud_left_with_normals);
            icp.setInputTarget (cloud_right_with_normals);

            cout << "Aligning left and right eye\n";
            icp.align(*cloud_final, initial_guess );

            cout << "icp has converged:" << icp.hasConverged() << " score: " <<
            icp.getFitnessScore() << endl;
            cout << icp.getFinalTransformation() << endl;

            
            cout << "Adjoining original and final clouds";

            *cloud_combined = *cloud_left_with_normals;
            *cloud_combined += *cloud_final;
            toPCLPointCloud2(*cloud_combined, *cloud2_combined);

        }
           
    }

    cout << "Downsampling points...";
    pcl::PCLPointCloud2::Ptr cloud_combined_filtered (new pcl::PCLPointCloud2 ());       
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    
    sor.setInputCloud (cloud2_combined);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_combined_filtered);
    
    cout << "Writing aligned, combined points\n";

//    pcl::io::savePCDFileBinary("combined.pcd", *cloud_combined);
    write_ply(output_file_combined, *cloud_combined_filtered, binary, use_camera);
    cout << "Done\n";

    return 0;

}
