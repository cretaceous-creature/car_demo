#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
//#include <pcl/filters/uniform_sampling.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <boost/foreach.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_reconfigure/server.h>
#include <templatematching/matchingparamConfig.h>
#include <geometry_msgs/TransformStamped.h>


typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

void showHelp (char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                     Show this help." << std::endl;
  std::cout << "     -k:                     Show used keypoints." << std::endl;
  std::cout << "     -c:                     Show used correspondences." << std::endl;
  std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
  std::cout << "                             each radius given by that value." << std::endl;
  std::cout << "     --algorithm (Hough|GC): Clustering algorithm used (default Hough)." << std::endl;
  std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
  std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
  std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
  std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
  std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
  std::cout << "     --cg_thresh val:        Clustering threshold (default 5)" << std::endl << std::endl;
}

void parseCommandLine (int argc, char *argv[]) {
  //Show help
  if (pcl::console::find_switch (argc, argv, "-h"))
  {
    showHelp (argv[0]);
    exit (0);
  }

  //Model & scene filenames
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 2)
  {
    std::cout << "Filenames missing.\n";
    showHelp (argv[0]);
    exit (-1);
  }


  //Program behavior
//  if (pcl::console::find_switch (argc, argv, "-k"))
//  {
//    show_keypoints_ = true;
//  }
//  if (pcl::console::find_switch (argc, argv, "-c"))
//  {
//    show_correspondences_ = true;
//  }
//  if (pcl::console::find_switch (argc, argv, "-r"))
//  {
//    use_cloud_resolution_ = true;
//  }

//  std::string used_algorithm;
//  if (pcl::console::parse_argument (argc, argv, "--algorithm", used_algorithm) != -1)
//  {
//    if (used_algorithm.compare ("Hough") == 0)
//    {
//      use_hough_ = true;
//    }else if (used_algorithm.compare ("GC") == 0)
//    {
//      use_hough_ = false;
//    }
//    else
//    {
//      std::cout << "Wrong algorithm name.\n";
//      showHelp (argv[0]);
//      exit (-1);
//    }
//  }

  //General parameters
//  pcl::console::parse_argument (argc, argv, "--model_ss", model_ss_);
//  pcl::console::parse_argument (argc, argv, "--scene_ss", scene_ss_);
//  pcl::console::parse_argument (argc, argv, "--rf_rad", rf_rad_);
//  pcl::console::parse_argument (argc, argv, "--descr_rad", descr_rad_);
//  pcl::console::parse_argument (argc, argv, "--cg_size", cg_size_);
//  pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);
}



class TemplateMatching {
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, jsk_recognition_msgs::BoundingBox> syncPolicy;
  public:
    //Algorithm params
    bool show_keypoints_ ;
    bool show_correspondences_;
    bool use_cloud_resolution_;
    bool use_hough_;
    float model_ss_;
    float scene_ss_;
    float rf_rad_;
    float descr_rad_;
    float cg_size_;
    float cg_thresh_;
    std::string model_filename_;

    pcl::PointCloud<PointType>::Ptr model;
    pcl::PointCloud<PointType>::Ptr model_keypoints;
    pcl::PointCloud<PointType>::Ptr scene;
    pcl::PointCloud<PointType>::Ptr scene_keypoints;
    pcl::PointCloud<NormalType>::Ptr model_normals;
    pcl::PointCloud<NormalType>::Ptr scene_normals;
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors;
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors;

    TemplateMatching(ros::NodeHandle& n, ros::NodeHandle& np)
        : n_(n),
          np_(np),
          show_keypoints_(false),
          show_correspondences_(false),
          use_cloud_resolution_(false),
          use_hough_(false),
          model_ss_(0.01f),
          scene_ss_(0.01f),
          rf_rad_(0.015f),
          descr_rad_(0.01f),
          cg_size_(0.01f),
          cg_thresh_(3.0f),
          model_filename_("") {

        ros::param::set("/model_filename","/home/chen/ros/kinetic/src/car_demo/templatematching/data/1.pcd");
        ros::param::set("use_cloud_resolution", false);
        ros::param::set("use_hough", false);
        ros::param::set("model_ss",0.01f);
        ros::param::set("scene_ss",  0.01f);
        ros::param::set("rf_rad", 0.015f);
        ros::param::set("descr_rad",  0.01f);
        ros::param::set("cg_size", 0.01f);
        ros::param::set("cg_thresh", 3.0f);

        ros::param::param<bool>("use_cloud_resolution", use_cloud_resolution_, false);
        ros::param::param<bool>("use_hough", use_hough_, false);
        ros::param::param<float>("model_ss", model_ss_, 0.01f);
        ros::param::param<float>("scene_ss", scene_ss_, 0.01f);
        ros::param::param<float>("rf_rad", rf_rad_, 0.015f);
        ros::param::param<float>("descr_rad", descr_rad_, 0.01f);
        ros::param::param<float>("cg_size", cg_size_, 0.01f);
        ros::param::param<float>("cg_thresh", cg_thresh_, 3.0f);
        ros::param::param<std::string>("/model_filename", model_filename_, "/home/chen/ros/kinetic/src/car_demo/templatematching/data/1.pcd");


        transformInitialized = false;

        /*** dynamic reconfigure ***/
        f = boost::bind(&TemplateMatching::dynamic_reconfigure_callback, this, _1,_2);
        server.setCallback(f);


        model = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType> ());
        model_keypoints = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType> ());
        scene = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType> ());
        scene_keypoints = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType> ());
        model_normals = pcl::PointCloud<NormalType>::Ptr(new pcl::PointCloud<NormalType> ());
        scene_normals = pcl::PointCloud<NormalType>::Ptr(new pcl::PointCloud<NormalType> ());
        model_descriptors = pcl::PointCloud<DescriptorType>::Ptr(new pcl::PointCloud<DescriptorType> ());
        scene_descriptors = pcl::PointCloud<DescriptorType>::Ptr(new pcl::PointCloud<DescriptorType> ());

        pcl::io::loadPCDFile(model_filename_, *model);

        pub_filtered = n_.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 1);
        pub_model = n_.advertise<sensor_msgs::PointCloud2>("/cloud_model", 1);
        pub_trans = n_.advertise<geometry_msgs::TransformStamped>("/matching_trans", 1);
        sub_pointcloud = new message_filters::Subscriber<sensor_msgs::PointCloud2>(n_, "/realsense/camera/depth_registered/points", 2);
        sub_boundingbox = new message_filters::Subscriber<jsk_recognition_msgs::BoundingBox>(n_, "/charge_detector/boundingbox", 2);

        sync = new message_filters::Synchronizer<syncPolicy>(syncPolicy(200), *sub_pointcloud, *sub_boundingbox);
        sync->registerCallback(boost::bind(&TemplateMatching::CloudCallBack, this, _1, _2));
    }

    void output(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations,
                std::vector<pcl::Correspondences> clustered_corrs){
        //
        //  Output results
        //
        // std::cout << "Model instances found: " << rototranslations.size () << std::endl;
        for (size_t i = 0; i < rototranslations.size (); ++i) {

          // Print the rotation matrix and translation vector
          Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
          Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);
          if (rotation(0,0) > 0.7 && rotation(1,1) > 0.85 && rotation(2,2) > 0.85 && rotation(0,0) != 1) {
              printf ("\n");
              std::cout << "      Instance " << i + 1 << ":" << std::endl;
              std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;
              printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
              printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
              printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
              printf ("\n");
              printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
          }
        }
    }

    double computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud) {
      double res = 0.0;
      int n_points = 0;
      int nres;
      std::vector<int> indices (2);
      std::vector<float> sqr_distances (2);
      pcl::search::KdTree<PointType> tree;
      tree.setInputCloud (cloud);

      for (size_t i = 0; i < cloud->size (); ++i) {
        if (! pcl_isfinite ((*cloud)[i].x)) {
          continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2) {
          res += sqrt (sqr_distances[1]);
          ++n_points;
        }
      }
      if (n_points != 0) {
        res /= n_points;
      }
      return res;
    }

    pcl::PointCloud<PointType> pointcloudfilter(const pcl::PointCloud<PointType>& pointcloud,
                          const jsk_recognition_msgs::BoundingBoxConstPtr& msg_box) {
        pcl::PointCloud<PointType> ret;
        std::cout << pointcloud.width << ' ' << pointcloud.height <<std::endl;
        for (int i = msg_box->pose.position.x; i < msg_box->pose.position.x + msg_box->dimensions.x; i++) {
            for (int j = msg_box->pose.position.y; j < msg_box->pose.position.y + msg_box->dimensions.y; j++) {
                ret.push_back(pointcloud.at(i, j));
            }
        }
        return ret;
    }


    void dynamic_reconfigure_callback(templatematching::matchingparamConfig &config, uint32_t level)
    {
        use_cloud_resolution_ = config.use_cloud_reso;
        use_hough_ = config.use_hough;
        model_ss_ = config.model_ss;
        scene_ss_ = config.scene_ss;
        rf_rad_ = config.rf_rad;
        descr_rad_ = config.descr_rad;
        cg_size_ = config.cg_size;
        cg_thresh_ = config.cg_thresh;
    }



    void CloudCallBack(const sensor_msgs::PointCloud2ConstPtr& msg_pt,
                       const jsk_recognition_msgs::BoundingBoxConstPtr& msg_box) {
        /*
        ros::param::get("use_cloud_resolution", use_cloud_resolution_);
        ros::param::get("use_hough", use_hough_);
        ros::param::get("model_ss", model_ss_);
        ros::param::get("scene_ss", scene_ss_);
        ros::param::get("rf_rad", rf_rad_);
        ros::param::get("descr_rad", descr_rad_);
        ros::param::get("cg_size", cg_size_);
        ros::param::get("cg_thresh", cg_thresh_);
*/
        std::cout << "Received " << use_hough_ << std::endl;
        pcl::PointCloud<PointType> scene_tmp;
        pcl::fromROSMsg(*msg_pt, scene_tmp);
        *scene = pointcloudfilter(scene_tmp, msg_box);
        sensor_msgs::PointCloud2 cloud2pub;
        sensor_msgs::PointCloud2 cloudmodel2pub;
        pcl::toROSMsg(*scene, cloud2pub);
        cloud2pub.header = msg_pt->header;
        pub_filtered.publish(cloud2pub);
        //publish model cloud too
        pcl::toROSMsg(*model, cloudmodel2pub);
        cloudmodel2pub.header = msg_pt->header;
        pub_model.publish(cloudmodel2pub);
        //
        //  Set up resolution invariance
        //
        if (use_cloud_resolution_) {
          float resolution = static_cast<float> (computeCloudResolution (model));
          if (resolution != 0.0f) {
            model_ss_   *= resolution;
            scene_ss_   *= resolution;
            rf_rad_     *= resolution;
            descr_rad_  *= resolution;
            cg_size_    *= resolution;
          }

          std::cout << "Model resolution:       " << resolution << std::endl;
          std::cout << "Model sampling size:    " << model_ss_ << std::endl;
          std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
          std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
          std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
          std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
        }

        //
        //  Compute Normals
        //
        pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
        norm_est.setKSearch (10);
        norm_est.setInputCloud (model);
        norm_est.compute (*model_normals);

        norm_est.setInputCloud (scene);
        norm_est.compute (*scene_normals);

        pcl::UniformSampling<PointType> uniform_sampling;
        uniform_sampling.setInputCloud (model);
        uniform_sampling.setRadiusSearch (model_ss_);
        //uniform_sampling.filter (*model_keypoints);
        pcl::PointCloud<int> keypointIndices1;
        uniform_sampling.compute(keypointIndices1);
        pcl::copyPointCloud(*model, keypointIndices1.points, *model_keypoints);
        std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;


        uniform_sampling.setInputCloud (scene);
        uniform_sampling.setRadiusSearch (scene_ss_);
        //uniform_sampling.filter (*scene_keypoints);
        pcl::PointCloud<int> keypointIndices2;
        uniform_sampling.compute(keypointIndices2);
        pcl::copyPointCloud(*scene, keypointIndices2.points, *scene_keypoints);
        std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;

        //
        //  Compute Descriptor for keypoints
        //
        pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
        descr_est.setRadiusSearch (descr_rad_);

        descr_est.setInputCloud (model_keypoints);
        descr_est.setInputNormals (model_normals);
        descr_est.setSearchSurface (model);
        descr_est.compute (*model_descriptors);

        descr_est.setInputCloud (scene_keypoints);
        descr_est.setInputNormals (scene_normals);
        descr_est.setSearchSurface (scene);
        descr_est.compute (*scene_descriptors);

        //
        //  Find Model-Scene Correspondences with KdTree
        //
        pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

        pcl::KdTreeFLANN<DescriptorType> match_search;
        match_search.setInputCloud (model_descriptors);

        //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
        for (size_t i = 0; i < scene_descriptors->size (); ++i)
        {
          std::vector<int> neigh_indices (1);
          std::vector<float> neigh_sqr_dists (1);
          if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
          {
            continue;
          }
          int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
          if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
          {
            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back (corr);
          }
        }
        std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

        //
        //  Actual Clustering
        //
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
        std::vector<pcl::Correspondences> clustered_corrs;

        //  Using Hough3D
        if (use_hough_) {
          //
          //  Compute (Keypoints) Reference Frames only for Hough
          //
          pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
          pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

          pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
          rf_est.setFindHoles (true);
          rf_est.setRadiusSearch (rf_rad_);

          rf_est.setInputCloud (model_keypoints);
          rf_est.setInputNormals (model_normals);
          rf_est.setSearchSurface (model);
          rf_est.compute (*model_rf);

          rf_est.setInputCloud (scene_keypoints);
          rf_est.setInputNormals (scene_normals);
          rf_est.setSearchSurface (scene);
          rf_est.compute (*scene_rf);

          //  Clustering
          pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
          clusterer.setHoughBinSize (cg_size_);
          clusterer.setHoughThreshold (cg_thresh_);
          clusterer.setUseInterpolation (true);
          clusterer.setUseDistanceWeight (false);

          clusterer.setInputCloud (model_keypoints);
          clusterer.setInputRf (model_rf);
          clusterer.setSceneCloud (scene_keypoints);
          clusterer.setSceneRf (scene_rf);
          clusterer.setModelSceneCorrespondences (model_scene_corrs);

          //clusterer.cluster (clustered_corrs);
          clusterer.recognize (rototranslations, clustered_corrs);
        }
        else {// Using GeometricConsistency
          pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
          gc_clusterer.setGCSize (cg_size_);
          gc_clusterer.setGCThreshold (cg_thresh_);

          gc_clusterer.setInputCloud (model_keypoints);
          gc_clusterer.setSceneCloud (scene_keypoints);
          gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

          //gc_clusterer.cluster (clustered_corrs);
          gc_clusterer.recognize (rototranslations, clustered_corrs);
        }

        // Publish:


        for (size_t i = 0; i < rototranslations.size (); ++i)
        {
            geometry_msgs::TransformStamped trans_msg;
            Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
            Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);
            tf::Quaternion q;
            tf::Matrix3x3 mat(rotation (0,0), rotation (0,1), rotation (0,2),
                              rotation (1,0), rotation (1,1), rotation (1,2),
                              rotation (2,0), rotation (2,1), rotation (2,2));

            mat.getRotation(q); //turn to quaternion
            tfScalar r, p, y;
            mat.getEulerYPR(y, p, r);
            printf("rpy angle is %lf,  %lf,  %lf \n", r, p, y);
            trans_msg.transform.rotation.x = q.getX();
            trans_msg.transform.rotation.y = q.getY();
            trans_msg.transform.rotation.z = q.getZ();
            trans_msg.transform.rotation.w = q.getW();
            trans_msg.transform.translation.x = translation (0);
            trans_msg.transform.translation.y = translation (1);
            trans_msg.transform.translation.z = translation (2);
            trans_msg.header = msg_pt->header;
            trans_msg.child_frame_id = "charge_model_link";
            if (rotation(0,0) > 0.7 && rotation(1,1) > 0.85 && rotation(2,2) > 0.85 && rotation(0,0) != 1) {
                transformColorBased.setOrigin(tf::Vector3(translation(0), translation(1), translation(2)));
                transformColorBased.setRotation(q);
                transformInitialized = true;
                pub_trans.publish(trans_msg);
            }
        }
        if (transformInitialized) {
            br.sendTransform(tf::StampedTransform(transformColorBased, msg_pt->header.stamp, "color", "charger"));
        }
        output(rototranslations, clustered_corrs);
    }

private:
    ros::NodeHandle n_;
    ros::NodeHandle np_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_pointcloud;
    message_filters::Subscriber<jsk_recognition_msgs::BoundingBox> *sub_boundingbox;
    message_filters::Synchronizer<syncPolicy> *sync;
    ros::Publisher pub_filtered;
    ros::Publisher pub_model;
    ros::Publisher pub_trans;

    dynamic_reconfigure::Server<templatematching::matchingparamConfig> server;
    dynamic_reconfigure::Server<templatematching::matchingparamConfig>::CallbackType f;

    tf::TransformBroadcaster br;
    tf::Transform transformColorBased;
    bool transformInitialized;
};




int main (int argc, char *argv[]) {
  // parseCommandLine (argc, argv);



  //
  //  Load clouds
  //
//  if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
//  {
//    std::cout << "Error loading model cloud." << std::endl;
//    showHelp (argv[0]);
//    return (-1);
//  }
//  if (pcl::io::loadPCDFile (scene_filename_, *scene) < 0)
//  {
//    std::cout << "Error loading scene cloud." << std::endl;
//    showHelp (argv[0]);
//    return (-1);
//  }





  //
  //  Downsample Clouds to Extract keypoints
  //

  // pcl::UniformSampling<PointType> uniform_sampling;
  // uniform_sampling.setInputCloud (model);
  // uniform_sampling.setRadiusSearch (model_ss_);
  // uniform_sampling.filter (*model_keypoints);
  // std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

  // uniform_sampling.setInputCloud (scene);
  // uniform_sampling.setRadiusSearch (scene_ss_);
  // uniform_sampling.filter (*scene_keypoints);
  // std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;






  //
  //  Visualization
  //
//  pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
//  viewer.addPointCloud (scene, "scene_cloud");

//  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
//  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

//  if (show_correspondences_ || show_keypoints_)
//  {
//    //  We are translating the model so that it doesn't end in the middle of the scene representation
//    pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
//    pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

//    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
//    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
//  }

//  if (show_keypoints_)
//  {
//    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
//    viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

//    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
//    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
//  }

//  for (size_t i = 0; i < rototranslations.size (); ++i)
//  {
//    pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
//    pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

//    std::stringstream ss_cloud;
//    ss_cloud << "instance" << i;

//    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
//    viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

//    if (show_correspondences_)
//    {
//      for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
//      {
//        std::stringstream ss_line;
//        ss_line << "correspondence_line" << i << "_" << j;
//        PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
//        PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

//        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
//        viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
//      }
//    }
//  }

//  while (!viewer.wasStopped ())
//  {
//    viewer.spinOnce ();
//  }


    ros::init(argc, argv, "pointcloud_filter");
    ros::NodeHandle n;
    ros::NodeHandle np("~");
    TemplateMatching tpm(n, np);

    while (ros::ok()) {
        ros::spinOnce();
        boost::this_thread::sleep (boost::posix_time::microseconds (50000));
    }
    return (0);
}
