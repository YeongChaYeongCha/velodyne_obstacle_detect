#include "plane_ground_filter_core.h"

/*
    @brief Compare function to sort points. Here use z axis.
    @return z-axis accent
*/
bool point_cmp(VPoint a, VPoint b)
{
    return a.z < b.z;
}

PlaneGroundFilter::PlaneGroundFilter(ros::NodeHandle &nh)
{
    std::string input_topic;
    nh.getParam("input_topic", input_topic);
    //sub_point_cloud_ = nh.subscribe(input_topic, 10, &PlaneGroundFilter::point_cb, this);
    sub_point_cloud_ = nh.subscribe("/filtered_point", 10, &PlaneGroundFilter::point_cb, this);
    // sub_point_cloud_ = nh.subscribe("/rslidar_points", 10, &PlaneGroundFilter::point_cb, this);

    // init publisher
    std::string no_ground_topic, ground_topic, all_points_topic;

    nh.getParam("no_ground_point_topic", no_ground_topic);
    nh.getParam("ground_point_topic", ground_topic);
    nh.getParam("all_points_topic", all_points_topic);

    nh.getParam("clip_height", clip_height_);
    ROS_INFO("clip_height: %f", clip_height_);
    nh.getParam("sensor_height", sensor_height_);
    ROS_INFO("sensor_height: %f", sensor_height_);
    nh.getParam("min_distance", min_distance_);
    ROS_INFO("min_distance: %f", min_distance_);
    nh.getParam("max_distance", max_distance_);
    ROS_INFO("max_distance: %f", max_distance_);

    nh.getParam("sensor_model", sensor_model_);
    ROS_INFO("sensor_model: %d", sensor_model_);
    nh.getParam("num_iter", num_iter_);
    ROS_INFO("num_iter: %d", num_iter_);
    nh.getParam("num_lpr", num_lpr_);
    ROS_INFO("num_lpr: %d", num_lpr_);
    nh.getParam("th_seeds", th_seeds_);
    ROS_INFO("th_seeds: %f", th_seeds_);
    nh.getParam("th_dist", th_dist_);
    ROS_INFO("th_dist: %f", th_dist_);

    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>(ground_topic, 10);
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 10);
    pub_all_points_ = nh.advertise<sensor_msgs::PointCloud2>(all_points_topic, 10);

    g_seeds_pc = pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
    g_ground_pc = pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
    g_not_ground_pc = pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
    g_all_pc = pcl::PointCloud<SLRPointXYZIRL>::Ptr(new pcl::PointCloud<SLRPointXYZIRL>);

    ros::spin();
}

PlaneGroundFilter::~PlaneGroundFilter() {}

void PlaneGroundFilter::Spin()
{
}

void PlaneGroundFilter::clip_above(const pcl::PointCloud<VPoint>::Ptr in,
                                   const pcl::PointCloud<VPoint>::Ptr out)
{
    pcl::ExtractIndices<VPoint> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        if (in->points[i].z > clip_height_)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}

void PlaneGroundFilter::remove_close_far_pt(const pcl::PointCloud<VPoint>::Ptr in,
                                            const pcl::PointCloud<VPoint>::Ptr out)
{
    pcl::ExtractIndices<VPoint> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);

        if ((distance < min_distance_) || (distance > max_distance_))
        {
			//std::cout<<"distance : "<<distance<<std::endl;
            indices.indices.push_back(i);
        }
    }
    //std::cout<<"indices : "<<std::endl<<indices<<std::endl;
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}

/*
    @brief The function to estimate plane model. The
    model parameter `normal_` and `d_`, and `th_dist_d_`
    is set here.
    The main step is performed SVD(UAV) on covariance matrix.
    Taking the sigular vector in U matrix according to the smallest
    sigular value in A, as the `normal_`. `d_` is then calculated 
    according to mean ground points.

    @param g_ground_pc:global ground pointcloud ptr.
    
*/
void PlaneGroundFilter::estimate_plane_(void)
{
    // Create covarian matrix in single pass.
    // TODO: compare the efficiency.
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean); //pcl::computeMeanAndCovarianceMatrix(cloud(in), covariance_matrix(out), centroid(out))
    //std::cout<<"\ncov : "<<std::endl<<cov<<std::endl;
    //std::cout<<"\npc_mean : "<<std::endl<<pc_mean<<std::endl;
    // Singular Value Decomposition: SVD
    JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    //std::cout<<"\nsvd.MatrixU : "<<std::endl<<svd.matrixU()<<std::endl;
    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    //std::cout<<"\nnormal : "<<std::endl<<normal_<<std::endl;
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>(); //4개 요소가 있는 pc_mean의 3번째 요소까지를 seeds_mean에 대입.
	//std::cout<<"\nseeds_mean : "<<std::endl<<seeds_mean<<std::endl;

    // according to normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose() * seeds_mean)(0, 0); //.transpose() : 전치행렬
    //std::cout<<"\nd_ : "<<d_<<std::endl;
    // set distance threhold to `th_dist - d`
    th_dist_d_ = th_dist_ - d_;
    
    //std::cout<<"th_dist_d_ : "<<th_dist_d_<<std::endl;
    //std::cout<<"-----------------------"<<std::endl;

    // return the equation parameters
}

/*
    @brief Extract initial seeds of the given pointcloud sorted segment.
    This function filter ground seeds points accoring to heigt.
    This function will set the `g_ground_pc` to `g_seed_pc`.
    @param p_sorted: sorted pointcloud
    
    @param ::num_lpr_: num of LPR points
    @param ::th_seeds_: threshold distance of seeds
    @param ::
*/
void PlaneGroundFilter::extract_initial_seeds_(const pcl::PointCloud<VPoint> &p_sorted)
{
    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;
    // Calculate the mean height value.
    //ROS_INFO("p_sorted_size : %d\n", p_sorted.points.size());
    for (int i = 0; i < p_sorted.points.size() && cnt < num_lpr_; i++) //조건이 2가지 모두 충족시켜야 하지만 보통 p_sorted.poins.size()가 몇 만개이기 때문에 cnt<num_lpr_만 만족시키면 될 듯?
    {//의문점 1.보통 /velodyne_points으로부터 들어오는 점의 개수는 만 단위인데 왜 조건을 i<p_sorted.points.size() && cnt<num_lpr으로 두었나.
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0; // in case divide by 0 | cnt != 0일 경우 lpr_height=sum/cnt(점군의 z값의 평균), cnt==0일 경우 lpr_height=0
    //ROS_INFO("lpr_height+th_seeds : %f", lpr_height+th_seeds_);
    g_seeds_pc->clear(); //51번째 줄 선언(velodyne_pointcloud::PointXYZIR 데이터 타입을 가짐)한 g_seeds_pc를 초기화
    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++)
    {
        if (p_sorted.points[i].z < lpr_height + th_seeds_)
        {
            g_seeds_pc->points.push_back(p_sorted.points[i]);
        }
    }
    // return seeds points
}

void PlaneGroundFilter::post_process(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out)
{
    pcl::PointCloud<VPoint>::Ptr cliped_pc_ptr(new pcl::PointCloud<VPoint>);
    clip_above(in, cliped_pc_ptr);
    pcl::PointCloud<VPoint>::Ptr remove_close(new pcl::PointCloud<VPoint>);
    remove_close_far_pt(cliped_pc_ptr, out);
}

void PlaneGroundFilter::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    // 1.Msg to pointcloud (1.raw data를 pcl::PointCloud로 변환)
    pcl::PointCloud<VPoint> laserCloudIn; //plane_ground_filter_core.h에 VPoint 명시(velodyne_pointcloud::PointXYZIR)
    pcl::fromROSMsg(*in_cloud_ptr, laserCloudIn); //Velodyne VLP16으로부터 받은 raw data를 pcl pointcloud로 변환

    pcl::PointCloud<VPoint> laserCloudIn_org;
    pcl::fromROSMsg(*in_cloud_ptr, laserCloudIn_org); //Velodyne VLP16으로부터 받은 raw data를 pcl pointcloud로 변환
    // For mark ground points and hold all points
    SLRPointXYZIRL point; //plane_ground_filter_core.h에 SLRPointXYZIRL 명시(plane_ground_filter::PointXYZIRL)

	ROS_INFO("laserCloudIn_size : %d", laserCloudIn.points.size());
    for (size_t i = 0; i < laserCloudIn.points.size(); i++)
    { //for문을 통해 데이터형이 plane_ground_filter::PointXYZIRL의 각각 x,y,z,intensity, ring, lable을 초기화해준다.
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        point.intensity = laserCloudIn.points[i].intensity;
        point.ring = laserCloudIn.points[i].ring;
        point.label = 0u; // 0 means uncluster
        g_all_pc->points.push_back(point); //plane_ground_filter_core.h에 선언되어 있는 g_all_pc에 point를 추가
    }
    //std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn,indices);
    // 2.Sort on Z-axis value. (2.z값에 따른 정렬)
    //ROS_INFO("%f | %f |%f", laserCloudIn.begin()->x, laserCloudIn.begin()->y, laserCloudIn.begin()->z);
    //ROS_INFO("%f", sensor_height_*-1.5);
    //ROS_INFO("%x", laserCloudIn.begin());
    sort(laserCloudIn.points.begin(), laserCloudIn.end(), point_cmp); //sort함수를 통해 laserCloudIn의 point.z를 비교하여 오름차순 정리한다.
    // 3.Error point removal (3.z값에 따른 점 제거)
    // As there are some error mirror reflection under the ground,
    // here regardless point under 2* sensor_height
    // Sort point according to height, here uses z-axis in default
    pcl::PointCloud<VPoint>::iterator it = laserCloudIn.points.begin();
    for (int i = 0; i < laserCloudIn.points.size(); i++)
    {
        if (laserCloudIn.points[i].z < -1.5 * sensor_height_) //29번째 줄에 getParam을 통해서 sensor_height_ 초기화
        {
			//ROS_INFO("%d %f\n", i, laserCloudIn.points[i].z);
            it++;
        }
        else
        {
            break;
        }
    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(), it);
    // 4. Extract init ground seeds.(4. 초기 지면 점군 추출)
    extract_initial_seeds_(laserCloudIn);
    g_ground_pc = g_seeds_pc; 
    // 5. Ground plane fitter mainloop
    for (int i = 0; i < num_iter_; i++)
    {
        estimate_plane_();
        g_ground_pc->clear();
        g_not_ground_pc->clear();

        //pointcloud to matrix
        MatrixXf points(laserCloudIn_org.points.size(), 3); //행렬 크기를 동적으로 할당하고 행 크기를 점의 개수, 열 크기를 3으로 하여 초기화.
        //std::cout<<"points : "<<std::endl<<points<<std::endl;
        
        int j = 0;
        for (auto p : laserCloudIn_org.points)
        {
            points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model
        VectorXf result = points * normal_;
        //std::cout<<"result : "<<std::endl<<result<<std::endl;
        // threshold filter
        for (int r = 0; r < result.rows(); r++)
        {
            if (result[r] < th_dist_d_)
            {
                g_all_pc->points[r].label = 1u; // means ground
                g_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
            else
            {
                g_all_pc->points[r].label = 0u; // means not ground and non clusterred
                g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
        }
    }

    pcl::PointCloud<VPoint>::Ptr final_no_ground(new pcl::PointCloud<VPoint>);
    post_process(g_not_ground_pc, final_no_ground); //바닥이 아닌 점군과 OUTPUT 점군을 post_process 함수로 넘겨준다.
    
    // ROS_INFO_STREAM("origin: "<<g_not_ground_pc->points.size()<<" post_process: "<<final_no_ground->points.size());

    // publish ground points
    sensor_msgs::PointCloud2 ground_msg;
    pcl::toROSMsg(*g_ground_pc, ground_msg);
    ground_msg.header.stamp = in_cloud_ptr->header.stamp;
    ground_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    pub_ground_.publish(ground_msg);

    // publish not ground points
    sensor_msgs::PointCloud2 groundless_msg;
    pcl::toROSMsg(*final_no_ground, groundless_msg);
    groundless_msg.header.stamp = in_cloud_ptr->header.stamp;
    groundless_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    pub_no_ground_.publish(groundless_msg);

    // publish all points
    sensor_msgs::PointCloud2 all_points_msg;
    pcl::toROSMsg(*g_all_pc, all_points_msg);
    all_points_msg.header.stamp = in_cloud_ptr->header.stamp;
    all_points_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    pub_all_points_.publish(all_points_msg);
    g_all_pc->clear();
}
