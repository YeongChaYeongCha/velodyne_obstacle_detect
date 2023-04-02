# velodyne_obstacle_detect

cpp_tutorial : 이곳에 있는 패키지들은 모두 c++기반으로 짜여졌기 때문에 c++의 class에 대해 무조건 알아야 한다.

velo_filter :  노트북으로 제어하기 때문에 점군을 더욱 줄이기 위해 Voxel_Filter, SOR_Filter,  처리한 후 pointcloud publish

              이때 filter 처리 후 point_cloud에 intensity, ring, label까지 부여해야 이후의 plane_fit_ground_filter 패키지에서 경고없이 pointcloud를 받을 수 있다.
              
plane_fit_ground_filter : 지면제거한 후 pointcloud publish

lidar_obstacle_detector : filter 처리 후 객체
