#include "costcube.h"

CostCube::CostCube(double len,double res){
        side_length = len;
        resoluion = res;
        voxel_n = side_length / resoluion;
        size[0] = size[1] = size[2] = voxel_n;
}

cv::Mat CostCube::calCostCubeByBresenham3D(vector<geometry_msgs::Point> map_points,geometry_msgs::Pose camera_pose){
                processMapPts(map_points,0,0,camera_pose.position);
        //cost = exp(-1.0 * cost_scaling_factor * (distance_from_obstacle – inscribed_radius)) * (costmap_2d::INSCRIBED_INFLATED_OBSTACLE – 1)
        map_prob = cv::Mat::zeros(3,size,CV_8UC1);
        for (int row = 0; row < voxel_n; ++row)
	{
		for (int col = 0; col < voxel_n; ++col)
		{
                        for (int hei = 0;hei < voxel_n; ++ hei){
                                int visits = visit_counter.at<int>(row, col,hei);
                                int occupieds = occupied_counter.at<int>(row, col,hei);
                                int maxVisitNum = *max_element(visit_counter.begin<int>(),visit_counter.end<int>());

                                if (occupieds){
                                        map_prob.at<uchar>(row, col, hei) = 0;
                                }
                                else if (visits <= free_thresh)
                                {
                                        map_prob.at<uchar>(row, col, hei) = 255;
                                        //grid_map_proba.at<uchar>(row, col) = 128;
                                }
                                else if(visits > occupied_thresh){
                                        map_prob.at<uchar>(row, col, hei) = 0;
                                }
                                else{
                                        map_prob.at<uchar>(row, col, hei) = int((1-1.0*visits/maxVisitNum)*255);
                                }
                        }
                }
	}
        return map_prob;
}

void CostCube::processMapPts(const std::vector<geometry_msgs::Point> &pts, unsigned int n_pts,
				   unsigned int start_id, const geometry_msgs::Point &cam_pos){
        occupied_counter = cv::Mat::zeros(3,size,CV_32SC1);
	visit_counter = cv::Mat::zeros(3,size,CV_32SC1);
        // unsigned int end_id = start_id + n_pts;
        // for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id)
        for (unsigned int pt_id = 0; pt_id < pts.size(); ++pt_id)
        {
                Bresenham3D(pts[pt_id], occupied_counter, visit_counter,cam_pos);
        }
}

void CostCube::Bresenham3D(const geometry_msgs::Point &pt_pos, cv::Mat &occupied,
				  cv::Mat &visited,const geometry_msgs::Point &cam_pos){
        // https://gist.github.com/yamamushi/5823518#file-bresenham3d-L11
        int x1 = int(voxel_n/2);
        int y1 = int(voxel_n/2);
        int z1 = int(voxel_n/2);
        int x2 = int((pt_pos.x - cam_pos.x)/resoluion + voxel_n/2);
        int y2 = int((pt_pos.y - cam_pos.y)/resoluion + voxel_n/2);
        int z2 = int((pt_pos.z - cam_pos.y)/resoluion + voxel_n/2);
	if (x2 < 0 || x2 >= voxel_n||y2 < 0 || y2 >= voxel_n||z2 < 0 || z2 >= voxel_n){
                cout << "Target index [ "<< x2 << " , " << y2 << " , " << z2 << " ] out of bound "<< voxel_n << "(maximum) when calculating Bresenham3D" << endl;
                return;
        }
	// Increment the occupency account of the grid cell where map point is located
	++occupied.at<int>(x2,y2,z2);
        occupied_ind.push_back(vector<int>{x2,y2,z2});

        int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;
        int point[3];
        
        point[0] = x1;
        point[1] = y1;
        point[2] = z1;
        dx = x2 - x1;
        dy = y2 - y1;
        dz = z2 - z1;
        x_inc = (dx < 0) ? -1 : 1;
        l = abs(dx);
        y_inc = (dy < 0) ? -1 : 1;
        m = abs(dy);
        z_inc = (dz < 0) ? -1 : 1;
        n = abs(dz);
        dx2 = l << 1;
        dy2 = m << 1;
        dz2 = n << 1;
        
        if ((l >= m) && (l >= n)) {
                err_1 = dy2 - l;
                err_2 = dz2 - l;
                for (i = 0; i < l; i++) {                
                ++visited.at<int>(point[0], point[1], point[2]);
                if (err_1 > 0) {
                        point[1] += y_inc;
                        err_1 -= dx2;
                }
                if (err_2 > 0) {
                        point[2] += z_inc;
                        err_2 -= dx2;
                }
                err_1 += dy2;
                err_2 += dz2;
                point[0] += x_inc;
                }
        } else if ((m >= l) && (m >= n)) {
                err_1 = dx2 - m;
                err_2 = dz2 - m;
                for (i = 0; i < m; i++) {
                ++visited.at<int>(point[0], point[1], point[2]);
                if (err_1 > 0) {
                        point[0] += x_inc;
                        err_1 -= dy2;
                }
                if (err_2 > 0) {
                        point[2] += z_inc;
                        err_2 -= dy2;
                }
                err_1 += dx2;
                err_2 += dz2;
                point[1] += y_inc;
                }
        } else {
                err_1 = dy2 - n;
                err_2 = dx2 - n;
                for (i = 0; i < n; i++) {
                ++visited.at<int>(point[0], point[1], point[2]);
                if (err_1 > 0) {
                        point[1] += y_inc;
                        err_1 -= dz2;
                }
                if (err_2 > 0) {
                        point[0] += x_inc;
                        err_2 -= dz2;
                }
                err_1 += dy2;
                err_2 += dx2;
                point[2] += z_inc;
                }
        }
        ++visited.at<int>(point[0], point[1], point[2]);
}

cv::Mat CostCube::calCostCubeByDistance(vector<geometry_msgs::Point> map_points,geometry_msgs::Pose camera_pose){
        processMapPts(map_points,0,0,camera_pose.position);
        map_prob = cv::Mat::zeros(3,size,CV_8UC1);
        for (int row = 0; row < voxel_n; ++row)
		for (int col = 0; col < voxel_n; ++col)		
                        for (int hei = 0;hei < voxel_n; ++ hei){
                                // TODO : Maybe need normalization? 
                                double dst = dstFromVoxelToObstacle(vector<int>{row,col,hei});
                                map_prob.at<uchar>(row, col, hei) = computeCostByDistance(dst);
                        }
        return map_prob;
}

double CostCube::dstFromVoxelToObstacle(vector<int> pos_id){
//Calculate average distance between current voxel and nearby obstacle. 
        if(pos_id.size()!=3){
                cout << "Wrong dim of voxel index has been input!";
                return -1;
        }
        vector<float> dst_vec;
        for(uint i=0;i<occupied_ind.size();++i){
                dst_vec.push_back(sqrt( pow(occupied_ind[i][0]-pos_id[0],2)+pow(occupied_ind[i][1]-pos_id[1],2)+pow(occupied_ind[i][2]-pos_id[2],2)));
        }
        sort(dst_vec.begin(),dst_vec.end());
        float dst_thresh = (dst_vec[-1] - dst_vec[0])/10;
        double dst = 0.0;
        int i;
        for(i=0;dst_vec[i]<=dst_thresh;++i){
                dst +=  dst_vec[i];
        }
        return dst/i;
}

unsigned char CostCube::computeCostByDistance(const double distance)
  {
    unsigned char cost = 0;
    if (distance == 0)
      cost = LETHAL_OBSTACLE;
    else if (distance  <= inscribed_radius_)
      cost = INSCRIBED_INFLATED_OBSTACLE;
    else
    {
      // make sure cost falls off by Euclidean distance
      double factor = exp(-1.0 * cost_scaling_factor * (distance - inscribed_radius_));
      cost = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }