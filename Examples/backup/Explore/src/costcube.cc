#include "costcube.h"

CostCube::CostCube(double shooting_dst,double field_size,double resolution){
        shooting_dst = shooting_dst;
        field_size = field_size;
        resolution = resolution;
        size[0] = size[1] = 2 * field_size / resolution;
        size[2] = shooting_dst / resolution;
}

void CostCube::reinitialize(double shooting_dst,double field_size,double resolution){
        shooting_dst = shooting_dst;
        field_size = field_size;
        resolution = resolution;
        size[0] = size[1] = 2 * field_size / resolution;
        size[2] = shooting_dst / resolution;
}

cv::Mat CostCube::calCostCubeByBresenham3D(vector<geometry_msgs::Point> map_points){
        map_prob = cv::Mat::zeros(3,size,CV_8UC1);
        if(map_points.size()==0)
                return map_prob;
        processMapPts(map_points);
        //cost = exp(-1.0 * cost_scaling_factor * (distance_from_obstacle – inscribed_radius)) * (costmap_2d::INSCRIBED_INFLATED_OBSTACLE – 1)
        int maxVisitNum = *max_element(visit_counter.begin<int>(),visit_counter.end<int>());
        for (int row = 0; row < size[0]; ++row)
	{
		for (int col = 0; col < size[1]; ++col)
		{
                        for (int hei = 0;hei < size[2]; ++ hei){
                                // continue;
                                int visits = visit_counter.at<int>(row, col,hei);
                                int occupieds = occupied_counter.at<int>(row, col,hei);                                

                                if (occupieds){
                                        map_prob.at<uint>(row, col, hei) = 0;
                                }
                                else if (visits <= free_thresh)
                                {
                                        map_prob.at<uint>(row, col, hei) = 255;
                                        //grid_map_proba.at<uint>(row, col) = 128;
                                }
                                else if(visits > occupied_thresh){
                                        map_prob.at<uint>(row, col, hei) = 0;
                                }
                                else{
                                        map_prob.at<uint>(row, col, hei) = int((1-1.0*visits/maxVisitNum)*255);
                                }
                        }
                }
	}
        return map_prob;
}

void CostCube::processMapPts(const std::vector<geometry_msgs::Point> &pts,bool cal_occupied_only){
        occupied_counter = cv::Mat::zeros(3,size,CV_32SC1);
	visit_counter = cv::Mat::zeros(3,size,CV_32SC1);
        // unsigned int end_id = start_id + n_pts;
        // for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id)

        // int num;
        for (unsigned int pt_id = 0; pt_id < pts.size(); ++pt_id)
        {
                double dst = sqrt(pow((pts[pt_id].x),2)+pow((pts[pt_id].y),2)+pow((pts[pt_id].z),2));
                if(dst > shooting_dst)
                        continue;
                Bresenham3D(pts[pt_id], occupied_counter, visit_counter,cal_occupied_only);
                // num++;
        }
        // cout << "size of occupied_ind after Bresenham3D algorithm: " << occupied_ind.size() << " , while size of map_points is " << pts.size() << endl;
}

void CostCube::Bresenham3D(const geometry_msgs::Point &pt_pos, cv::Mat &occupied,cv::Mat &visited,bool cal_occupied_only){
        // https://gist.github.com/yamamushi/5823518#file-bresenham3d-L11
        // int x1 = int(size[0]/2);
        // int y1 = int(size[1]/2);
        // int z1 = int(size[2]/2);
        // int x2 = int((pt_pos.x - cam_pos.x)/resolution + size[0]/2);
        // int y2 = int((pt_pos.y - cam_pos.y)/resolution + size[1]/2);
        // int z2 = int((pt_pos.z - cam_pos.z)/resolution + size[2]/2);
        int x1 = int(size[0]/2);
        int y1 = int(size[1]/2);
        int z1 = 0;
        int x2 = int((pt_pos.x )/resolution + x1);
        int y2 = int((pt_pos.y )/resolution + y1);
        int z2 = int((pt_pos.z )/resolution  + z1);
	if (x2 < 0 || x2 >= size[0]||y2 < 0 || y2 >= size[1]||z2 < 0 || z2 >= size[2]){
                cout << "Target index [ "<< x2 << " , " << y2 << " , " << z2 << " ] out of bound [" << size[0] << " , " 
                          << size[1] << " , " << size[2]  << "](maximum) when calculating Bresenham3D" << endl;
                return;
        }
	// Increment the occupency account of the grid cell where map point is located
	++occupied.at<int>(x2,y2,z2);
        occupied_ind.push_back(vector<int>{x2,y2,z2});
        if(cal_occupied_only) 
                return;
        else{
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
}

cv::Mat CostCube::calCostCubeByDistance(vector<geometry_msgs::Point> map_points){
        map_prob = cv::Mat::zeros(3,size,CV_32FC1);
        dst_mat = cv::Mat::zeros(3,size,CV_32FC1);
        occupied_ind.clear();
        if(map_points.size()==0)
                return map_prob;
        processMapPts(map_points,true);
        for (int row = 0; row < size[0]; ++row)
		for (int col = 0; col < size[1]; ++col)		
                        for (int hei = 0;hei < size[2]; ++ hei){
                                // TODO : Maybe need normalization?
                                // float dst = dstFromVoxelToObstacle(vector<int>{row,col,hei});
                                float dst = dstFromVoxelToObstacle(vector<int>{row,col,hei},map_points);
                                dst_mat.at<float>(row, col, hei) = dst;
                                if(dst == -1)//something wrong happen,dont change map_prob
                                        return map_prob;
                                map_prob.at<float>(row, col, hei) = computeCostByDistance(dst);
                                // cout << dst << " " <<  computeCostByDistance(dst) << endl;
                        }
        return map_prob;
}

float CostCube::dstFromVoxelToObstacle(vector<int> pos_id){
//Calculate average distance between current voxel and nearby obstacle. 
        if(pos_id.size()!=3){
                cout << "Wrong dim of voxel index has been input!";
                return -1;
        }
        vector<float> dst_vec;
        //calculate distance only between current voxel and the voxel which is occupied.
        int occ_n = occupied_ind.size();
        if(occ_n <= 0){
                cout <<"No obstacle points detected when calculate distance from voxel to obstacle! return -1" << endl;
                return -1;
        }        
        for(uint i=0;i<occ_n;++i){
                dst_vec.push_back(resolution * sqrt(pow(occupied_ind[i][0]-pos_id[0],2)+pow(occupied_ind[i][1]-pos_id[1],2)+pow(occupied_ind[i][2]-pos_id[2],2)));
        }
        sort(dst_vec.begin(),dst_vec.end());        
        float dst_thresh = (dst_vec.back() - dst_vec.front()) * occ_scale +dst_vec.front() ;
        // cout << occ_n << " " << dst_thresh << " "<<  dst_vec.back() << " " << dst_vec.front() << endl;
        float dst = 0.0;
        int i;
        for(i=0;dst_vec[i]<=dst_thresh;++i){
                dst +=  dst_vec[i];
        }
        return dst/i;
}

float CostCube::dstFromVoxelToObstacle(vector<int> pos_id,vector<geometry_msgs::Point> map_points){
//Calculate average distance between current voxel and all map points in the field of view. 
        if(pos_id.size()!=3){
                cout << "Wrong dim of voxel index has been input!";
                return -1;
        }
        vector<float> dst_vec;
        float x = (pos_id[0] - size[0]/2) * resolution;
        float y = (pos_id[1] - size[1]/2) * resolution;
        float z = (pos_id[2] - size[2]/2) * resolution;
        for(uint i=0;i<map_points.size();++i){
                float dst = sqrt(pow(map_points[i].x - x , 2) + pow(map_points[i].y - y , 2) + pow(map_points[i].z-z , 2));
                dst_vec.push_back(dst);
        }
        sort(dst_vec.begin(),dst_vec.end());        
        float dst_thresh = (dst_vec.back() - dst_vec.front()) * occ_scale +dst_vec.front() ;
        // cout << occ_n << " " << dst_thresh << " "<<  dst_vec.back() << " " << dst_vec.front() << endl;
        float dst = 0.0;
        int i;
        for(i=0;dst_vec[i]<=dst_thresh;++i){
                dst +=  dst_vec[i];
        }
        return dst/i;
}

float CostCube::computeCostByDistance(const float distance)
  {
//     unsigned char cost = 0;
//     if (distance == 0)
//       cost = LETHAL_OBSTACLE;
//     else if (distance  <= inscribed_radius_)
//       cost = INSCRIBED_INFLATED_OBSTACLE;
//     else
//     {
        float cost;
        if(distance < inscribed_radius_){
                cost = 255;                
        }
        else{
                float cost = exp(-1.0 * cost_scaling_factor * (distance - inscribed_radius_));
                // cost = 255 * cost;//(unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
        }
        return cost;
  }