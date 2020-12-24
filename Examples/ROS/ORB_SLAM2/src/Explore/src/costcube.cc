#include "costcube.h"

CostCube::CostCube(double len,double res){
        side_length = len;
        resoluion = res;
        cell_num = side_length / resoluion;
}

CostCube::Cube CostCube::getCostCube(vector<geometry_msgs::Point> map_points,geometry_msgs::Pose camera_pose){
        Cube cube;
        int i,j,k;
        for(int i = 0;i<cell_num;++i)
                for(int j = 0;j<cell_num;++j)
                        for(int k = 0;k<cell_num;++k){
                                
                        }
                
        
}
