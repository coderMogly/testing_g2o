#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Geometry>

class Sim_trans{
public:
  Sim_trans(Eigen::Quaterniond q, Eigen::Vector3d tran, float scale);
  Eigen::Quaterniond quat();
  Eigen::Vector3d trans();
  float scale();
  //Sim_trans multiply(Sim_trans &other);
  void setquat();
  void settrans();
  void setscale();


protected:
  Eigen::Quaterniond q;
  Eigen::Vector3d tran;
  float sc;

};

Sim_trans::Sim_trans(Eigen::Quaterniond _q, Eigen::Vector3d _tran, float _scale){
  q = _q;
  tran = _tran;
  sc = _scale;
}

Eigen::Quaterniond Sim_trans::quat(){
  return q;
}

Eigen::Vector3d Sim_trans::trans(){
  return tran;
}

float Sim_trans::scale(){
  return sc;
}

void Sim_trans::setquat(Eigen::Quaterniond &_q){
  q = _q
}

void Sim_trans::settrans(Eigen::Vector3d &_tran){
  tran = _tran
}

void Sim_trans::setscale(float &_scale){
  sc = _scale
}

//Sim_trans Sim_trans::multiply(Sim_trans &other){
//  Sim_trans ret;
//  ret.setquat(((q.normalized().toRotationMatrix())*(other.quat().normalized().toRotationMatrix())));
//  ret.settrans();
//  ret.scale();
//  return ret;    
//}














pcl::PointXYZ transform_point(pcl::PointXYZ inp, Sim_trans &si_trans){
  pcl::PointXYZ ret;
  Eigen::Quaterniond q = si_tran.quat();
  Eigen::Vector3d tran = si_tran.trans();
  float scale = si_tran.scale()
  Eigen::Matrix3d rot = q.normalized().toRotationMatrix();
  Eigen::Vector3d in;
  in[0] = inp.x;
  in[1] = inp.y;
  in[2] = inp.z;

  Eigen::Vector3d out = scale*(rot*in)+tran;

  ret.x = out[0];
  ret.y = out[1]; 
  ret.z = out[2];
  return ret; 
}


pcl::PointCloud<pcl::PointXYZ>::Ptr loadPCL(std::string filename){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  return cloud;
  
}

pcl::KdTreeFLANN<pcl::PointXYZ> saveAsKDTree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  return kdtree;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transform_PCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Sim_trans tran){
  pcl::PointCloud<pcl::PointXYZ>::Ptr ret (new pcl::PointCloud<pcl::PointXYZ>);
  ret->points.resize(cloud->points.size());
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    pcl::PointXYZ temp_point = transform_point(cloud->points[i], tran);
    ret->points[i].x = temp_point.x;
    ret->points[i].y = temp_point.y;
    ret->points[i].z = temp_point.z;
  }
  return ret;

}

float InitialTHguess(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  float th = 0;
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    pcl::PointXYZ searchPoint;
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    if(pointNKNSquaredDistance[0] > th){
      th = pointNKNSquaredDistance[0];
    }
  }

  return th;
}

void updateTH_max(float &th_max, float temp){
  if(th_max<temp){
    th_max = temp;    
  }
}

void updateTH_min(float &th_min, float temp){
  if(th_min>temp){
    th_min = temp;    
  }
}

std::std::vector<std::pair<int, int>> findCorrespondences(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float &th, float &th_max, float &th_min){
  std::std::vector<std::pair<int, int>> ret;
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    pcl::PointXYZ searchPoint;
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    if(pointNKNSquaredDistance[0] < th){
      ret.push_back(std::make_pair(i,pointIdxNKNSearch[0]));
    } 
    updateTH_max(th_max, pointNKNSquaredDistance[0]);
    updateTH_min(th_min, pointNKNSquaredDistance[0]);
  }
  return ret;
}


Sim_trans findSimilarityTrans(pcl::PointCloud<pcl::PointXYZ>::Ptr globalPCL, pcl::PointCloud<pcl::PointXYZ>::Ptr localPCL, std::vector<std::pair<int,int>> corres_set){

}






int main ()
{
  //parameters
  float th = 0;
  float th_max = 0;
  float th_min = 0;
  float iter = 1;
  Eigen::Quaterniond q = ;
  Eigen::Vector3d tran = ;
  float scale = ;
  Sim_trans si(q, tran, scale);

  //enter global pcd filename here 
  std::string gl_filename = "./globalPCD.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr globalPCL = loadPCL(gl_filename);
  pcl::KdTreeFLANN<pcl::PointXYZ> globalKDTree = saveAsKDTree(globalPCL);
  


  //enter local pcd filename here
  std::string lo_filename = "./localPCD.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr localPCL = loadPCL(lo_filename);

  //transform local pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_transformed_localPCL = transform_PCL(localPCL, si);



  //estimating initial threshold guess
  th = InitialTHguess(globalKDTree, temp_transformed_localPCL);
  th_max = th;
  th_min = th;


  //starting the Iterative process
  Sim_trans temp = si;
  for(int j =0; j<iter; j++){
    std::vector<std::pair<int,int>> corres_set = findCorrespondences(globalKDTree, transformed_localPCL, th, th_max, th_min);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_localPCL = transform_PCL(localPCL, temp);    
    temp = findSimilarityTrans(globalPCL, transformed_localPCL, corres_set);
    //temp = temp * si;
  }

  //final transformation out is temp

  return (0);
}