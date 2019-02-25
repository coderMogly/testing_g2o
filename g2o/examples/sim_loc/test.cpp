#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Geometry>

#include <Eigen/StdVector>
#include <random>
#include <iostream>
#include <stdint.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>   
#include <pcl/common/transforms.h>       

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/icp/types_icp.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/types/sim3/sim3.h"
#include "g2o/core/robust_kernel_impl.h"

#include "g2o/types/slam3d/se3_ops.h"
#include "g2o/stuff/misc.h"
#include <pcl/filters/voxel_grid_covariance.h>


using namespace Eigen;
using namespace std;
using namespace g2o;




/*class Sim_trans{
public:
  Sim_trans(Quaternion q, Eigen::Vector3d tran, float scale);
  Quaternion quat();
  Eigen::Vector3d trans();
  float scale();
  //Sim_trans multiply(Sim_trans &other);
  void setquat(Quaternion &_q);
  void settrans(Eigen::Vector3d &_tran);
  void setscale(float &_scale);


protected:
  Quaternion q;
  Eigen::Vector3d tran;
  float sc;

};

Sim_trans::Sim_trans(Quaternion _q, Eigen::Vector3d _tran, float _scale){
  q = _q;
  tran = _tran;
  sc = _scale;
}

Quaternion Sim_trans::quat(){
  return q;
}

Eigen::Vector3d Sim_trans::trans(){
  return tran;
}

float Sim_trans::scale(){
  return sc;
}

void Sim_trans::setquat(Quaternion &_q){
  q = _q;
}

void Sim_trans::settrans(Eigen::Vector3d &_tran){
  tran = _tran;
}

void Sim_trans::setscale(float &_scale){
  sc = _scale;
}

//Sim_trans Sim_trans::multiply(Sim_trans &other){
//  Sim_trans ret;
//  ret.setquat(((q.normalized().toRotationMatrix())*(other.quat().normalized().toRotationMatrix())));
//  ret.settrans();
//  ret.scale();
//  return ret;    
//}


*/

int MIN_POINTS = 100;
float MULTIPLE = 5;

class VoxelData{
public:
	Eigen::Matrix4d matK;
	Eigen::Vector3d sigma;
	int numPoints;
	const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf* leafPTR;
};








pcl::PointXYZ transform_point(pcl::PointXYZ inp, Sim3 &si_tran){
  pcl::PointXYZ ret;
  //Quaternion q = si_tran.getquat();
  //Eigen::Vector3d tran = si_tran.gettrans();
  //float scale = si_tran.getscale();
  //Eigen::Matrix3d rot = q.normalized().toRotationMatrix();
  Eigen::Vector3d in;
  in[0] = inp.x;
  in[1] = inp.y;
  in[2] = inp.z;

  Eigen::Vector3d out = si_tran*in;

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
    //            return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;


//  for (size_t i = 0; i < cloud->points.size (); ++i)
 // {
 //   cout<<filename<<endl<<" x:"<<cloud->points[i].x<<" y:"<<cloud->points[i].y<<" z:"<<cloud->points[i].z<<endl;
 // }
  return cloud;
  
}

pcl::KdTreeFLANN<pcl::PointXYZ> saveAsKDTree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  return kdtree;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transform_PCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Sim3 tran){
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




void InitialTHguess(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float &th, float &th_max, float &th_min){
  th = 0;
  th_max = 0;
  th_min = 100000000000;
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    pcl::PointXYZ searchPoint = cloud->points[i];
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    //cout<<pointIdxNKNSearch[0]<<endl;
    if(pointNKNSquaredDistance[0] > th){
      th = pointNKNSquaredDistance[0];
    }
    updateTH_max(th_max, pointNKNSquaredDistance[0]);
    updateTH_min(th_min, pointNKNSquaredDistance[0]);
  }

  //cout<<"th "<<th<<endl<<"th_max "<<th_max<<endl<<"th_min "<<th_min<<endl;

  //return th;
}


std::vector<std::pair<int, int>> findCorrespondences(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float &th, float &th_max, float &th_min){
  std::vector<std::pair<int, int>> ret;
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    pcl::PointXYZ searchPoint = cloud->points[i];
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    //cout<<pointNKNSquaredDistance[0]<<endl;
    
    if(pointNKNSquaredDistance[0] <= th){
      ret.push_back(std::make_pair(pointIdxNKNSearch[0],i));
    } 
    updateTH_max(th_max, pointNKNSquaredDistance[0]);
    updateTH_min(th_min, pointNKNSquaredDistance[0]);
//    cout<<"  th :"<<th<<endl<<"  th_max :"<<th_max<<endl<<"  th_min :"<<th_min<<"  dist :"<<pointNKNSquaredDistance[0]<<"  index  :"<<pointIdxNKNSearch[0]<<endl<<endl<<endl;
  }
  return ret;
}


Sim3 findSimilarityTrans(pcl::PointCloud<pcl::PointXYZ>::Ptr globalPCL, pcl::PointCloud<pcl::PointXYZ>::Ptr localPCL, std::vector<std::pair<int,int>> corres_set){

  SparseOptimizer optimizer;
  optimizer.setVerbose(false);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<BlockSolverX>(g2o::make_unique<LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));

  optimizer.setAlgorithm(solver);

  // set up two poses
  int vertex_id = 0;
  for (size_t i=0; i<2; ++i)
  {
    // set up rotation, translation and scale for this node
    Vector3d t(0,0,0);
    g2o::Quaternion q;
    q.setIdentity();
    float scale = 1;

    Sim3 cam(q, t, scale); //initiate a sim3 from the above set params 

    // set up node
    VertexSim3 *vc = new VertexSim3();
    vc->setEstimate(cam);

    vc->setId(vertex_id);      // vertex id

    

    // set first sim fixed
    if (i==0)
      vc->setFixed(true);

    // add to optimizer
    optimizer.addVertex(vc);

    vertex_id++;                
  }

//	cout<<"entering for loop"<<endl;
  for (size_t i=0; i<corres_set.size(); i++)
  {
	//cout<<0<<endl;
    // get two poses
    VertexSim3* vp0 = 
      dynamic_cast<VertexSim3*>(optimizer.vertices().find(0)->second);
    VertexSim3* vp1 = 
      dynamic_cast<VertexSim3*>(optimizer.vertices().find(1)->second);
    //cout<<1<<endl;
    // calculate the relative 3D position of the point
    Vector3d pt0,pt1;


    //sim transforming the input pointcloud

    //Vector3d vic = Vector3d(0,0,0);
    //Matrix3 m;
    //m << 0, -1, 0 , 1 ,0 ,0, 0, 0, 1; 

    //pt0 = 2*m*(vp0->estimate().inverse() * true_points[i]) + vic;
    //pt1 = vp1->estimate().inverse() * true_points[i];

    //cout<<3<<endl;
    pt0[0] = globalPCL->points[corres_set[i].first].x;
    pt0[1] = globalPCL->points[corres_set[i].first].y;
    pt0[2] = globalPCL->points[corres_set[i].first].z;

    //cout<<4<<endl;

    pt1[0] = localPCL->points[corres_set[i].second].x;
    pt1[1] = localPCL->points[corres_set[i].second].y;
    pt1[2] = localPCL->points[corres_set[i].second].z; 

    //pt0 = m*pt0 + vic;

    //cout<<5<<endl;

    // form edge
    Vector3d nm0, nm1;


    Edge_V_V_SIM * e           // new edge with correct cohort for caching
        = new Edge_V_V_SIM(); 

    e->setVertex(0, vp0);      // first viewpoint

    e->setVertex(1, vp1);      // second viewpoint

    //cout<<6<<endl;
    //a data structure to calculate the error
    EdgeSIM meas;
    meas.pos0 = pt0;
    meas.pos1 = pt1;

    e->setMeasurement(meas);
    
    
    meas = e->measurement();
    e->information().setIdentity();




    //trying out robust kernel with huber functions


    //e->setRobustKernel(true);
    //e->setHuberWidth(0.01);
    //RobustKernel* rbk;
    //rbk->setDelta(.01);
    //e->setRobustKernel(rbk);
    //g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    //e->setRobustKernel(rk);
    optimizer.addEdge(e);
  //  cout<<"vertex added"<<endl;

  }

//	cout<<"done for loop"<<endl;

  //VertexSim3* vc = 
    //dynamic_cast<VertexSim3*>(optimizer.vertices().find(1)->second);
  //Sim3 cam = vc->estimate();

  
  //printing sim before calculations
  //cout<< cam<<endl;
  

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  cout << "Initial chi2 = " << FIXED(optimizer.chi2()) << endl;

  optimizer.setVerbose(true);

  optimizer.optimize(5);

  //priting sim after calculations
  VertexSim3* temp_ver = dynamic_cast<VertexSim3*>(optimizer.vertices().find(1)->second);
  Sim3 temp_sim = temp_ver->estimate();
  //Sim_trans ret;
  //ret.setquat(temp_sim.getquat());
  //ret.settrans(temp_sim.gettrans());
  //ret.setscale(temp_sim.getscale());


  return temp_sim;

}

float updateTH(const float &th_max, const float &th_min, const float &iter, int k){
	return(th_max - (th_max-th_min)*k/iter);
}


void printPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	for (size_t i = 0; i < cloud->points.size (); ++i)
  	{
    	cout<<endl<<" x:"<<cloud->points[i].x<<" y:"<<cloud->points[i].y<<" z:"<<cloud->points[i].z<<endl;
  	}
}

/*
void doPCA(const std::map<long unsigned int, pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf> &leaves, std::vector<VoxelData> &data){
	std::map<long unsigned int, pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf>::const_iterator itr;
	for(itr = leaves.begin(); itr !=leaves.end(); itr++){
		VoxelData temp;
		Eigen::Vector3d mean = itr->second.getMean();
		Eigen::Matrix3d evecs = itr->second.getEvecs();
		Eigen::Matrix3d temp_cov = itr->second.getCov();
		Eigen::Matrix4d cov;
		Eigen::Matrix4d A;
		temp.matK(0,0) = evecs(0,0);
  		temp.matK(0,1) = evecs(1,0);
  		temp.matK(0,2) = evecs(2,0);
  		temp.matK(1,0) = evecs(0,1);
  		temp.matK(1,1) = evecs(1,1);
  		temp.matK(1,2) = evecs(2,1);
  		temp.matK(2,0) = evecs(0,2);
  		temp.matK(2,1) = evecs(1,2);
  		temp.matK(2,2) = evecs(2,2);
  		temp.matK(0,3) = 0;
  		temp.matK(1,3) = 0;
  		temp.matK(2,3) = 0;
  		temp.matK(3,0) = mean(0);
  		temp.matK(3,1) = mean(1);
  		temp.matK(3,2) = mean(2);
  		temp.matK(3,3) = 1;
  		cov(0,0)=temp_cov(0,0);
  		cov(0,1)=temp_cov(0,1);
  		cov(0,2)=temp_cov(0,2);
  		cov(1,0)=temp_cov(1,0);
  		cov(1,1)=temp_cov(1,1);
  		cov(1,2)=temp_cov(1,2);
  		cov(2,0)=temp_cov(2,0);
  		cov(2,1)=temp_cov(2,1);
  		cov(2,2)=temp_cov(2,2);
  		cov(3,0)=0;
  		cov(3,1)=0;
  		cov(3,2)=0;
  		cov(0,3)=0;
  		cov(1,3)=0;
  		cov(2,3)=0;
  		cov(3,3)=1;
  		A = ((temp.matK)*cov)*((temp.matK).transpose());
  		temp.sigma(0) = sqrt(A(0,0));
  		temp.sigma(1) = sqrt(A(1,1));
  		temp.sigma(2) = sqrt(A(2,2));
  		temp.numPoints = itr->second.getPointCount();
  		temp.leafPTR = &(itr->second);
  		data.push_back(temp);
	}
}

bool isless(pcl::PointXYZ p, float M, Eigen::Vector3d a){
  if(p.x < M*a[0]){
    if(p.y < M*a[1]){
      if(p.z < M*a[2]){
        return true;
      }
    }
  }
  return false;
}


void RefineCorresSet(std::vector<std::pair<int,int>> &corres_set, std::vector<std::pair<int,int>> &corres_set_ref, pcl::VoxelGridCovariance<pcl::PointXYZ> &VGC, std::vector<VoxelData> &data, pcl::PointCloud<pcl::PointXYZ>::Ptr &globalPCL, pcl::PointCloud<pcl::PointXYZ>::Ptr &localPCL){
  int flag_number = 0;
  int flag_cov = 0;
	for(int i = 0; i<corres_set.size(); i++){
    flag_cov = 0;
    flag_number = 0;
		pcl::PointXYZ temp_point = globalPCL->points[corres_set[i].first];
    pcl::PointXYZ temp_point_loc = localPCL->points[corres_set[i].second];
    const pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf* temp_leaf = VGC.getLeaf(temp_point);
    std::vector<VoxelData>::iterator it;
    for(it = data.begin();it!=data.end();it++){
      if(temp_leaf == it->leafPTR){
        break;
      }
    }
    if(it->numPoints > MIN_POINTS){
      flag_number = 1;
    }
    if(isless(temp_point_loc, MULTIPLE, it->sigma)){
      flag_cov = 1;
    }
    if(flag_cov && flag_number){
      std::pair<int,int> temp_pair;
      temp_pair = corres_set[i];
      corres_set_ref.push_back(temp_pair);
    }
	}
}

*/



int main ()
{
  //parameters
  float th = 0;
  float th_max = 0;
  float th_min = 0;
  float iter = 10;
  g2o::Quaternion q;
  q.setIdentity();
  Eigen::Vector3d tran(0,0,0);
  float scale = 10;
  Sim3 si(q, tran, scale);

  //enter global pcd filename here 
  std::string gl_filename = "./globalPCD.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr globalPCL = loadPCL(gl_filename);
  pcl::KdTreeFLANN<pcl::PointXYZ> globalKDTree = saveAsKDTree(globalPCL);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_globalPCL;


  //enter local pcd filename here
  std::string lo_filename = "./localPCD.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr localPCL_1 = loadPCL(lo_filename);

/*
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  transform_1(0,0) = 0.0075;
  transform_1(0,1) = 0.0148;
  transform_1(0,2) = 1;
  transform_1(1,0) = -1;
  transform_1(1,1) = 0.00072;
  transform_1(1,2) = 0.0075;
  transform_1(2,0) = 0;
  transform_1(2,1) = -1;
  transform_1(2,2) = 0.0148;
  transform_1(0,3) = 0.27;
  transform_1(1,3) = -0.001;
  transform_1(2,3) = -0.07;
  transform_1(3,0) = 0;
  transform_1(3,1) = 0;
  transform_1(3,2) = 0;
  transform_1(3,3) = 1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr localPCL(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::transformPointCloud(*localPCL_1, *localPCL, transform_1); 


*/

  pcl::PointCloud<pcl::PointXYZ>::Ptr localPCL = transform_PCL(localPCL_1, si);
  pcl::io::savePCDFileASCII ("test_pcd.pcd", *localPCL);
 



  //voxelize the pointcloud
  
/*
  pcl::VoxelGridCovariance<pcl::PointXYZ> VGC;
  VGC.setInputCloud (globalPCL);
  VGC.setLeafSize (0.01f, 0.01f, 0.01f);
  VGC.filter(*filtered_globalPCL);
  const std::map<long unsigned int, pcl::VoxelGridCovariance<pcl::PointXYZ>::Leaf>& leaves = VGC.getLeaves();
  std::vector<VoxelData> data;
  doPCA(leaves, data);*/



/*
  //transform local pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_transformed_localPCL = transform_PCL(localPCL, si);

  //printPCD(globalPCL);
//	cout<<"transform 1"<<endl;
  //cout<<globalPCL->points[0]<<endl<<temp_transformed_localPCL->points[0]<<endl<<localPCL->points[0]<<endl;



  //estimating initial threshold guess
  InitialTHguess(globalKDTree, temp_transformed_localPCL, th, th_max, th_min);
  //cout<<th<<endl;
//	cout<<"estimated TH"<<endl;
  	//cout<<th_max<<endl<<th_min<<endl<<th<<endl<<endl;

  //starting the Iterative process
  Sim3 temp = si;
  for(int j =0; j<iter; j++){
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_localPCL = transform_PCL(localPCL, temp);
//	cout<<"entered loop"<<endl;
    std::vector<std::pair<int,int>> corres_set = findCorrespondences(globalKDTree, transformed_localPCL, th, th_max, th_min);
    cout<<corres_set.size()<<endl;
    std::vector<std::pair<int,int>> corres_set_ref;
    //RefineCorresSet(corres_set, corres_set_ref);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_localPCL = transform_PCL(localPCL, temp);    
    Sim3 temp1 = findSimilarityTrans(globalPCL, transformed_localPCL, corres_set);
 //	std::cout<<temp1<<std::endl;
    temp = temp * temp1;
    th = updateTH(th_max, th_min, iter, j+1);
  	//cout<<th_max<<endl<<th_min<<endl<<th<<endl<<endl;
    

//    cout<<"for loop done"<<endl<<endl;

  	std::cout<<temp<<std::endl;
  }
//cout<<globalPCL->points[60871].x<<"  "<<globalPCL->points[60871].y<<"  "<<globalPCL->points[60871].z<<endl<<endl;
//cout<<globalPCL->points[60872].x<<"  "<<globalPCL->points[60872].y<<"  "<<globalPCL->points[60872].z<<endl<<endl;
  //final transformation out is temp
*/
  return (0);
}
