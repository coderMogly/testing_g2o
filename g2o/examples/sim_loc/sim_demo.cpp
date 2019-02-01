/*Author Anirudh Yadav
  Similarity transformation identification demo

*/



#include <Eigen/StdVector>
#include <random>
#include <iostream>
#include <stdint.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>          

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

using namespace Eigen;
using namespace std;
using namespace g2o;

// sampling distributions
  class Sample
  {

    static default_random_engine gen_real;
    static default_random_engine gen_int;
  public:

    static double uniform();
  };


  default_random_engine Sample::gen_real;
  default_random_engine Sample::gen_int;



  double Sample::uniform()
  {
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    double sam = unif(gen_real);
    return  sam;
  }



//
// set up simulated system.
//

int main()
{


  SparseOptimizer optimizer;
  optimizer.setVerbose(false);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<BlockSolverX>(g2o::make_unique<LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));

  optimizer.setAlgorithm(solver);

  vector<Vector3d> true_points;
  for (size_t i=0;i<1000; ++i)
  {
    true_points.push_back(Vector3d((Sample::uniform()-0.5)*3,
                                   Sample::uniform()-0.5,
                                   Sample::uniform()+10));
  }


  // set up two poses
  int vertex_id = 0;
  for (size_t i=0; i<2; ++i)
  {
    // set up rotation, translation and scale for this node
    Vector3d t(0,0,0);
    g2o::Quaternion q;
    q.setIdentity();
    int scale = 1;

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

  // set up point matches
  for (size_t i=0; i<true_points.size(); ++i)
  {
    // get two poses
    VertexSim3* vp0 = 
      dynamic_cast<VertexSim3*>(optimizer.vertices().find(0)->second);
    VertexSim3* vp1 = 
      dynamic_cast<VertexSim3*>(optimizer.vertices().find(1)->second);

    // calculate the relative 3D position of the point
    Vector3d pt0,pt1;


    //sim transforming the input pointcloud

    Vector3d vic = Vector3d(10,11,12);
    Matrix3 m;
    m << 0, -1, 0 , 1 ,0 ,0, 0, 0, 1; 

    pt0 = 2*m*(vp0->estimate().inverse() * true_points[i]) + vic;
    pt1 = vp1->estimate().inverse() * true_points[i];


    // form edge
    Vector3d nm0, nm1;


    Edge_V_V_SIM * e           // new edge with correct cohort for caching
        = new Edge_V_V_SIM(); 

    e->setVertex(0, vp0);      // first viewpoint

    e->setVertex(1, vp1);      // second viewpoint


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
  }

  //VertexSim3* vc = 
  //  dynamic_cast<VertexSim3*>(optimizer.vertices().find(1)->second);
  //Sim3 cam = vc->estimate();

  
  //printing sim before calculations
  //cout<< cam<<endl;
  

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  cout << "Initial chi2 = " << FIXED(optimizer.chi2()) << endl;

  optimizer.setVerbose(true);

  optimizer.optimize(20);

  //priting sim after calculations
  VertexSim3* temp_ver = dynamic_cast<VertexSim3*>(optimizer.vertices().find(1)->second);
  Sim3 temp_sim = temp_ver->estimate();
  cout<<temp_sim<<endl;

}
