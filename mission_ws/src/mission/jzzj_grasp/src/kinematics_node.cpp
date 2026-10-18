// ROS adapter; the seeded, bounded solver has no vendor-library dependency.
#include <ros/ros.h>
#include <jzzj_grasp_interfaces/ComputeArmMotion.h>
#include "kinematics_solver.h"
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <algorithm>
#include <cmath>
#include <memory>

class Adapter {
  ros::NodeHandle nh_, pnh_{"~"};
  ros::ServiceServer service_, endpoint_service_;
  KDL::Chain chain_;
  std::unique_ptr<stage_c::SeededIk> solver_;
  std::string urdf_, frame_;
  std::vector<double> low_, high_, tcp_, grip_limits_;
  stage_c::IkPolicy policy_;
  KDL::Frame forward(const std::vector<double>& q) {
    return solver_->forward(std::vector<double>(q.begin(), q.begin()+5));
  }
  bool handle(jzzj_grasp_interfaces::ComputeArmMotion::Request& req,
              jzzj_grasp_interfaces::ComputeArmMotion::Response& res, bool endpoint=false) {
    res.valid=false;
    try {
      if(req.tcp_target.header.frame_id!=frame_) throw std::runtime_error("unexpected arm frame");
      if(req.seed.name.size()!=6 || req.seed.position.size()!=6) throw std::runtime_error("six-joint seed required");
      std::vector<double> seed(6);
      for(unsigned i=0;i<6;i++) {
        std::string name="joint"+std::to_string(i+1);
        if(std::count(req.seed.name.begin(),req.seed.name.end(),name)!=1) throw std::runtime_error("joint names must be unique");
        auto it=std::find(req.seed.name.begin(),req.seed.name.end(),name);
        seed[i]=req.seed.position.at(std::distance(req.seed.name.begin(),it));
        if(!std::isfinite(seed[i])) throw std::runtime_error("invalid seed");
      }
      // Reject invalid measured state before solving or projecting any candidate.
      forward(seed);
      if(seed[5]<grip_limits_[0]||seed[5]>grip_limits_[1]) throw std::runtime_error("gripper limits");
      std::vector<double> solution=seed;
      if(!req.forward_only) {
        auto p=req.tcp_target.pose.position;auto q=req.tcp_target.pose.orientation;
        const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
        if(!std::isfinite(norm)||std::abs(norm-1.)>1e-3 || !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))
          throw std::runtime_error("invalid TCP pose");
        KDL::Rotation rotation=KDL::Rotation::Quaternion(q.x/norm,q.y/norm,q.z/norm,q.w/norm);
        KDL::Frame desired(rotation,KDL::Vector(p.x,p.y,p.z));
        const auto arm_seed=std::vector<double>(seed.begin(),seed.begin()+5);
        const auto out=endpoint ? solver_->solveEndpoint(desired,arm_seed) : solver_->solve(desired,arm_seed);
        std::copy(out.begin(),out.end(),solution.begin());
      }
      for(unsigned i=0;i<5;i++) if(solution[i]<low_[i]||solution[i]>high_[i]) throw std::runtime_error("seed/solution limits");
      if(solution[5]<grip_limits_[0]||solution[5]>grip_limits_[1]) throw std::runtime_error("gripper limits");
      auto f=forward(solution);
      res.solution=req.seed;res.solution.position.clear();res.solution.name.clear();
      for(unsigned i=0;i<6;i++){res.solution.name.push_back("joint"+std::to_string(i+1));res.solution.position.push_back(solution[i]);}
      res.solution.header.stamp=ros::Time::now();res.tcp_pose.header=res.solution.header;res.tcp_pose.header.frame_id=frame_;
      res.tcp_pose.pose.position.x=f.p.x();res.tcp_pose.pose.position.y=f.p.y();res.tcp_pose.pose.position.z=f.p.z();
      f.M.GetQuaternion(res.tcp_pose.pose.orientation.x,res.tcp_pose.pose.orientation.y,res.tcp_pose.pose.orientation.z,res.tcp_pose.pose.orientation.w);
      res.valid=true;res.reason="ok";
    } catch(const std::exception& e){res.reason=e.what();}
    return true;
  }
  bool incremental(jzzj_grasp_interfaces::ComputeArmMotion::Request& req,
                   jzzj_grasp_interfaces::ComputeArmMotion::Response& res) {return handle(req,res,false);}
  bool endpoint(jzzj_grasp_interfaces::ComputeArmMotion::Request& req,
                jzzj_grasp_interfaces::ComputeArmMotion::Response& res) {return handle(req,res,true);}
public:
  Adapter(){
    bool calibrated=false;pnh_.param("calibration_confirmed",calibrated,false);
    if(!calibrated) throw std::runtime_error("kinematics calibration not confirmed");
    if(!pnh_.getParam("urdf",urdf_) || !pnh_.getParam("tcp_offset_m",tcp_) || tcp_.size()!=3 ||
       !pnh_.getParam("gripper_limits_rad",grip_limits_) || grip_limits_.size()!=2)
      throw std::runtime_error("explicit URDF/TCP/gripper limits required");
    pnh_.param<std::string>("arm_frame",frame_,"arm_left/arm_base_link");
    pnh_.param("max_position_residual_m",policy_.position_residual,.003);
    pnh_.param("max_orientation_residual_rad",policy_.rotation_residual,.08);
    pnh_.param("max_joint_step_rad",policy_.max_joint_step,.35);
    pnh_.param("joint_limit_epsilon_rad",policy_.limit_epsilon,.001);
    for(double value:tcp_) if(!std::isfinite(value)) throw std::runtime_error("nonfinite TCP offset");
    if(!std::isfinite(grip_limits_[0])||!std::isfinite(grip_limits_[1])||grip_limits_[0]>=grip_limits_[1])
      throw std::runtime_error("invalid calibration limits/tolerances");
    urdf::Model model;KDL::Tree tree;
    if(!model.initFile(urdf_)||!kdl_parser::treeFromUrdfModel(model,tree)||!tree.getChain("arm_base_link","link5",chain_)||chain_.getNrOfJoints()!=5)
      throw std::runtime_error("expected five joint chain arm_base_link -> link5");
    for(unsigned i=1;i<=5;i++){
      auto joint=model.getJoint("joint"+std::to_string(i));
      if(!joint||!joint->limits) throw std::runtime_error("missing URDF limits");
      low_.push_back(joint->limits->lower);high_.push_back(joint->limits->upper);
    }
    solver_.reset(new stage_c::SeededIk(chain_,low_,high_,KDL::Vector(tcp_[0],tcp_[1],tcp_[2]),policy_));
    service_=nh_.advertiseService("/stage_c/compute_arm_motion",&Adapter::incremental,this);
    endpoint_service_=nh_.advertiseService("/stage_c/compute_arm_endpoint",&Adapter::endpoint,this);
  }
};
int main(int argc,char**argv){ros::init(argc,argv,"stage_c_kinematics");try{Adapter a;ros::spin();}catch(const std::exception&e){ROS_FATAL("%s",e.what());return 1;}return 0;}
