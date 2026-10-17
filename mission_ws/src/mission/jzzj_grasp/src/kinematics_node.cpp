// Only built explicitly on the target with the vendor ARM64 kinematics library.
#include <ros/ros.h>
#include <jzzj_grasp_interfaces/ComputeArmMotion.h>
#include <arm/arm_kinematics.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <algorithm>
#include <cmath>
#include <memory>

class Adapter {
  ros::NodeHandle nh_, pnh_{"~"};
  ros::ServiceServer service_;
  MyArm arm_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;
  std::string urdf_, frame_;
  std::vector<double> low_, high_, tcp_, grip_limits_;
  double residual_, rotation_residual_, max_delta_;
  KDL::Frame forward(const std::vector<double>& q) {
    KDL::JntArray joints(5);
    for(unsigned i=0;i<5;i++) joints(i)=q.at(i);
    KDL::Frame f;
    if(fk_->JntToCart(joints,f)<0) throw std::runtime_error("FK failed");
    return f*KDL::Frame(KDL::Vector(tcp_[0],tcp_[1],tcp_[2]));
  }
  bool handle(jzzj_grasp_interfaces::ComputeArmMotion::Request& req,
              jzzj_grasp_interfaces::ComputeArmMotion::Response& res) {
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
      std::vector<double> solution=seed;
      if(!req.forward_only) {
        auto p=req.tcp_target.pose.position;auto q=req.tcp_target.pose.orientation;
        const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
        if(!std::isfinite(norm)||std::abs(norm-1.)>1e-3 || !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))
          throw std::runtime_error("invalid TCP pose");
        KDL::Rotation rotation=KDL::Rotation::Quaternion(q.x,q.y,q.z,q.w);
        KDL::Frame desired(rotation,KDL::Vector(p.x,p.y,p.z));
        KDL::Frame wrist=desired*KDL::Frame(KDL::Vector(tcp_[0],tcp_[1],tcp_[2])).Inverse();
        double roll,pitch,yaw;rotation.GetRPY(roll,pitch,yaw);
        std::vector<double> xyz{wrist.p.x(),wrist.p.y(),wrist.p.z()},rpy{roll,pitch,yaw},out;
        if(!arm_.arm_getIK(urdf_,xyz,rpy,out)||out.size()<5) throw std::runtime_error("IK failed");
        for(unsigned i=0;i<5;i++) {
          if(!std::isfinite(out[i])||out[i]<low_[i]||out[i]>high_[i]||std::abs(out[i]-seed[i])>max_delta_)
            throw std::runtime_error("IK limits/continuity violation");
          solution[i]=out[i];
        }
        auto f=forward(solution);
        if((f.p-desired.p).Norm()>residual_ || KDL::diff(f.M,desired.M).Norm()>rotation_residual_)
          throw std::runtime_error("IK FK pose residual too large");
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
public:
  Adapter(){
    bool calibrated=false;pnh_.param("calibration_confirmed",calibrated,false);
    if(!calibrated) throw std::runtime_error("kinematics calibration not confirmed");
    if(!pnh_.getParam("urdf",urdf_) || !pnh_.getParam("tcp_offset_m",tcp_) || tcp_.size()!=3 ||
       !pnh_.getParam("gripper_limits_rad",grip_limits_) || grip_limits_.size()!=2)
      throw std::runtime_error("explicit URDF/TCP/gripper limits required");
    pnh_.param<std::string>("arm_frame",frame_,"arm_left/arm_base_link");
    pnh_.param("max_position_residual_m",residual_,.003);
    pnh_.param("max_orientation_residual_rad",rotation_residual_,.08);
    pnh_.param("max_joint_step_rad",max_delta_,.35);
    for(double value:tcp_) if(!std::isfinite(value)) throw std::runtime_error("nonfinite TCP offset");
    if(!std::isfinite(residual_)||residual_<=0 || !std::isfinite(rotation_residual_)||rotation_residual_<=0 ||
       !std::isfinite(max_delta_)||max_delta_<=0 || !std::isfinite(grip_limits_[0])||!std::isfinite(grip_limits_[1])||grip_limits_[0]>=grip_limits_[1])
      throw std::runtime_error("invalid calibration limits/tolerances");
    urdf::Model model;KDL::Tree tree;
    if(!model.initFile(urdf_)||!kdl_parser::treeFromUrdfModel(model,tree)||!tree.getChain("arm_base_link","link5",chain_)||chain_.getNrOfJoints()!=5)
      throw std::runtime_error("expected five joint chain arm_base_link -> link5");
    for(unsigned i=1;i<=5;i++){
      auto joint=model.getJoint("joint"+std::to_string(i));
      if(!joint||!joint->limits) throw std::runtime_error("missing URDF limits");
      low_.push_back(joint->limits->lower);high_.push_back(joint->limits->upper);
    }
    fk_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    service_=nh_.advertiseService("/stage_c/compute_arm_motion",&Adapter::handle,this);
  }
};
int main(int argc,char**argv){ros::init(argc,argv,"stage_c_kinematics");try{Adapter a;ros::spin();}catch(const std::exception&e){ROS_FATAL("%s",e.what());return 1;}return 0;}
