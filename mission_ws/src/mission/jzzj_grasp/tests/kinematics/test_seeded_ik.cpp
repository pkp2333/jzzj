#include "../../src/kinematics_solver.h"
#include <tinyxml2.h>
#include <functional>
#include <iostream>
#include <limits>

namespace {
void require(bool ok, const std::string& message) {
  if (!ok) throw std::runtime_error(message);
}
KDL::Vector vector(const char* text) {
  double x=0, y=0, z=0;
  if (text) { std::istringstream s(text); s >> x >> y >> z; }
  return KDL::Vector(x,y,z);
}
struct Robot {
  KDL::Chain chain;
  std::vector<double> low,high;
  explicit Robot(const char* path) {
    tinyxml2::XMLDocument xml;
    require(xml.LoadFile(path)==tinyxml2::XML_SUCCESS,"cannot read actual arm URDF");
    for (unsigned i=1;i<=5;++i) {
      const std::string name="joint"+std::to_string(i);
      auto j=xml.FirstChildElement("robot")->FirstChildElement("joint");
      while (j && name!=j->Attribute("name")) j=j->NextSiblingElement("joint");
      require(j!=nullptr,"missing "+name);
      auto origin=j->FirstChildElement("origin");
      auto p=vector(origin->Attribute("xyz")), r=vector(origin->Attribute("rpy"));
      KDL::Frame frame(KDL::Rotation::RPY(r.x(),r.y(),r.z()),p);
      auto axis=vector(j->FirstChildElement("axis")->Attribute("xyz"));
      chain.addSegment(KDL::Segment(name,KDL::Joint(name,frame.p,frame.M*axis,KDL::Joint::RotAxis),frame));
      auto limits=j->FirstChildElement("limit");
      low.push_back(limits->DoubleAttribute("lower"));high.push_back(limits->DoubleAttribute("upper"));
    }
  }
};
void rejects(const std::function<void()>& f,const std::string& expected) {
  try { f(); } catch (const std::exception& e) {
    require(std::string(e.what()).find(expected)!=std::string::npos,e.what());return;
  }
  throw std::runtime_error("accepted invalid input: "+expected);
}
}
int main(int argc,char** argv) {
  try {
    require(argc==2,"expected URDF path");Robot robot(argv[1]);stage_c::IkPolicy policy;
    stage_c::SeededIk ik(robot.chain,robot.low,robot.high,KDL::Vector(.053,.079,-.051),policy);
    const double deg=std::acos(-1.)/180.;
    std::vector<double> seed={89*deg,-3*deg,90*deg,90*deg,-deg};
    const auto desired=ik.forward(seed);
    require((desired.p-KDL::Vector(-.08050706320083502,-.13733959295096645,.03188118670604627)).Norm()<1e-9,"FK differs from measured ARM64 baseline");
    unsigned cases=1;
    auto result=ik.solve(desired,seed);
    for(unsigned i=0;i<5;++i)require(std::abs(result[i]-seed[i])<1e-6,"same-pose solution changed measured joint");
    ++cases;
    std::vector<double> vendor={88.9999984206*deg,-3.01921233689*deg,90.0193180979*deg,89.9792241678*deg,-1.00011011113*deg};
    result=ik.validateCandidate(desired,seed,vendor);
    require(result[2]==robot.high[2],"numeric overshoot did not project to hard upper limit");++cases;
    auto bad=vendor;bad[2]=robot.high[2]+.01;
    rejects([&]{ik.validateCandidate(desired,seed,bad);},"joint3");++cases;
    auto moved=desired;moved.p.x(moved.p.x()+.1);
    rejects([&]{ik.validateCandidate(moved,seed,vendor);},"FK residual");++cases;
    moved=desired;moved.M=KDL::Rotation::RotX(.3)*moved.M;
    rejects([&]{ik.validateCandidate(moved,seed,vendor);},"FK residual");++cases;
    bad=seed;bad[0]+=.6;const auto distant=ik.forward(bad);
    rejects([&]{ik.validateCandidate(distant,seed,bad);},"continuity: joint1");++cases;
    auto unreachable=desired;unreachable.p=KDL::Vector(10,10,10);
    rejects([&]{ik.solve(unreachable,seed);},"IK");++cases;
    bad=seed;bad[1]=std::numeric_limits<double>::quiet_NaN();
    rejects([&]{ik.solve(desired,bad);},"invalid measured seed");++cases;
    bad=seed;bad[2]=robot.high[2]+.0001;
    rejects([&]{ik.solve(desired,bad);},"invalid measured seed");++cases;
    auto nan=desired;nan.p.x(std::numeric_limits<double>::quiet_NaN());
    rejects([&]{ik.solve(nan,seed);},"nonfinite target");++cases;
    bad=seed;bad[2]=std::numeric_limits<double>::quiet_NaN();
    rejects([&]{ik.validateCandidate(desired,seed,bad);},"joint3");++cases;
    nan=desired;nan.M(0,0)=std::numeric_limits<double>::quiet_NaN();
    rejects([&]{ik.solve(nan,seed);},"nonfinite target rotation");++cases;
    auto unsafe=policy;unsafe.limit_epsilon=.1;
    rejects([&]{stage_c::SeededIk s(robot.chain,robot.low,robot.high,KDL::Vector(),unsafe);},"invalid IK policy");++cases;
    std::vector<double> interior={.2,-.2,.3,.4,-.1},next=interior;next[0]+=.02;next[1]+=.01;
    auto reachable=ik.forward(next);result=ik.solve(reachable,interior);
    require((ik.forward(result).p-reachable.p).Norm()<=policy.position_residual,"reachable neighbor not reached");++cases;
    for (double q1 : {-1.,0.,1.}) for (double q2 : {-.5,0.,.5}) {
      std::vector<double> start={q1,q2,.4,.3,-.2},target=start;
      target[2]+=.01;target[3]-=.015;
      auto pose=ik.forward(target);auto solved=ik.solve(pose,start);
      for(unsigned i=0;i<5;++i)
        require(solved[i]>=robot.low[i] && solved[i]<=robot.high[i] &&
                std::abs(solved[i]-start[i])<=policy.max_joint_step,"grid solution violated limits/continuity");
      require((ik.forward(solved).p-pose.p).Norm()<=policy.position_residual,"grid target residual failed");
      ++cases;
    }
    std::cout<<"PASS "<<cases<<" actual-URDF KDL regression cases\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<"\n";return 1;}
}
