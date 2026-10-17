// Host-only regression: compile together with the real postprocess.cc.
#include <user_msgs/image_orientation.h>
#include "postprocess.h"
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

struct Image { unsigned height, step; std::vector<unsigned char> data; };
struct Info { unsigned height; std::array<double,9> K; std::array<double,12> P; };

int main() {
    for (unsigned h : {1u, 2u, 3u, 480u}) {
        // Byte rows cover RGB, uint16/float depth and padded storage equally.
        for (unsigned step : {3u, 8u, 17u}) {
            Image image{h,step,{}};
            for (unsigned i=0; i<h*step; ++i) image.data.push_back(i%251);
            const auto original=image.data;
            jzzj_image::flip_vertical(image);
            for (unsigned y=0; y<h; ++y)
                for (unsigned x=0; x<step; ++x)
                    assert(image.data[y*step+x]==original[(h-1-y)*step+x]);
            jzzj_image::flip_vertical(image);
            assert(image.data==original);
        }
    }
    Image invalid{2,4,{0}};
    bool rejected=false;
    try { jzzj_image::flip_vertical(invalid); } catch(const std::invalid_argument&) { rejected=true; }
    assert(rejected);
    Info info{480,{{600,0,320,0,610,240,0,0,1}},{{600,0,320,0,0,610,240,12,0,0,1,0}}};
    const Info original=info;
    jzzj_image::flip_camera_info(info);
    assert(info.K[4]==-610 && info.K[5]==239 && info.P[7]==-12);
    const double z=0.8, y=0.13;
    assert(std::abs((info.K[4]*y/z+info.K[5]) - (479-(610*y/z+240))) < 1e-10);
    jzzj_image::flip_camera_info(info);
    assert(info.K==original.K && info.P==original.P);

    const std::vector<int> mapping{0,1,1,0,1,0};
    std::vector<int8_t> a(33*4*4,-128), b(33*2*2,-128), c(33,-128);
    auto set=[&](int anchor,int channel,int value) { a[(11*anchor+channel)*16+5]=value; };
    for (int anchor=0; anchor<3; ++anchor) {
        for (int channel=0; channel<4; ++channel) set(anchor,channel,0); // ~0.5
        set(anchor,4,127); // objectness 1
    }
    set(0,5+0,40); set(1,5+3,120); set(2,5+1,90);
    float anchors[6]={8,8,8,8,8,8};
    std::vector<int32_t> zps(3,-128); std::vector<float> scales(3,1.f/255);
    detect_result_group_t group{};
    auto decode=[&](const std::vector<int>& map) {
        return post_process(6,a.data(),b.data(),c.data(),32,32,anchors,anchors,anchors,
                            .25,.45,1,1,zps,scales,&group,map);
    };
    assert(decode({})==0 && group.count==3); // Different species/raw classes coexist.
    assert(group.results[0].class_id==3 && group.results[1].class_id==1);
    assert(decode(mapping)==0 && group.count==2);
    assert(group.results[0].class_id==0 && group.results[1].class_id==1);
    assert(group.results[0].conf>group.results[1].conf);
    // All six raw IDs are converted using the confirmed mapping.
    for (int raw=0; raw<6; ++raw) {
        std::fill(a.begin(),a.end(),-128);
        for(int channel=0; channel<4; ++channel) set(0,channel,0);
        set(0,4,127);set(0,5+raw,127);
        assert(decode(mapping)==0 && group.count==1);
        assert(group.results[0].class_id==mapping[raw]);
    }
    assert(decode({0,1})<0);
    std::fill(a.begin(),a.end(),-128);
    assert(decode(mapping)==0 && group.count==0);
    std::cout << "image orientation, calibration, six-class mapping and species NMS: PASS\n";
}
