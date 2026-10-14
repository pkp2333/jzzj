#include <iostream>
#include <cassert>
#include <string.h>
#include <fstream>
#include <unordered_map>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "postprocess.h"
#include "STrack.h"


static std::vector<uint32_t> colors = {0xFF3838, 0xFF9D97, 0xFF701F, 0xFFB21D, 0xCFD231, 0x48F90A, 0x92CC17,
                                       0x3DDB86, 0x1A9334, 0x00D4BB, 0x2C99A8, 0x00C2FF, 0x344593, 0x6473FF,
                                       0x0018EC, 0x8438FF, 0x520085, 0xCB38FF, 0xFF95C8, 0xFF37C7};

const static int fontFace = 3;
// FONT_HERSHEY_SIMPLEX        = 0, //!< normal size sans-serif font
// FONT_HERSHEY_PLAIN          = 1, //!< small size sans-serif font
// FONT_HERSHEY_DUPLEX         = 2, //!< normal size sans-serif font (more complex than FONT_HERSHEY_SIMPLEX)
// FONT_HERSHEY_COMPLEX        = 3, //!< normal size serif font
// FONT_HERSHEY_TRIPLEX        = 4, //!< normal size serif font (more complex than FONT_HERSHEY_COMPLEX)
// FONT_HERSHEY_COMPLEX_SMALL  = 5, //!< smaller version of FONT_HERSHEY_COMPLEX
// FONT_HERSHEY_SCRIPT_SIMPLEX = 6, //!< hand-writing style font
// FONT_HERSHEY_SCRIPT_COMPLEX = 7, //!< more complex variant of FONT_HERSHEY_SCRIPT_SIMPLEX
// FONT_ITALIC                 = 16 //!< flag for italic font
const static float fontScale = 0.6;
const static int thickness = 2;


static void dump_tensor_attr(rknn_tensor_attr* attr) {

    std::string shape_str = attr->n_dims < 1 ? "" : std::to_string(attr->dims[0]);
    for (int i = 1; i < attr->n_dims; ++i) {
        shape_str += ", " + std::to_string(attr->dims[i]);
    }

    printf("  index=%d, name=%s, n_dims=%d, dims=[%s], n_elems=%d, size=%d, w_stride = %d, size_with_stride=%d, fmt=%s, "
           "type=%s, qnt_type=%s, "
           "zp=%d, scale=%f\n",
           attr->index, attr->name, attr->n_dims, shape_str.c_str(), attr->n_elems, attr->size, attr->w_stride,
           attr->size_with_stride, get_format_string(attr->fmt), get_type_string(attr->type),
           get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);

}


double __get_us(struct timeval t) { 

    return (t.tv_sec * 1000000 + t.tv_usec); 

}


static unsigned char* load_data(FILE* fp, size_t ofst, size_t sz) {

    unsigned char* data;
    int            ret;

    data = NULL;

    if (NULL == fp) {
        return NULL;
    }

    ret = fseek(fp, ofst, SEEK_SET);
    if (ret != 0) {
        printf("blob seek failure.\n");
        return NULL;
    }

    data = (unsigned char*)malloc(sz);
    if (data == NULL) {
        printf("buffer malloc failure.\n");
        return NULL;
    }
    ret = fread(data, 1, sz, fp);
  return data;
  
}


static unsigned char* load_model(std::string filename, int* model_size) {

    FILE*          fp;
    unsigned char* data;

    fp = fopen(const_cast<char*>(filename.c_str()), "rb");
    if (NULL == fp) {
        // printf("Open file %s failed.\n", filename);
        return NULL;
    }

    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);

    data = load_data(fp, 0, size);

    fclose(fp);

    *model_size = size;
    return data;
    
}


// Function to trim leading and trailing whitespace from a string
static inline std::string trim_leading_whitespace(const std::string& str) {
    size_t first = str.find_first_not_of(' ');
    if (std::string::npos == first) {
        return str;
    }
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last - first + 1));
}


static inline int read_labels(const std::string labels_filename, std::unordered_map<int, std::string>& labels_map, int class_num_set) {

    std::ifstream file(labels_filename);
    // Read each line of the file
    std::string line;
    int index = 0;

    std::getline(file, line);
    line = trim_leading_whitespace(line);
    int class_num = std::stoi(line);

    assert(class_num == class_num_set);
    
    index++;

    while (std::getline(file, line)) {
        // Strip the line of any leading or trailing whitespace
        line = trim_leading_whitespace(line);
        // Add the stripped line to the labels_map, using the loop index as the key
        labels_map[index-1] = line;
        index++;
    }
    // Close the file
    file.close();

    assert(class_num == labels_map.size());

    return 0;

}


// Src: https://stackoverflow.com/questions/16605967
static inline std::string to_string_with_precision(const float a_value, const int n = 2) {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}


void draw_sorted_bbox(cv::Mat& img, std::vector<STrack>& output_stracks, std::unordered_map<int, std::string>& labels_map) {
    for (unsigned long i = 0; i < output_stracks.size(); i++) {
        try {
            auto color = colors[(int)output_stracks[i].class_id % colors.size()];
            auto bgr = cv::Scalar(color & 0xFF, color >> 8 & 0xFF, color >> 16 & 0xFF);

            std::vector<float> tlwh = output_stracks[i].tlwh;
            int x1 = tlwh[0];
            int y1 = tlwh[1];
            int x2 = tlwh[0] + tlwh[2];
            int y2 = tlwh[1] + tlwh[3];

            cv::Mat obj_roi = img(cv::Rect(x1, y1, x2-x1, y2-y1)).clone();
            obj_roi = obj_roi/1.4;
            obj_roi.copyTo(img(cv::Rect(x1, y1, x2-x1, y2-y1)));

            int corner_size = std::max(std::min(x2-x1, y2-y1)/4, 3);
            cv::line(img, cv::Point(x1, y1), cv::Point(x1+corner_size, y1), bgr, 3);
            cv::line(img, cv::Point(x1, y1), cv::Point(x1, y1+corner_size), bgr, 3);

            cv::line(img, cv::Point(x2, y1), cv::Point(x2-corner_size, y1), bgr, 3);
            cv::line(img, cv::Point(x2, y1), cv::Point(x2, y1+corner_size), bgr, 3);
            
            cv::line(img, cv::Point(x1, y2), cv::Point(x1+corner_size, y2), bgr, 3);
            cv::line(img, cv::Point(x1, y2), cv::Point(x1, y2-corner_size), bgr, 3);
            
            cv::line(img, cv::Point(x2, y2), cv::Point(x2-corner_size, y2), bgr, 3);
            cv::line(img, cv::Point(x2, y2), cv::Point(x2, y2-corner_size), bgr, 3);

            std::string txt1 = "id-"+cv::format("%d", output_stracks[i].track_id);
            int baseline1 = 0;
            cv::Size textSize1 = cv::getTextSize(txt1, fontFace, fontScale, thickness, &baseline1);
            std::string txt2 = labels_map[(int)output_stracks[i].class_id] + " " + to_string_with_precision(output_stracks[i].score);
            int baseline2 = 0;
            cv::Size textSize2 = cv::getTextSize(txt2, fontFace, fontScale, thickness, &baseline2);
            
            cv::putText(img, txt1, cv::Point(x1, y1-baseline1-baseline2-textSize2.height), // cv::Point((x1+x2)/2-textSize.width/2, y2-baseline),
                        fontFace, fontScale, cv::Scalar(0xFF, 0xFF, 0xFF), thickness);
            
            
            cv::putText(img, txt2, cv::Point(x1, y1-baseline2),
                        fontFace, fontScale, cv::Scalar(0xFF, 0xFF, 0xFF), thickness);
        }
        catch (...) {}
    }
}


void draw_bbox(cv::Mat& img, detect_result_group_t& dets, std::unordered_map<int, std::string>& labels_map) {

    for (int i = 0; i < dets.count; i++) {
        try {
            std::string txt = labels_map[(int)dets.results[i].class_id] + " " + to_string_with_precision(dets.results[i].conf);
            int baseline = 0;
            cv::Size textSize = cv::getTextSize(txt, fontFace, fontScale, thickness, &baseline);
            auto color = colors[(int)dets.results[i].class_id % colors.size()];
            auto bgr = cv::Scalar(color & 0xFF, color >> 8 & 0xFF, color >> 16 & 0xFF);

            int x1 = dets.results[i].bbox.left;
            int y1 = dets.results[i].bbox.top;
            int x2 = dets.results[i].bbox.right;
            int y2 = dets.results[i].bbox.bottom;

            cv::rectangle(img, cv::Point(x1, y1), cv::Point(x2, y2), bgr, 2);
            // cv::rectangle(img, 
            //               cv::Point(x1, y1 - textSize.height - baseline - 4), 
            //               cv::Point(x1 + textSize.width, y1), 
            //               bgr, -1);
            // cv::putText(img, txt, 
            //             cv::Point(x1, y1 - baseline), 
            //             fontFace, fontScale, cv::Scalar(0xFF, 0xFF, 0xFF), thickness);

            // cv::Mat obj_roi = img(cv::Rect(x1, y1, x2-x1, y2-y1)).clone();
            // obj_roi = obj_roi/1.4;
            // obj_roi.copyTo(img(cv::Rect(x1, y1, x2-x1, y2-y1)));

            // int corner_size = std::max(std::min(x2-x1, y2-y1)/4, 3);
            // cv::line(img, cv::Point(x1, y1), cv::Point(x1+corner_size, y1), bgr, 3);
            // cv::line(img, cv::Point(x1, y1), cv::Point(x1, y1+corner_size), bgr, 3);

            // cv::line(img, cv::Point(x2, y1), cv::Point(x2-corner_size, y1), bgr, 3);
            // cv::line(img, cv::Point(x2, y1), cv::Point(x2, y1+corner_size), bgr, 3);
            
            // cv::line(img, cv::Point(x1, y2), cv::Point(x1+corner_size, y2), bgr, 3);
            // cv::line(img, cv::Point(x1, y2), cv::Point(x1, y2-corner_size), bgr, 3);
            
            // cv::line(img, cv::Point(x2, y2), cv::Point(x2-corner_size, y2), bgr, 3);
            // cv::line(img, cv::Point(x2, y2), cv::Point(x2, y2-corner_size), bgr, 3);
        }
        catch (...) {}
    }

}

int resize_rga(rga_buffer_t &src, rga_buffer_t &dst, const cv::Mat &image, cv::Mat &resized_image, const cv::Size &target_size)
{
    im_rect src_rect;
    im_rect dst_rect;
    memset(&src_rect, 0, sizeof(src_rect));
    memset(&dst_rect, 0, sizeof(dst_rect));
    size_t img_width = image.cols;
    size_t img_height = image.rows;
    // if (image.type() != CV_8UC3)
    // {
    //     printf("source image type is %d!\n", image.type());
    //     return -1;
    // }
    size_t target_width = target_size.width;
    size_t target_height = target_size.height;
    src = wrapbuffer_virtualaddr((void *)image.data, img_width, img_height, RK_FORMAT_RGB_888);
    dst = wrapbuffer_virtualaddr((void *)resized_image.data, target_width, target_height, RK_FORMAT_RGB_888);
    int ret = imcheck(src, dst, src_rect, dst_rect);
    if (IM_STATUS_NOERROR != ret)
    {
        fprintf(stderr, "rga check error! %s", imStrError((IM_STATUS)ret));
        return -1;
    }
    IM_STATUS STATUS = imresize(src, dst);
    return 0;
}
