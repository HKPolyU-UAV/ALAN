// Tencent is pleased to support the open source community by making ncnn available.
//
// Copyright (C) 2020 THL A29 Limited, a Tencent company. All rights reserved.
//
// Licensed under the BSD 3-Clause License (the "License"); you may not use this file except
// in compliance with the License. You may obtain a copy of the License at
//
// https://opensource.org/licenses/BSD-3-Clause
//
// Unless required by applicable law or agreed to in writing, software distributed
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

#include "/home/patty/ncnn/build/install/include/ncnn/net.h"
// #include "/home/patty/ncnn/src/benchmark.h"

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#if CV_MAJOR_VERSION >= 3
#include <opencv2/videoio/videoio.hpp>
#endif

#include <vector>

#include <stdio.h>

#define NCNN_PROFILING
#define YOLOV4_TINY //Using yolov4_tiny, if undef, using original yolov4


struct Object
{
    cv::Rect_<float> rect;
    int label;
    float prob;
};

class run_ncnn
{

public:
    run_ncnn()
    {
        

    };
    ~run_ncnn(){};
    void ncnn_init(ncnn::Net* yolov4, int* target_size);
    void detect_yolo(const cv::Mat& bgr, std::vector<Object>& objects, int target_size, ncnn::Net* yolov4);
    void draw_objects(const cv::Mat& bgr, const std::vector<Object>& objects);
};

void run_ncnn::ncnn_init(ncnn::Net* yolov4, int* target_size)
{
    /* --> Set the params you need for the ncnn inference <-- */

    yolov4->opt.num_threads = 4; //You need to compile with libgomp for multi thread support

    yolov4->opt.use_vulkan_compute = true; //You need to compile with libvulkan for gpu support

    yolov4->opt.use_winograd_convolution = true;
    yolov4->opt.use_sgemm_convolution = true;
    yolov4->opt.use_fp16_packed = true;
    yolov4->opt.use_fp16_storage = true;
    yolov4->opt.use_fp16_arithmetic = true;
    yolov4->opt.use_packing_layout = true;
    yolov4->opt.use_shader_pack8 = false;
    yolov4->opt.use_image_storage = false;

    /* --> End of setting params <-- */
    int ret = 0;

    // original pretrained model from https://github.com/AlexeyAB/darknet
    // the ncnn model https://drive.google.com/drive/folders/1YzILvh0SKQPS_lrb33dmGNq7aVTKPWS0?usp=sharing
    // the ncnn model https://github.com/nihui/ncnn-assets/tree/master/models


    ret = yolov4->load_param("/home/patty/alan_ws/src/alan/offb/src/include/yolo/yolov4-tiny-opt.param");
    cout << ret << endl;
    ret = yolov4->load_model("/home/patty/alan_ws/src/alan/offb/src/include/yolo/yolov4-tiny-opt.bin");
    cout << ret << endl;
    cout<<endl<<endl;
    printf("init succeed\n");


}

void run_ncnn::detect_yolo(const cv::Mat& bgr, std::vector<Object>& objects, int target_size, ncnn::Net* yolov4)
{
    int img_w = bgr.cols;
    int img_h = bgr.rows;

    ncnn::Mat in = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_BGR2RGB, bgr.cols, bgr.rows, 416, 416);

    const float mean_vals[3] = {0, 0, 0};
    const float norm_vals[3] = {1 / 255.f, 1 / 255.f, 1 / 255.f};
    in.substract_mean_normalize(mean_vals, norm_vals);

    ncnn::Extractor ex = yolov4->create_extractor();

    ex.input("data", in);

    ncnn::Mat out;
    ex.extract("output", out);

    objects.clear();
    cout<<"why"<<out.h<<endl;
    for (int i = 0; i < out.h; i++)
    {
        const float* values = out.row(i);

        Object object;
        object.label = values[0];
        object.prob = values[1];
        object.rect.x = values[2] * img_w;
        object.rect.y = values[3] * img_h;
        object.rect.width = values[4] * img_w - object.rect.x;
        object.rect.height = values[5] * img_h - object.rect.y;

        objects.push_back(object);
    }
    draw_objects(bgr, objects);
}

void run_ncnn::draw_objects(const cv::Mat& bgr, const std::vector<Object>& objects)
{
    static const char* class_names[] = {"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
        "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
        "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
        "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
        "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
        "hair drier", "toothbrush"
                                       };

    cv::Mat image = bgr.clone();
    printf("size:%i\n", objects.size());

    for (size_t i = 0; i < objects.size(); i++)
    {
        const Object& obj = objects[i];

        fprintf(stderr, "%d = %.5f at %.2f %.2f %.2f x %.2f\n", obj.label, obj.prob,
                obj.rect.x, obj.rect.y, obj.rect.width, obj.rect.height);

        cv::rectangle(image, obj.rect, cv::Scalar(255, 0, 0));

        char text[256];
        sprintf(text, "%s %.1f%%", class_names[obj.label], obj.prob * 100);

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        int x = obj.rect.x;
        int y = obj.rect.y - label_size.height - baseLine;
        if (y < 0)
            y = 0;
        if (x + label_size.width > image.cols)
            x = image.cols - label_size.width;

        cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                      cv::Scalar(255, 255, 255), -1);

        cv::putText(image, text, cv::Point(x, y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }

    cv::imshow("image", image);
    cv::waitKey(20);
}
