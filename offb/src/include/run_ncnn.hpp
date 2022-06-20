// #include "/home/patty/ncnn/src/benchmark.h"
#include "essential.h"
#include "./ncnn/include/ncnn/net.h"

struct Object
{
    cv::Rect_<float> rect;
    int label;
    float prob;
};

class run_ncnn
{
    char* parampath = "";
    char* binpath   = "";
    int target_size = 0;
    std::vector<Object> objects;
    ncnn::Net* cnn_local = new ncnn::Net();    

public:
    run_ncnn(char* param_input, char* bin_input, int target_size_input) 
    : parampath(param_input), binpath(bin_input), target_size(target_size_input)
    {
        this->cnn_local->opt.num_threads = 8; //You need to compile with libgomp for multi thread support
        this->cnn_local->opt.use_vulkan_compute = false; //You need to compile with libvulkan for gpu support

        this->cnn_local->opt.use_winograd_convolution = true;
        this->cnn_local->opt.use_sgemm_convolution = true;
        this->cnn_local->opt.use_fp16_packed = true;
        this->cnn_local->opt.use_fp16_storage = true;
        this->cnn_local->opt.use_fp16_arithmetic = true;
        this->cnn_local->opt.use_packing_layout = true;
        this->cnn_local->opt.use_shader_pack8 = false;
        this->cnn_local->opt.use_image_storage = false;

        int succeed = -1;

        try
        {
            succeed = this->cnn_local->load_param(this->parampath);
            succeed = this->cnn_local->load_model(this->binpath);
            if(succeed!=0)
                throw "bad doggie!";
        }
        catch(...)
        {
            cerr << "check! Initiaion fail!\n";
            exit(0);
        }

        this->cnn_local->load_param(this->parampath);
        this->cnn_local->load_model(this->binpath);

        printf("initialization succeed\n");
    };

    ~run_ncnn(){};

    void detect_yolo(const cv::Mat& bgr);
    void draw_objects(const cv::Mat& bgr, const std::vector<Object>& objects);
};

void run_ncnn::detect_yolo(const cv::Mat& bgr)
{
    int img_w = bgr.cols;
    int img_h = bgr.rows;

    cout<<this->parampath<<endl;
    cout<<this->binpath<<endl;

    cout<<"lalalala"<<this->cnn_local->opt.use_image_storage<<endl;



    ncnn::Mat in = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_BGR2RGB, bgr.cols, bgr.rows, target_size, target_size);

    const float mean_vals[3] = {0, 0, 0};
    const float norm_vals[3] = {1 / 255.f, 1 / 255.f, 1 / 255.f};
    in.substract_mean_normalize(mean_vals, norm_vals);


    ncnn::Extractor ex = cnn_local->create_extractor();

    ex.input("data", in);

    ncnn::Mat out;
    cout<<4<<endl;
    ex.extract("output", out);

    cout<<"4"<<endl;


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