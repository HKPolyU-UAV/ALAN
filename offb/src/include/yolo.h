#include "run_ncnn.hpp"

#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/PointStamped.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include "offb/obj.h"