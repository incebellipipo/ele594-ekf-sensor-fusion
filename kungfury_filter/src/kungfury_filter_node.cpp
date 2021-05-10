#include "kungfury_filter/kungfury_filter.h"

int main(int argc, char** argv) {
   
    ros::init(argc, argv, "kungfury_filter");
    
    auto k = KungfuryFilter();
    
    ros::spin();
    
    return 0;
}