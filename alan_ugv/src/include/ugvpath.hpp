
#include <Eigen/Dense>
#include <type_traits>

#define CIRCLE_TRAJ "CIRCLE_TRAJ"
#define BLOCK_TRAJ "BLOCK_TRAJ"
#define STRAIGHT_TRAJ "STRAIGHT_TRAJ"
#define EIGHT_TRAJ "EIGHT_TRAJ"


namespace ugv
{
    typedef struct ugv_traj_info_circle
    {
        std::string traj_type;
        Eigen::Vector2d starto; 
        Eigen::Vector2d center; 
        double radius; 
        int pub_freq; 
        double velo;
    }ugv_traj_info_circle;

    typedef struct ugv_traj_info_block
    {
        std::string traj_type;
        Eigen::Vector2d starto;
        Eigen::Vector2d center; 
        double length; 
        double aspect_ratio; 
        int pub_freq;
        double velo;
    }ugv_traj_info_block;

    typedef struct ugv_traj_info_straight
    {
        std::string traj_type;
        Eigen::Vector2d starto;
        Eigen::Vector2d center;
        double radius;
        int pub_freq;
        double length;
        double aspect_ratio;
    }ugv_traj_info_straight;

    typedef struct ugv_traj_info_eight
    {
        std::string traj_type;
        Eigen::Vector2d starto;
        Eigen::Vector2d center;
        double radius;
        int pub_freq;
        double length;
        double aspect_ratio;
    }ugv_traj_info_eight;


    template <typename T>
    class ugvpath
    {
    private:
        std::string _traj_type;
        T _traj_info;

    public:

        // define overload
        // circle
        ugvpath(T traj_info) :
        _traj_info(traj_info)
        {
            std::cout<<"hiiiii"<<std::endl;
            if(std::is_same<T, ugv_traj_info_circle>::value)
            {
                std::cout<<"here"<<std::endl;

            }
            else if (std::is_same<T, ugv_traj_info_block>::value)
            {
                /* code */
            }
            else if (std::is_same<T, ugv_traj_info_straight>::value)
            {

            }
            else if (std::is_same<T, ugv_traj_info_eight>::value)
            {
                /* code */
            }
            else
            {
                std::printf("!!!PLEASE CHECK TYPENAME!!!");
            }

        };
        
        

        
        ~ugvpath(){};
};

}




