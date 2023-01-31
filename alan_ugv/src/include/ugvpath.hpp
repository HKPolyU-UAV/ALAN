
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
        Eigen::Vector2d center; 
        double radius; 
        int pub_freq; 
        double velo; // dg/s
        int lap;
        bool ccw;
    }ugv_traj_info_circle;

    typedef struct ugv_traj_info_block
    {
        Eigen::Vector2d center; 
        double length; 
        double aspect_ratio; 
        int pub_freq;
        double velo;
        int lap;
        bool ccw;
    }ugv_traj_info_block;

    typedef struct ugv_traj_info_straight
    {
        Eigen::Vector2d endo;
        int pub_freq;
        double velo;
    }ugv_traj_info_straight;

    typedef struct ugv_traj_info_eight
    {
        Eigen::Vector2d center;
        double center2center_distance;
        double T_angle;
        int pub_freq;
        double velo;
        int lap;
        bool ccw; // ∞ (RHS ccw true or not)
    }ugv_traj_info_eight;

    class ugvpath
    {
    private:
        int traj_i = 0;
        bool start_trajectory = false;
        std::vector<Eigen::Vector2d> trajectory;
        std::string _traj_type;

        ugv_traj_info_circle circle_traj_info;
        ugv_traj_info_block block_traj_info;
        ugv_traj_info_straight straight_traj_info;
        ugv_traj_info_eight eight_traj_info;

        void set_traj()
        {
            if(_traj_type == CIRCLE_TRAJ)
            {
                int traj_total_no = (360 / circle_traj_info.velo) * circle_traj_info.pub_freq;

                std::cout<<traj_total_no<<std::endl;

                for(int i = 0; i < circle_traj_info.lap; i++)
                {                
                    Eigen::Vector2d traj_pt_temp;
                    for(int j = 0; j < traj_total_no; j++)
                    {
                        traj_pt_temp.x() = 
                            circle_traj_info.radius * cos(j / double(traj_total_no) * 2.0 * M_PI);

                        traj_pt_temp.y() = 
                            circle_traj_info.radius * sin(j / double(traj_total_no) * 2.0 * M_PI);
                        
                        std::cout<<traj_pt_temp<<std::endl;

                        trajectory.emplace_back(traj_pt_temp);
                    }
                }     

                std::cout<<"final size: "<<trajectory.size()<<std::endl;           

            }
            else if(_traj_type == BLOCK_TRAJ)
            {

            }
            else if(_traj_type == STRAIGHT_TRAJ)
            {

            }
            else if(_traj_type == EIGHT_TRAJ)
            {

            }
            else
            {
                std::printf("PLEASE CHECK CODE...");
            }
        }

    public:

        // define overload
        
        ugvpath(            
            Eigen::Vector2d center, 
            double radius,
            int pub_freq,
            double velo,
            int lap,
            bool ccw,
            std::string traj_type = CIRCLE_TRAJ
        )  // circle
        : _traj_type(traj_type)
        {
            circle_traj_info.center = center;
            circle_traj_info.radius = radius;
            circle_traj_info.pub_freq = pub_freq;
            circle_traj_info.velo = velo;
            circle_traj_info.lap = lap;
            circle_traj_info.ccw = ccw;

            std::cout<<CIRCLE_TRAJ<<std::endl;

            set_traj();
        };

        ugvpath(            
            Eigen::Vector2d center, 
            double length,
            double aspect_ratio,
            double T_angle,
            int pub_freq,
            double velo,
            int lap,
            bool ccw,
            std::string traj_type = BLOCK_TRAJ
        )  // block
        : _traj_type(traj_type)
        {
            block_traj_info.center = center;
            block_traj_info.length = length;
            block_traj_info.aspect_ratio = aspect_ratio;
            block_traj_info.pub_freq = pub_freq;
            block_traj_info.velo = velo;
            block_traj_info.lap = lap;
            block_traj_info.ccw = ccw;

            std::cout<<BLOCK_TRAJ<<std::endl;

            set_traj();
        };

        
        ugvpath(            
            Eigen::Vector2d endo,
            int pub_freq,
            double velo,
            std::string traj_type = STRAIGHT_TRAJ
        ) // straight
        : _traj_type(traj_type)
        {
            straight_traj_info.endo = endo;
            straight_traj_info.pub_freq = pub_freq;
            straight_traj_info.velo = velo;

            std::cout<<STRAIGHT_TRAJ<<std::endl;

            set_traj();
        };

        
        ugvpath(
            Eigen::Vector2d center,
            double center2center_distance,
            double T_angle,
            int pub_freq,
            double velo,
            int lap,
            bool ccw,
            std::string traj_type = EIGHT_TRAJ
        ) // eight
        : _traj_type(traj_type)
        {
            eight_traj_info.center = center;
            eight_traj_info.center2center_distance = center2center_distance;
            eight_traj_info.T_angle = T_angle;
            eight_traj_info.pub_freq = pub_freq;
            eight_traj_info.velo = velo;
            eight_traj_info.lap = lap;
            eight_traj_info.ccw = ccw;

            std::cout<<EIGHT_TRAJ<<std::endl;

            set_traj();
        };

                        
        ~ugvpath(){};
    };  

}
