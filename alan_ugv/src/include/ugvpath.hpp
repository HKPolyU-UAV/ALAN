
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
        double radius;
        double T_angle;
        int pub_freq;
        double velo; // dg/s
        int lap;
        bool ccw; // ∞ (RHS circle ccw true or not)
    }ugv_traj_info_eight;

    // typedef struct ugv_traj_info_lemi // leminiscate
    // {
    //     Eigen::Vector2d center;
    //     double radius;
    //     double T_angle;
    //     int pub_freq;
    //     double velo; // dg/s
    //     int lap;
    //     bool ccw; // ∞ (RHS circle ccw true or not)
    // }ugv_traj_info_lemi;

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
                            circle_traj_info.center.x() +
                                circle_traj_info.radius * 
                                cos(j / double(traj_total_no) * 2.0 * M_PI);

                        traj_pt_temp.y() = 
                            circle_traj_info.center.y() +
                                circle_traj_info.radius * 
                                sin(j / double(traj_total_no) * 2.0 * M_PI);
                        
                        // std::cout<<traj_pt_temp<<std::endl;

                        trajectory.emplace_back(traj_pt_temp);                        
                    }
                }     

                std::cout<<"final size: "<<trajectory.size()<<std::endl;  

                if(!circle_traj_info.ccw)
                {
                    std::reverse(trajectory.begin(), trajectory.end());
                }

            }
            else if(_traj_type == BLOCK_TRAJ)
            {

            }
            else if(_traj_type == STRAIGHT_TRAJ)
            {

            }
            else if(_traj_type == EIGHT_TRAJ)
            {
                Eigen::Vector2d center_rhs(
                    eight_traj_info.center.x() + eight_traj_info.radius * sqrt(2),
                    eight_traj_info.center.y() 
                );
                Eigen::Vector2d center_lhs(
                    eight_traj_info.center.x() - eight_traj_info.radius * sqrt(2),
                    eight_traj_info.center.y() 
                );;

                std::cout<<center_rhs<<std::endl;
                std::cout<<center_lhs<<std::endl;

                std::vector<Eigen::Vector2d> traj_one_lap;
                
                // #1
                std::cout<<eight_traj_info.velo<<std::endl;
                int traj_total_no = (135 / eight_traj_info.velo) * eight_traj_info.pub_freq;
                std::cout<<"hi"<<traj_total_no<<std::endl;

                Eigen::Vector2d traj_pt_temp;
                for(int j = 0; j < traj_total_no; j++)
                {
                
                    traj_pt_temp.x() = 
                        center_rhs.x() +
                            eight_traj_info.radius * 
                            cos(j / double(traj_total_no) * 2.0 * M_PI * 135 / 360);

                    traj_pt_temp.y() = 
                        center_rhs.y() +
                            eight_traj_info.radius * 
                            sin(j / double(traj_total_no) * 2.0 * M_PI * 135 / 360);
                    
                    // std::cout<<j / double(traj_total_no) * 360<<std::endl;
                    // std::cout<<traj_pt_temp<<std::endl<<std::endl;
                    
                    traj_one_lap.emplace_back(traj_pt_temp);                        
                }

                // #2       
                Eigen::Vector2d pt1 = Eigen::Vector2d(
                    center_rhs.x() + eight_traj_info.radius * cos(135.0 / 360.0 * 2.0 * M_PI),
                    center_rhs.y() + eight_traj_info.radius * sin(135.0 / 360.0 * 2.0 * M_PI)
                );

                Eigen::Vector2d pt2 = Eigen::Vector2d(
                    center_lhs.x() + eight_traj_info.radius * cos(315.0 / 360.0 * 2.0 * M_PI),
                    center_lhs.y() + eight_traj_info.radius * sin(315.0 / 360.0 * 2 * M_PI)
                );

                

                Eigen::Vector2d rhs_lhs_vector = pt2 - pt1;       

                // std::cout<<pt1<<std::endl<<pt2<<std::endl;
                // std::cout<<rhs_lhs_vector<<std::endl;

                double linear_velo = 2 * M_PI * eight_traj_info.radius * eight_traj_info.velo / 360;
                double linear_distance = rhs_lhs_vector.norm();
                
                traj_total_no = linear_distance / linear_velo * eight_traj_info.pub_freq;

                for(int i = 0; i < traj_total_no; i++)
                {
                    traj_pt_temp = pt1 + rhs_lhs_vector * i / traj_total_no;
                    // std::cout<<traj_pt_temp<<std::endl;
                    traj_one_lap.emplace_back(traj_pt_temp);
                }

                // #3 
                std::vector<Eigen::Vector2d> no3_traj;
                traj_total_no = (270 / eight_traj_info.velo) * eight_traj_info.pub_freq;
                for(int j = 0; j < traj_total_no; j++)
                {
                    traj_pt_temp.x() = 
                        center_lhs.x() +
                            eight_traj_info.radius * 
                            cos(j / double(traj_total_no) * 2.0 * M_PI * 270 / 360 + 45.0 / 360.0 * M_PI);

                    traj_pt_temp.y() = 
                        center_lhs.y() +
                            eight_traj_info.radius * 
                            sin(j / double(traj_total_no) * 2.0 * M_PI * 270 / 360 + 45.0 / 360.0 * M_PI);
                    
                    no3_traj.emplace_back(traj_pt_temp);                        
                }

                std::reverse(no3_traj.begin(), no3_traj.end());
                for(auto what : no3_traj)
                {
                    traj_one_lap.emplace_back(what);
                }


                // #4
                Eigen::Vector2d pt3 = Eigen::Vector2d(
                    center_lhs.x() + eight_traj_info.radius * cos(45.0 / 360.0 * 2.0 * M_PI),
                    center_lhs.y() + eight_traj_info.radius * sin(45.0 / 360.0 * 2.0 * M_PI)
                );

                Eigen::Vector2d pt4 = Eigen::Vector2d(
                    center_rhs.x() + eight_traj_info.radius * cos(225.0 / 360.0 * 2.0 * M_PI),
                    center_rhs.y() + eight_traj_info.radius * sin(225.0 / 360.0 * 2.0 * M_PI)
                );

                Eigen::Vector2d lhs_rhs_vector = pt4 - pt3;

                std::cout<<"hey here..."<<std::endl;
                std::cout<<pt3<<std::endl<<pt4<<std::endl;
                std::cout<<lhs_rhs_vector<<std::endl<<std::endl;

                linear_velo = 2 * M_PI * eight_traj_info.radius * eight_traj_info.velo / 360;
                linear_distance = lhs_rhs_vector.norm();
                
                traj_total_no = linear_distance / linear_velo * eight_traj_info.pub_freq;

                for(int i = 0; i < traj_total_no; i++)
                {
                    traj_pt_temp = pt3 + lhs_rhs_vector * i / traj_total_no;
                    traj_one_lap.emplace_back(traj_pt_temp);
                }

                // #5
                traj_total_no = (135 / eight_traj_info.velo) * eight_traj_info.pub_freq;
                std::cout<<std::endl<<center_rhs<<std::endl<<eight_traj_info.radius<<std::endl;
                for(int j = 0; j < traj_total_no; j++)
                {
                    traj_pt_temp.x() = 
                        center_rhs.x() +
                            eight_traj_info.radius * 
                            cos(j / double(traj_total_no) * 2.0 * M_PI * 135 / 360 + 225.0 / 360.0 * 2.0 * M_PI);

                    traj_pt_temp.y() = 
                        center_rhs.y() +
                            eight_traj_info.radius * 
                            sin(j / double(traj_total_no) * 2.0 * M_PI * 135 / 360 + 225.0 / 360.0 * 2.0 * M_PI);
                    
                    // std::cout<<traj_pt_temp<<std::endl;
                    traj_one_lap.emplace_back(traj_pt_temp);                        
                }

                // std::cout<<traj_one_lap.size()<<std::endl;

                for(int i = 0; i < eight_traj_info.lap; i++)
                    for(auto& what : traj_one_lap)
                        trajectory.emplace_back(what);

        // ugvpath(
        //     Eigen::Vector2d center,
        //     double radius,
        //     double T_angle,
        //     int pub_freq,
        //     double velo,
        //     int lap,
        //     bool ccw,
        //     std::string traj_type = EIGHT_TRAJ
        // ) // eight


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
            double radius,
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
            eight_traj_info.radius = radius;
            eight_traj_info.T_angle = T_angle;
            eight_traj_info.pub_freq = pub_freq;
            eight_traj_info.velo = velo;
            eight_traj_info.lap = lap;
            eight_traj_info.ccw = ccw;

            std::cout<<EIGHT_TRAJ<<std::endl;

            set_traj();
        };
                        
        ~ugvpath(){};

        Eigen::Vector2d get_traj_wp(int& traj_i)
        {
            if(traj_i < trajectory.size())
            {
                Eigen::Vector2d traj_now = trajectory[traj_i];
                traj_i ++;

                return traj_now;
            }
            else
            {
                return trajectory[trajectory.size() - 1];
            }
        }
    };  

}
