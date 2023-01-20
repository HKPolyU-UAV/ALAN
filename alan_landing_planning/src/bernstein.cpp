#include "include/bezier_lib/bernstein.h"

namespace alan_traj
{
    //for cube_list
    bernstein::bernstein(
    int axis_dim,
    int n_order, int m, int d_order, std::vector<double> s,
    endpt start, endpt end,
    std::vector<corridor> cube_list, 
    dynamic_constraints d_constraints
    )
    {        
        //first check each matrix
            //1. especially starto position
            //2. unify all expression

        //remember all three dimension, 
            //1. stack everything together,
            //2. calculate each individual
        //see which one is faster

        if(m == cube_list.size() && m == s.size())
        {
            std::cout<<"hi: now in bernstein..."<<std::endl;
            A_array.clear();
            ub_array.clear();
            lb_array.clear();
            MQM_array.clear();

            for(int axis_i = 0; axis_i < axis_dim; axis_i++)
            {
                endpt_cond start_local, end_local;

                start_local.p_ = start.posi(axis_i);//
                start_local.v_ = start.velo(axis_i);
                start_local.a_ = start.accl(axis_i);
                start_local.j_ = start.jerk(axis_i);

                end_local.p_ = end.posi(axis_i);
                end_local.v_ = end.velo(axis_i);
                end_local.a_ = end.accl(axis_i);
                end_local.j_ = end.jerk(axis_i);

                setAeq1D(axis_i, n_order, m, d_order, s);
                setUBeq1D(axis_i, start_local, end_local, n_order, m, d_order);//remember continuotiy
                setLBeq1D(axis_i, start_local, end_local, n_order, m, d_order);//remember continuotiy

                printf("eq matrices: pass\n");

                setAieq1D(axis_i, n_order, m, d_order, s, "CUBE");     
                setUBieq1D(axis_i, cube_list, d_constraints, n_order, m, d_order);        
                setLBieq1D(axis_i, cube_list, d_constraints, n_order, m, d_order);

                printf("ieq matriaces: pass\n");

                setA1D();
                setUB1D();
                setLB1D();

                // std::cout<<"\nhere!"<<std::endl;

                // std::cout<<A.rows()<<std::endl;
                // std::cout<<A.cols()<<std::endl;
                // std::cout<<ub.size()<<std::endl;
                // std::cout<<lb.size()<<std::endl;

                // printf("pass 3\n");
                setMQM1D(axis_i, n_order, m, d_order, s);
                // printf("pass 4\n");


                A_array.emplace_back(A);
                ub_array.emplace_back(ub);
                lb_array.emplace_back(lb);
                MQM_array.emplace_back(MQM);

                // std::cout<<"finish 1D"<<std::endl;

            }


            setMQMFinal();
            // std::cout<<1<<std::endl;
            setAFinal();
            // std::cout<<2<<std::endl;
            setUbFinal();
            // std::cout<<3<<std::endl;
            setLbFinal();
            // std::cout<<4<<std::endl;

        }
        else
        {
            ROS_ERROR("Please check input. Size does not correspond!\n");
        }

    }

    //for polyh list
    bernstein::bernstein(
    int axis_dim,
    int n_order, int m, int d_order, std::vector<double> s,
    endpt start, endpt end,
    std::vector<alan_visualization::Polyhedron> sfc_list,
    dynamic_constraints d_constraints
    )
    {        
        //first check each matrix
            //1. especially starto position
            //2. unify all expression

        //remember all three dimension, 
            //1. stack everything together,
            //2. calculate each individual
        //see which one is faster

        ROS_INFO("hi we now in POLYH...\n");

        if(m == sfc_list.size() && m == s.size())
        {
            A_sfc_eq_array.clear();
            A_sfc_ieq_dyn_array.clear();

            ub_eq_array.clear();
            ub_ieq_array.clear();

            lb_eq_array.clear();
            lb_ieq_array.clear();

            MQM_array.clear();

            for(int axis_i = 0; axis_i < axis_dim; axis_i++)
            {
                endpt_cond start_local, end_local;

                start_local.p_ = start.posi(axis_i);//
                start_local.v_ = start.velo(axis_i);
                start_local.a_ = start.accl(axis_i);
                start_local.j_ = start.jerk(axis_i);

                end_local.p_ = end.posi(axis_i);
                end_local.v_ = end.velo(axis_i);
                end_local.a_ = end.accl(axis_i);
                end_local.j_ = end.jerk(axis_i);


                //set Aeq
                setAeq1D(axis_i, n_order, m, d_order, s);
                A_sfc_eq_array.emplace_back(A_eq); 

                //set beq(U,L)
                setUBeq1D(axis_i, start_local, end_local, n_order, m, d_order);
                setLBeq1D(axis_i, start_local, end_local, n_order, m, d_order);
                ub_eq_array.emplace_back(ub_eq);
                lb_eq_array.emplace_back(lb_eq);
                
                printf("eq matrices: pass\n");                

                // std::cout<<axis_i<<" dimension:\n";
                // std::cout<<A_eq.rows()<<" "<<A_eq.cols()<<std::endl<<std::endl;
                // std::cout<<ub_eq<<std::endl<<std::endl;
                // std::cout<<lb_eq<<std::endl<<std::endl<<std::endl;

                //----------------------------------------------------------------------------------------------

                //set Aieq
                setAieq1D(axis_i, n_order, m, d_order, s, "POLYH");
                A_sfc_ieq_dyn_array.emplace_back(A_ieq);  
                
                //set bieq(U,L)
                setUBieq1D_polyh(axis_i, d_constraints, n_order, m, d_order);        
                setlBieq1D_polyh(axis_i, d_constraints, n_order, m, d_order);
                ub_ieq_array.emplace_back(ub_ieq);                
                lb_ieq_array.emplace_back(lb_ieq);

                printf("ieq matriaces: pass\n");

                // std::cout<<axis_i<<" dimension:\n\n";
                // std::cout<<A_ieq<<std::endl;
                // std::cout<<A_ieq.rows()<<" "<<A_ieq.cols()<<std::endl<<std::endl;
                
                // std::cout<<ub_ieq<<std::endl<<std::endl;
                // std::cout<<lb_ieq<<std::endl<<std::endl<<std::endl;

                //----------------------------------------------------------------------------------------------

                setMQM1D(axis_i, n_order, m, d_order, s);
                MQM_array.emplace_back(MQM);

                // std::cout<<"that's one dimension\n\n\n";
            }

            printf("\n---------------------- now set all dimension ----------------------\n");
            printf("now set Aieqsfc...:\n");

            setAieqBieqsfc(axis_dim, sfc_list, n_order, m, d_order, s);

            // std::cout<<0<<std::endl;
            setMQMFinal();
            

            std::cout<<"set final AFinal_polyh..."<<std::endl;
            setAFinal_polyh();
            setUbFinal_polyh();
            setLbFinal_polyh();
            std::cout<<"A_Final size  = "<<A_final.rows()<<" "<<A_final.cols()<<std::endl;
            std::cout<<"ub_final size = "<<ub_final.rows()<<std::endl;;

            // std::cout<<"Aieq_sfc size: "<<A_ieqsfc.rows()<<" "<<A_ieqsfc.cols()<<std::endl;
            // std::cout<<"ub_ieqsfc size: "<<ub_ieqsfc.size()<<std::endl;
            // std::cout<<"lb_ieqsfc size: "<<lb_ieqsfc.size()<<std::endl<<std::endl;
        }
        else
        {
            ROS_ERROR("Please check input. Size does not correspond!\n");
        }


    }


    bernstein::bernstein()
    {
        ROS_INFO("Bernstein bases settings for ALAN_LANDING...ready to set pre-requisite\n");
    }

    void bernstein::setAeq1D(int axis_dim, int n_order, int m, int d_order, std::vector<double> s)
    {    
        int n_cond = 0;
        int _dim = (n_order + 1) * m; //how many control points
        std::vector<double> pascal;

        //Aeq_start
        Eigen::MatrixXd Aeq_start;
        n_cond =  d_order;//p v a s -> 0 1 2 3
        Aeq_start.resize(n_cond, _dim);
        Aeq_start.setZero();

        for(int i = 0 ; i < n_cond; i++)
        {
            pascal.clear();
            pascal = pascal_triangle(i + 1);
            
            double p = permutation(n_order, n_order - i);

            for(int j = 0; j <  i + 1; j++)
            {
                Aeq_start(i, j) = p * pascal[j] * pow(s[0], 1-i);
            }
        }

        //Aeq_end
        Eigen::MatrixXd Aeq_end;
        n_cond = d_order;
        Aeq_end.resize(n_cond, _dim);
        Aeq_end.setZero();

        for(int i = 0 ; i < n_cond; i++)
        {
            pascal.clear();
            pascal = pascal_triangle(i + 1);

            double p = permutation(n_order, n_order - i);

            for(int j = 0; j <  i + 1; j++)
            {
                Aeq_end(i, _dim-1 - j) = p * pascal[j] * pow(s[s.size()-1], 1-i);
            }
        }

        //Aeq_continuity
        Eigen::MatrixXd Aeq_cont;
        n_cond = (m - 1) * d_order;
        Aeq_cont.resize(n_cond, _dim);
        Aeq_cont.setZero();

        int i_m = (m - 1) ; //how many continuity should there be? A: m - 1 points, indicator m-1 -1 
        int starto_col = 0, starto_row = 0;

        // std::cout<<Aeq_cont<<std::endl;
        // std::cout<<Aeq_cont.rows()<<std::endl;
        // std::cout<<Aeq_cont.cols()<<std::endl;
        // std::cout<<"below show matrix"<<std::endl<<std::endl;

        for(int i = 0; i < i_m; i++) //each intersection point condition
        {
            starto_col = (n_order + 1) * (i + 1) - 1;
            for(int j = 0; j < d_order; j++) // p v a continuity
            {
                starto_row = i * d_order + j;
                pascal = pascal_triangle(j + 1);

                double p = permutation(n_order, n_order - j);

                // std::cout<<starto_col<<std::endl;
                // std::cout<<starto_row<<std::endl;

                // std::cout<<"p"<<std::endl;
                // std::cout<<p<<std::endl;

                switch (j)
                {
                case 0://p 
                    Aeq_cont(starto_row, starto_col + 0) = p * pascal[0] * pow(s[i+0], 1-j);

                    Aeq_cont(starto_row, starto_col + 1) = -p * pascal[0] * pow(s[i+1], 1-j);
                    break;
                
                case 1://v
                    // std::cout<<"vel: "<<std::endl;
                    // std::cout<<p * pascal[0] * pow(s[i+0], 1-j)<<std::endl;

                    Aeq_cont(starto_row, starto_col - 1) = p * pascal[0] * pow(s[i+0], 1-j);
                    Aeq_cont(starto_row, starto_col - 0) = p * pascal[1] * pow(s[i+0], 1-j);

                    Aeq_cont(starto_row, starto_col + 1) = -p * pascal[0] * pow(s[i+1], 1-j);
                    Aeq_cont(starto_row, starto_col + 2) = -p * pascal[1] * pow(s[i+1], 1-j);
                    break;
                
                case 2://a
                    Aeq_cont(starto_row, starto_col - 2) = p * pascal[0] * pow(s[i+0], 1-j);
                    Aeq_cont(starto_row, starto_col - 1) = p * pascal[1] * pow(s[i+0], 1-j);
                    Aeq_cont(starto_row, starto_col - 0) = p * pascal[2] * pow(s[i+0], 1-j);

                    Aeq_cont(starto_row, starto_col + 1) = -p * pascal[0] * pow(s[i+1], 1-j);
                    Aeq_cont(starto_row, starto_col + 2) = -p * pascal[1] * pow(s[i+1], 1-j);
                    Aeq_cont(starto_row, starto_col + 3) = -p * pascal[2] * pow(s[i+1], 1-j);
                    break;

                case 3://j
                    Aeq_cont(starto_row, starto_col - 3) = p * pascal[0] * pow(s[i+0], 1-j);
                    Aeq_cont(starto_row, starto_col - 2) = p * pascal[1] * pow(s[i+0], 1-j);
                    Aeq_cont(starto_row, starto_col - 1) = p * pascal[2] * pow(s[i+0], 1-j);
                    Aeq_cont(starto_row, starto_col - 0) = p * pascal[3] * pow(s[i+0], 1-j);

                    Aeq_cont(starto_row, starto_col + 1) = -p * pascal[0] * pow(s[i+1], 1-j);
                    Aeq_cont(starto_row, starto_col + 2) = -p * pascal[1] * pow(s[i+1], 1-j);
                    Aeq_cont(starto_row, starto_col + 3) = -p * pascal[2] * pow(s[i+1], 1-j);
                    Aeq_cont(starto_row, starto_col + 4) = -p * pascal[3] * pow(s[i+1], 1-j);
                    break;
                
                default:
                    ROS_ERROR("Something wrong with Aeq.\nPlease re-select minimization Order: with Max. Order Snap(4)!");
                    break;
                }

            }
            
        }
        // std::cout<<"hi..."<<std::endl;
        // std::cout<<Aeq_start<<std::endl;
        // std::cout<<Aeq_end<<std::endl<<std::endl;;
        A_eq.resize(Aeq_start.rows() + Aeq_end.rows() + Aeq_cont.rows(), _dim);
        
        A_eq << Aeq_start,
                Aeq_end,
                Aeq_cont;

        // std::cout<<"Aeq in setAeq1D!:\n"<<A_eq.rows()<<std::endl<<std::endl;
    }

    void bernstein::setUBeq1D(int axis_dim, endpt_cond start, endpt_cond end, int n_order, int m, int d_order)
    {
        // std::cout<<"setUBeq1D"<<std::endl;
        //set BUeq_start/end
        Eigen::VectorXd BUeq_start;
        Eigen::VectorXd BUeq_end;
        
        switch (d_order)
        {
        case 2:
            BUeq_start.resize(2);
            
            BUeq_start(0) = start.p_;
            BUeq_start(1) = start.v_;
            
            BUeq_end.resize(2);

            BUeq_end(0) = end.p_;
            BUeq_end(1) = end.v_;

            break;
        case 3:
            BUeq_start.resize(3);
            
            BUeq_start(0) = start.p_;
            BUeq_start(1) = start.v_;
            BUeq_start(2) = start.a_;
            
            BUeq_end.resize(3);

            BUeq_end(0) = end.p_;
            BUeq_end(1) = end.v_;
            BUeq_end(2) = end.a_;

            break;

        case 4:
            BUeq_start.resize(4);
            
            BUeq_start(0) = start.p_;
            BUeq_start(1) = start.v_;
            BUeq_start(2) = start.a_;
            BUeq_start(3) = start.j_;
            
            BUeq_end.resize(4);

            BUeq_end(0) = end.p_;
            BUeq_end(1) = end.v_;
            BUeq_end(2) = end.a_;
            BUeq_end(3) = end.j_;
            break;
        
        default:
            break;
        }
        

        //set BUeq_cont
        Eigen::VectorXd BUeq_cont;
        BUeq_cont.resize((m - 1) * d_order);

        BUeq_cont.setZero();

        //combine BUeq_start, BU_end, BU_cont
        ub_eq.resize(BUeq_start.size() + BUeq_end.size() + BUeq_cont.size());
        ub_eq << BUeq_start,
                BUeq_end,
                BUeq_cont;
        
        // std::cout<<"ub_eq!\n"<<ub_eq<<std::endl;;

    }

    void bernstein::setLBeq1D(int axis_dim, endpt_cond start, endpt_cond end, int n_order, int m, int d_order)
    {
        // std::cout<<"setLBeq"<<std::endl;

        //set BLeq_start/end
        Eigen::VectorXd BLeq_start;
        Eigen::VectorXd BLeq_end;
        
        switch (d_order)
        {
        case 2:
            BLeq_start.resize(2);
            
            BLeq_start(0) = start.p_;
            BLeq_start(1) = start.v_;
            
            BLeq_end.resize(2);

            BLeq_end(0) = end.p_;
            BLeq_end(1) = end.v_;

            break;
        case 3:
            BLeq_start.resize(3);
            
            BLeq_start(0) = start.p_;
            BLeq_start(1) = start.v_;
            BLeq_start(2) = start.a_;
            
            BLeq_end.resize(3);

            BLeq_end(0) = end.p_;
            BLeq_end(1) = end.v_;
            BLeq_end(2) = end.a_;

            break;

        case 4:
            BLeq_start.resize(4);
            
            BLeq_start(0) = start.p_;
            BLeq_start(1) = start.v_;
            BLeq_start(2) = start.a_;
            BLeq_start(3) = start.j_;
            
            BLeq_end.resize(4);

            BLeq_end(0) = end.p_;
            BLeq_end(1) = end.v_;
            BLeq_end(2) = end.a_;
            BLeq_end(3) = end.j_;
            break;
        
        default:
            break;
        }

        //set BLeq_cont
        Eigen::VectorXd BLeq_cont;

        BLeq_cont.resize((m - 1) * d_order);
        BLeq_cont.setZero();

        //combine BLeq_start, BLeq_end, BLeq_cont
        lb_eq.resize(BLeq_start.size() + BLeq_end.size() + BLeq_cont.size());
        lb_eq << BLeq_start,
                BLeq_end,
                BLeq_cont;

        // std::cout<<"ub_eq!\n"<<lb_eq<<std::endl;;
    }

    void bernstein::setAieq1D(int axis_dim, int n_order, int m, int d_order, std::vector<double> s, std::string corridor_type)
    {
        int _dim_crtl_pts = m * (n_order + 1);
        int _dim_p = m * (n_order + 1 - 0);
        int _dim_v = m * (n_order + 1 - 1);
        int _dim_a = m * (n_order + 1 - 2);
        int _dim_j = m * (n_order + 1 - 3);

        //Aieq_p
        Eigen::MatrixXd Aieq_p;
        Aieq_p.resize(_dim_p, _dim_crtl_pts);
        Aieq_p.setIdentity();
        
        int starto_row_col = 0;
        
        for(int i = 0; i < s.size(); i++)
        {
            starto_row_col = i * (n_order + 1);   
            // std::cout<<"starto_row_col = "<<starto_row_col  <<std::endl;

            Aieq_p.block(starto_row_col, starto_row_col, (n_order + 1), (n_order + 1)) = Aieq_p.block(starto_row_col, starto_row_col, (n_order + 1), (n_order + 1)) * pow(s[i], 1);
        }

        // std::cout<<"should be fine..."<<std::endl;

        //Aieq_v
        Eigen::MatrixXd Aieq_v;
        Aieq_v.resize(_dim_v, _dim_crtl_pts);
        Aieq_v.setZero();

        // std::cout<<Aieq_v<<std::endl;
        // std::cout<<Aieq_v.rows()<<std::endl;
        // std::cout<<Aieq_v.cols()<<std::endl<<std::endl;;
        
        int i_m = m;//each segment velocity
        int i_v_seg = n_order + 1 - 1;

        std::vector<double> pascal = pascal_triangle(2);
        double p = permutation(n_order, n_order - 1);
        int starto_col = 0, starto_row = 0;

        for(int i = 0; i < i_m; i++)
        {
            starto_col = (n_order + 1 - 0) * i;
            starto_row = (n_order + 1 - 1) * i;

            for(int j = 0; j < i_v_seg; j++)
            {
                Aieq_v(starto_row + j, starto_col + j + 0) = p * pascal[0] * pow(s[i], 0);
                Aieq_v(starto_row + j, starto_col + j + 1) = p * pascal[1] * pow(s[i], 0);
            }
        }
        pascal.clear();

        // std::cout<<Aieq_v<<std::endl;

        // std::cout<<"that's Aieq_v"<<std::endl;

        //Aieq_a
        Eigen::MatrixXd Aieq_a;
        Aieq_a.resize(_dim_a, _dim_crtl_pts);
        Aieq_a.setZero();

        i_m = m;
        int i_a_seg = n_order + 1 - 2;

        pascal = pascal_triangle(3);
        p = permutation(n_order, n_order - 2);
        starto_col = 0;
        starto_row = 0;

        for (int i = 0; i < i_m; i++)
        {
            starto_col = (n_order + 1 - 0) * i;
            starto_row = (n_order + 1 - 2) * i;

            for(int j = 0; j < i_a_seg; j++)
            {
                Aieq_a(starto_row + j, starto_col + j + 0) = p * pascal[0] * pow(s[i], -1);
                Aieq_a(starto_row + j, starto_col + j + 1) = p * pascal[1] * pow(s[i], -1);
                Aieq_a(starto_row + j, starto_col + j + 2) = p * pascal[2] * pow(s[i], -1);
            }
            
        }
        // std::cout<<Aieq_a<<std::endl;

        // std::cout<<"that's Aieq_a"<<std::endl;

        //Aieq_j
        Eigen::MatrixXd Aieq_j;
        Aieq_j.resize(_dim_j, _dim_crtl_pts);
        Aieq_j.setZero();


        i_m = m;
        int i_j_seg = n_order + 1 - 3;

        pascal = pascal_triangle(4);
        p = permutation(n_order, n_order - 3);
        starto_col = 0;
        starto_row = 0;

        for(int i = 0; i < i_m; i++)
        {
            starto_col = (n_order + 1 - 0) * i;
            starto_row = (n_order + 1 - 3) * i;

            for(int j = 0; j < i_j_seg; j++)
            {
                Aieq_j(starto_row + j, starto_col + j + 0) = p * pascal[0] * pow(s[i], -2);
                Aieq_j(starto_row + j, starto_col + j + 1) = p * pascal[1] * pow(s[i], -2);
                Aieq_j(starto_row + j, starto_col + j + 2) = p * pascal[2] * pow(s[i], -2);
                Aieq_j(starto_row + j, starto_col + j + 3) = p * pascal[3] * pow(s[i], -2);

            }

        }
        // std::cout<<Aieq_j<<std::endl;

        // std::cout<<"that's Aieq_j"<<std::endl;

        //combine
        if(corridor_type == "CUBE")
        {
            switch (d_order)
            {
            case 2:
                A_ieq.resize(Aieq_p.rows() + Aieq_v.rows(), _dim_crtl_pts);
                A_ieq << Aieq_p,
                        Aieq_v;
                break;

            case 3:
                A_ieq.resize(Aieq_p.rows() + Aieq_v.rows() + Aieq_a.rows(), _dim_crtl_pts);
                A_ieq << Aieq_p,
                        Aieq_v, 
                        Aieq_a;
                break;
            
            case 4:
                A_ieq.resize(Aieq_p.rows() + Aieq_v.rows() + Aieq_a.rows() + Aieq_j.rows(), _dim_crtl_pts);
                A_ieq << Aieq_p,
                        Aieq_v,
                        Aieq_a,
                        Aieq_j;
                break;
                
            default:
                ROS_ERROR("Something wrong with Aieq.\nPlease re-select d_order:\n\t2 for min. accl,\n\t 3 for min. jerk,\n\t 4 for min. snap\n");
                break;
            }
        }
        else if(corridor_type == "POLYH")
        {
            switch (d_order)
            {
            case 2:
                A_ieq.resize(Aieq_v.rows(), _dim_crtl_pts);
                A_ieq << Aieq_v;

                break;

            case 3:
                A_ieq.resize(Aieq_v.rows() + Aieq_a.rows(), _dim_crtl_pts);
                A_ieq << Aieq_v, 
                        Aieq_a;

                break;
            
            case 4:
                A_ieq.resize(Aieq_v.rows() + Aieq_a.rows() + Aieq_j.rows(), _dim_crtl_pts);
                A_ieq << Aieq_v,
                        Aieq_a,
                        Aieq_j;

                break;
                
            default:
                ROS_ERROR("Something wrong with Aieq.\nPlease re-select d_order:\n\t2 for min. accl,\n\t 3 for min. jerk,\n\t 4 for min. snap\n");
                break;
        }

        }
        else
        {
            ROS_ERROR("CORRIDOR TYPE NOT YET SPECIFIED...PLEASE CHECK!");
        }

        

        // std::cout<<"Aieq in setAieq1D !:\n"<<A_ieq.rows()<<std::endl<<std::endl;;

    }

    void bernstein::setUBieq1D(int axis_dim, std::vector<corridor> cube_list, dynamic_constraints d_constraints, int n_order, int m, int d_order)
    {
        std::cout<<"setUBieq1D"<<std::endl;
        // for(auto what : cube_list)
        // {
        //     std::cout<<"corridor one"<<std::endl;
        //     std::cout<<what.p_max<<std::endl;
        //     std::cout<<what.p_min<<std::endl;

            
        // }
        //UBieq_p
        Eigen::VectorXd UBieq_p;
        int _dim_p = m * (n_order + 1 - 0);
        UBieq_p.resize(_dim_p);
        

        int starto = 0;
        for(int i = 0; i < cube_list.size(); i++)//cube_list.size() = size of corridor = m
        {
            for(int j = 0; j < n_order + 1; j++) //each control point, n_order + 1 = size of ctrl_pts per segment
            {
                int _i = starto + j;
                // std::cout<<"what's the big deal?: "<<axis_dim - 1<<std::endl;
                UBieq_p(_i) = cube_list[i].p_max(axis_dim);//remember other dimension
            }
            
            starto = starto + (n_order + 1);
        }

        std::cout<<"seUBieq_v"<<std::endl;


        //UBieq_v
        Eigen::VectorXd UBieq_v;
        int _dim_v = m * (n_order + 1 - 1);
        UBieq_v.resize(_dim_v);

        for(int i = 0; i < _dim_v; i++)
            UBieq_v(i) = d_constraints.v_max(axis_dim);


        //UBiep_a
        Eigen::VectorXd UBieq_a;
        int _dim_a = m * (n_order + 1 - 2);
        UBieq_a.resize(_dim_a);

        for(int i = 0; i < _dim_a; i++)
            UBieq_a(i) = d_constraints.a_max(axis_dim);

        //UBieq_j
        Eigen::VectorXd UBieq_j;
        int _dim_j = m * (n_order + 1 - 3);
        UBieq_j.resize(_dim_j);
        
        for(int i = 0; i < _dim_j; i++)
            UBieq_j(i) = d_constraints.j_max(axis_dim);

        
        switch (d_order)
        {
        case 2:
            ub_ieq.resize(UBieq_p.size() + UBieq_v.size());
            ub_ieq << UBieq_p,
                    UBieq_v;
            break;
        
        case 3:
            ub_ieq.resize(UBieq_p.size() + UBieq_v.size() + UBieq_a.size());
            ub_ieq << UBieq_p,
                    UBieq_v,
                    UBieq_a;
            break;
        
        case 4:
            ub_ieq.resize(UBieq_p.size() + UBieq_v.size() + UBieq_a.size() + UBieq_j.size());
            ub_ieq << UBieq_p,
                    UBieq_v,
                    UBieq_a,
                    UBieq_j;
            break;
        
        default:
            ROS_ERROR("Something wrong with UB.\nRe-Select d_order:\n\t2 for min. accl,\n\t 3 for min. jerk,\n\t 4 for min. snap\n");
            break;
        }
    }

    void bernstein::setLBieq1D(int axis_dim, std::vector<corridor> cube_list, dynamic_constraints d_constraints, int n_order, int m, int d_order)
    {
        Eigen::VectorXd LBieq_p;
        int _dim_p = m * (n_order + 1 - 0);
        LBieq_p.resize(_dim_p);

        int starto = 0;
        for(int i = 0; i < cube_list.size(); i++)//cube_list.size() = size of corridor = m
        {
            for(int j = 0; j < n_order + 1; j++) //each control point, n_order + 1 = size of ctrl_pts per segment
            {
                int _i = starto + j;
                LBieq_p(_i) = cube_list[i].p_min(axis_dim);//remember other dimension
            }
            
            starto = starto + (n_order + 1);
        }
            
        
        Eigen::VectorXd LBieq_v;
        int _dim_v = m * (n_order + 1 - 1);
        LBieq_v.resize(_dim_v);

        for(int i = 0; i < _dim_v; i++)
            LBieq_v(i) = d_constraints.v_min(axis_dim);


        Eigen::VectorXd LBieq_a;
        int _dim_a = m * (n_order + 1 - 2);
        LBieq_a.resize(_dim_a);

        for(int i = 0; i < _dim_a; i++)
            LBieq_a(i) = d_constraints.a_min(axis_dim);

        
        Eigen::VectorXd LBieq_j;
        int _dim_j = m * (n_order + 1 - 3);
        LBieq_j.resize(_dim_j);
        
        for(int i = 0; i < _dim_j; i++)
            LBieq_j(i) = d_constraints.j_min(axis_dim);

        
        switch (d_order)
        {
        case 2:
            lb_ieq.resize(LBieq_p.size() + LBieq_v.size());
            lb_ieq << LBieq_p,
                    LBieq_v;
            break;
        
        case 3:
            lb_ieq.resize(LBieq_p.size() + LBieq_v.size() + LBieq_a.size());
            lb_ieq << LBieq_p,
                    LBieq_v,
                    LBieq_a;
            break;
        
        case 4:
            lb_ieq.resize(LBieq_p.size() + LBieq_v.size() + LBieq_a.size() + LBieq_j.size());
            lb_ieq << LBieq_p,
                    LBieq_v,
                    LBieq_a,
                    LBieq_j;
            break;
        
        default:
            ROS_ERROR("Something wrong with LB.\nRe-Select d_order:\n\t2 for min. accl,\n\t 3 for min. jerk,\n\t 4 for min. snap\n");
            break;
        }


    }

    void bernstein::setUBieq1D_polyh(int axis_dim, dynamic_constraints d_constraints, int n_order, int m, int d_order)
    {
        // std::cout<<"setUBieq1D"<<std::endl;
        

        // std::cout<<"seUBieq_v"<<std::endl;


        //UBieq_v
        Eigen::VectorXd UBieq_v;
        int _dim_v = m * (n_order + 1 - 1);
        UBieq_v.resize(_dim_v);

        for(int i = 0; i < _dim_v; i++)
            UBieq_v(i) = d_constraints.v_max(axis_dim);


        //UBiep_a
        Eigen::VectorXd UBieq_a;
        int _dim_a = m * (n_order + 1 - 2);
        UBieq_a.resize(_dim_a);

        for(int i = 0; i < _dim_a; i++)
            UBieq_a(i) = d_constraints.a_max(axis_dim);

        //UBieq_j
        Eigen::VectorXd UBieq_j;
        int _dim_j = m * (n_order + 1 - 3);
        UBieq_j.resize(_dim_j);
        
        for(int i = 0; i < _dim_j; i++)
            UBieq_j(i) = d_constraints.j_max(axis_dim);

        
        switch (d_order)
        {
        case 2:
            ub_ieq.resize(UBieq_v.size());
            ub_ieq << UBieq_v;
            break;
        
        case 3:
            ub_ieq.resize(UBieq_v.size() + UBieq_a.size());
            ub_ieq << UBieq_v,
                    UBieq_a;
            break;
        
        case 4:
            ub_ieq.resize(UBieq_v.size() + UBieq_a.size() + UBieq_j.size());
            ub_ieq << UBieq_v,
                    UBieq_a,
                    UBieq_j;
            break;
        
        default:
            ROS_ERROR("Something wrong with UB.\nRe-Select d_order:\n\t2 for min. accl,\n\t 3 for min. jerk,\n\t 4 for min. snap\n");
            break;
        }
    }

    void bernstein::setlBieq1D_polyh(int axis_dim, dynamic_constraints d_constraints, int n_order, int m, int d_order)
    {                    
        Eigen::VectorXd LBieq_v;
        int _dim_v = m * (n_order + 1 - 1);
        LBieq_v.resize(_dim_v);

        for(int i = 0; i < _dim_v; i++)
            LBieq_v(i) = d_constraints.v_min(axis_dim);


        Eigen::VectorXd LBieq_a;
        int _dim_a = m * (n_order + 1 - 2);
        LBieq_a.resize(_dim_a);

        for(int i = 0; i < _dim_a; i++)
            LBieq_a(i) = d_constraints.a_min(axis_dim);

        
        Eigen::VectorXd LBieq_j;
        int _dim_j = m * (n_order + 1 - 3);
        LBieq_j.resize(_dim_j);
        
        for(int i = 0; i < _dim_j; i++)
            LBieq_j(i) = d_constraints.j_min(axis_dim);

        
        switch (d_order)
        {
        case 2:
            lb_ieq.resize(LBieq_v.size());
            lb_ieq << LBieq_v;
            break;
        
        case 3:
            lb_ieq.resize(LBieq_v.size() + LBieq_a.size());
            lb_ieq << LBieq_v,
                    LBieq_a;
            break;
        
        case 4:
            lb_ieq.resize(LBieq_v.size() + LBieq_a.size() + LBieq_j.size());
            lb_ieq << LBieq_v,
                    LBieq_a,
                    LBieq_j;
            break;
        
        default:
            ROS_ERROR("Something wrong with LB.\nRe-Select d_order:\n\t2 for min. accl,\n\t 3 for min. jerk,\n\t 4 for min. snap\n");
            break;
        }


    }

    void bernstein::setMQM1D(int axis_dim, int n_order, int m, int d_order, std::vector<double> s)
    {
        setQM1D(axis_dim, n_order, m, d_order, s);
        
        MQM.resize(Q.rows(), Q.cols());
        
        // MQM =  Q * M;

        MQM = M.transpose() * Q * M;


        // std::cout<<Q.rows()<<std::endl;
        // std::cout<<Q.cols()<<std::endl;

        // std::cout<<M.rows()<<std::endl;
        // std::cout<<M.cols()<<std::endl;
        
        // std::cout<<Q<<std::endl;
        // std::cout<<M<<std::endl;
        // std::cout<<MQM<<std::endl;
        // MQM_spd = getSPD(MQM);

    }

    void bernstein::setQM1D(int axis_dim, int n_order, int m, int d_order, std::vector<double> s)
    {
        int n_dim = (n_order + 1) * m;

        Q.resize(n_dim, n_dim);
        Q.setZero();
        M.resize(n_dim, n_dim);
        M.setZero();
        // std::cout<<Q.rows()<<std::endl;
        // std::cout<<Q.cols()<<std::endl;

        Q_temp.resize(n_order + 1, n_order + 1);
        Q_temp.setZero();
        M_temp.resize(n_order + 1, n_order + 1);
        M_temp.setZero();
        int starto = 0;
        
        for(int t = 0; t < m; t++)
        {        
            starto = t * (n_order + 1);
            for(int i = 0; i < Q_temp.rows(); i++)
            {
                for(int j = 0; j < Q_temp.cols(); j++)
                {
                    if(i < 4 || j < 4)
                        Q_temp(i, j) = 0;
                    else
                        Q_temp(i, j) = i * (i-1) * (i-2) * (i-3)
                                    * j * (j-1) * (j-2) * (j-3)
                                    * pow(s[t], (-2 * d_order + 3))  
                                    / (i+j-7);
                }
            }

            setM1D(axis_dim, n_order);
            M.block(starto, starto, n_order + 1, n_order + 1) = M_temp;

            Q.block(starto, starto, n_order + 1, n_order + 1) = Q_temp;
            
            // std::cout<<"that's one"<<std::endl;
            // std::cout<<Q<<std::endl;
        }

    }

    void bernstein::setA1D()
    {
        //combine A_eq && A_ieq
        A.resize(A_eq.rows() + A_ieq.rows() , A_eq.cols());
        //+ A_ieq.rows()
        //A_eq.rows() + 

        A << A_eq,
            A_ieq;//,
            //  A_ieq;

        std::cout<<"A in setA1D!:\n"<<A.rows()<<std::endl<<std::endl;

    }

    void bernstein::setUB1D()
    {
        //combine ub_eq && ub_ieq
        ub.resize( ub_eq.size()  + ub_ieq.size());
        //ub_eq.size() +
        ub << ub_eq,
            ub_ieq;//,
            //   ub_ieq;

        // std::cout<<"ub!:\n"<<ub<<std::endl;

    }

    void bernstein::setLB1D()
    {
        //combine lb_eq && lb_ieq
        lb.resize( lb_eq.size() + lb_ieq.size());
        //lb_eq.size() +
        lb << lb_eq, 
            lb_ieq;//,
            //   lb_ieq;
            //

        // std::cout<<"lb!:\n"<<lb<<std::endl;

    };

    void bernstein::setM1D(int axis_dim, int order)
    {
        M_temp.resize(order + 1, order + 1);
        switch (order)
        {
        case 0: 
        {
            M_temp << 1;
            break;

        }
        case 1: 
        {
            M_temp << -1,  0,
                    -1,  1;
            break;

        }
        case 2:
        {
            M_temp << -1,  0,  0,
                    -2,  2,  0,
                    1, -2,  1;
            break;

        }
        case 3: 
        {
            M_temp << -1,  0,  0,  0,
                    -3,  3,  0,  0,
                    3, -6,  3,  0,
                    -1,  3, -3,  1;	
            break;

        }
        case 4:
        {
            M_temp <<  1,   0,   0,   0,  0,
                    -4,   4,   0,   0,  0,
                    6, -12,   6,   0,  0,
                    -4,  12, -12,   4,  0,
                    1,  -4,   6,  -4,  1;
            break;
        }
        case 5:
        {
            M_temp << 1,   0,   0,   0,  0,  0,
                -5,   5,   0,   0,  0,  0,
                10, -20,  10,   0,  0,  0,
                -10,  30, -30,  10,  0,  0,
                    5, -20,  30, -20,  5,  0,
                -1,   5, -10,  10, -5,  1;
            break;
        }
        case 6:
        {	

            M_temp << 1,   0,   0,   0,   0,  0,  0,
                -6,   6,   0,   0,   0,  0,  0,
                15, -30,  15,   0,   0,  0,  0,
                -20,  60, -60,  20,   0,  0,  0,
                15, -60,  90, -60,  15,  0,  0,
                -6,  30, -60,  60, -30,  6,  0,
                    1,  -6,  15, -20,  15, -6,  1;
            break;
        }
        case 7:
        {
            M_temp << 1,    0,    0,    0,    0,   0,   0,   0,
                -7,    7,    0,    0,    0,   0,   0,   0,
                21,   42,   21,    0,    0,   0,   0,   0,
                -35,  105, -105,   35,    0,   0,   0,   0, 
                35, -140,  210, -140,   35,   0,   0,   0,
                -21,  105, -210,  210, -105,  21,   0,   0,
                    7,  -42,  105, -140,  105, -42,   7,   0,
                -1,    7,  -21,   35,  -35,  21,  -7,   1;
            break;
        }
        case 8:
        {
            M_temp << 1,    0,    0,    0,    0,    0,   0,   0,   0,
                -8,    8,    0,    0,    0,    0,   0,   0,   0,
                28,  -56,   28,    0,    0,    0,   0,   0,   0,
                -56,  168, -168,   56,    0,    0,   0,   0,   0, 
                70, -280,  420, -280,   70,    0,   0,   0,   0,
                -56,  280, -560,  560, -280,   56,   0,   0,   0,
                28, -168,  420, -560,  420, -168,  28,   0,   0,
                -8,   56, -168,  280, -280,  168, -56,   8,   0,
                    1,   -8,   28,  -56,   70,  -56,  28,  -8,   1;
            break;
        }
        case 9:
        {
            M_temp << 1,    0,     0,     0,     0,    0,    0,     0,     0,    0,
                -9,    9,     0,     0,     0,    0,    0,     0,     0,    0, 
                36,  -72,    36,     0,     0,    0,    0,     0,     0,    0, 
                -84,  252,  -252,    84,     0,    0,    0,     0,     0,    0, 
                126, -504,   756,  -504,   126,    0,    0,     0,     0,    0,
                -126,  630, -1260,  1260,  -630,  126,    0,     0,     0,    0,
                84, -504,  1260, -1680,  1260, -504,   84,     0,     0,    0,
                -36,  252,  -756,  1260, -1260,  756, -252,    36,     0,    0,
                    9,  -72,   252,  -504,   630, -504,  252,   -72,     9,    0,
                -1,    9,   -36,    84,  -126,  126,  -84,    36,    -9,    1;
            break;
        }
        case 10:
        {
            M_temp <<  1,     0,     0,     0,      0,     0,    0,     0,     0,    0,   0,
                -10,    10,     0,     0,      0,     0,    0,     0,     0,    0,   0,
                    45,   -90,    45,     0,      0,     0,    0,     0,     0,    0,   0,
                -120,   360,  -360,   120,      0,     0,    0,     0,     0,    0,   0,
                210,  -840,  1260,  -840,    210,     0,    0,     0,     0,    0,   0,
                -252,  1260, -2520,  2520,  -1260,   252,    0,     0,     0,    0,   0,
                210, -1260,  3150, -4200,   3150, -1260,  210,     0,     0,    0,   0,
                -120,  840,  -2520,  4200,  -4200,  2520, -840,   120,     0,    0,   0,
                    45, -360,   1260, -2520,   3150, -2520, 1260,  -360,    45,    0,   0,
                -10,   90,   -360,   840,  -1260,  1260, -840,   360,   -90,   10,   0,
                    1,  -10,     45,  -120,    210,  -252,  210,  -120,    45,  -10,   1;
            break;
        }
        case 11:
        {
            M_temp <<  1,     0,    0,      0,      0,      0,     0,     0,     0,    0,   0,  0,
                -11,    11,    0,      0,      0,      0,     0,     0,     0,    0,   0,  0,
                    55,  -110,   55,      0,      0,      0,     0,     0,     0,    0,   0,  0,
                -165,   495, -495,    165,      0,      0,     0,     0,     0,    0,   0,  0,
                330, -1320, 1980,  -1320,    330,      0,     0,     0,     0,    0,   0,  0,
                -462,  2310, -4620,  4620,  -2310,    462,     0,     0,     0,    0,   0,  0,
                462, -2772,  6930, -9240,   6930,  -2772,   462,     0,     0,    0,   0,  0,
                -330,  2310, -6930, 11550, -11550,   6930, -2310,   330,     0,    0,   0,  0,
                165, -1320,  4620, -9240,  11550,  -9240,  4620, -1320,   165,    0,   0,  0,
                -55,   495, -1980,  4620,  -6930,   6930, -4620,  1980,  -495,   55,   0,  0,
                    11,  -110,   495, -1320,   2310,  -2772,  2310, -1320,   495, -110,  11,  0,
                    -1,    11,   -55,   165,   -330,    462,  -462,   330,  -165,   55, -11,  1;
            break;
        }
        case 12:
        {
            M_temp <<  1,     0,     0,      0,      0,      0,     0,     0,     0,    0,    0,   0,   0,
                -12,    12,     0,      0,      0,      0,     0,     0,     0,    0,    0,   0,   0,
                    66,  -132,    66,      0,      0,      0,     0,     0,     0,    0,    0,   0,   0,
                -220,   660,  -660,    220,      0,      0,     0,     0,     0,    0,    0,   0,   0,
                495, -1980,  2970,  -1980,    495,      0,     0,     0,     0,    0,    0,   0,   0, 
                -792,  3960, -7920,   7920,  -3960,    792,     0,     0,     0,    0,    0,   0,   0,
                924, -5544, 13860, -18480,  13860,  -5544,   924,     0,     0,    0,    0,   0,   0,
                -792,  5544,-16632,  27720, -27720,  16632, -5544,   792,     0,    0,    0,   0,   0,
                495, -3960, 13860, -27720,  34650, -27720, 13860, -3960,   495,    0,    0,   0,   0,
                -220,  1980, -7920,  18480, -27720,  27720,-18480,  7920, -1980,  220,    0,   0,   0,
                    66,  -660,  2970,  -7920,  13860, -16632, 13860, -7920,  2970, -660,   66,   0,   0,
                -12,   132,  -660,   1980,  -3960,   5544, -5544,  3960, -1980,  660, -132,  12,   0,
                    1,   -12,    66,   -220,    495,   -792,   924,  -792,   495, -220,   66, -12,   1;
            break;
        }
        }
    }

    void bernstein::setAieqBieqsfc(
        int axis_dim,
        std::vector<alan_visualization::Polyhedron> corridor, 
        int n_order, 
        int m, 
        int d_order, 
        std::vector<double> s
        )
    {
        // this is for A_ieq_sfc

        int onedim_ctrl_pts_per_seg = (n_order + 1);
        int onedim_ctrl_pts_per_dim = m * (n_order + 1);
        int total_ctrl_pts          = axis_dim * m * (n_order + 1);
        

        
        int tangent_plane_no = 0;
        //total kinematic constraints for control points
        int total_constraint_no = 0;

        for(int i = 0; i < corridor.size(); i++)//  corridor.size()
            tangent_plane_no = tangent_plane_no + corridor[i].PolyhedronTangentArray.size();        
        
        // std::cout<<"onedim_ctrl_pts_per_seg: "<<onedim_ctrl_pts_per_seg<<std::endl;
        // std::cout<<"tangent_plane_no: "<<tangent_plane_no<<std::endl<<std::endl;
        
        total_constraint_no = tangent_plane_no * onedim_ctrl_pts_per_seg;    


        A_ieqsfc.resize(total_constraint_no, total_ctrl_pts);
        ub_ieqsfc.resize(total_constraint_no);
        lb_ieqsfc.resize(total_constraint_no);

        A_ieqsfc.setZero();
        ub_ieqsfc.setZero();
        lb_ieqsfc.setZero();
        
        int starto_row = 0;
        int starto_col = 0;

        int starto_row_for_each_segment = 0;

        for(int i = 0; i < corridor.size(); i++) // corridor.size()
        {
            //which corridor, i.e., segment
            starto_col = i * onedim_ctrl_pts_per_seg;            
            
            if(i > 0)
                //which control points we are at , hence the row of constraints
                starto_row = starto_row + corridor[i-1].PolyhedronTangentArray.size() * onedim_ctrl_pts_per_seg;
               
            
            for(int j = 0; j < onedim_ctrl_pts_per_seg; j++)
            {

                switch (axis_dim)
                {
                case 1:

                    for(int k = 0; k <corridor[i].PolyhedronTangentArray.size(); k++)
                    {
                        A_ieqsfc(starto_row_for_each_segment, starto_col + j) = corridor[i].PolyhedronTangentArray[k].n.X;
                    
                        ub_ieqsfc(starto_row_for_each_segment) = corridor[i].PolyhedronTangentArray[k].n.X 
                                                    * corridor[i].PolyhedronTangentArray[k].pt.X;
                        
                        lb_ieqsfc(starto_row_for_each_segment) = -(double)1e30;
                        starto_row_for_each_segment++;
                    }
                    
                    break;
                
                case 2:

                    for(int k = 0; k < corridor[i].PolyhedronTangentArray.size(); k++)
                    {
                        
                        A_ieqsfc(starto_row_for_each_segment, starto_col + j) = corridor[i].PolyhedronTangentArray[k].n.X;                    
                        A_ieqsfc(starto_row_for_each_segment, starto_col + j + 1 * onedim_ctrl_pts_per_dim) = corridor[i].PolyhedronTangentArray[k].n.Y;
                        
                        ub_ieqsfc(starto_row_for_each_segment) = corridor[i].PolyhedronTangentArray[k].n.X
                                                    * corridor[i].PolyhedronTangentArray[k].pt.X
                                                    +
                                                    corridor[i].PolyhedronTangentArray[k].n.Y
                                                    * corridor[i].PolyhedronTangentArray[k].pt.Y;
                        
                        lb_ieqsfc(starto_row_for_each_segment) = -(double)1e30;
                        starto_row_for_each_segment++;
                    }
                                        
                    break;

                case 3: 

                    for(int k = 0; k < corridor[i].PolyhedronTangentArray.size(); k++)                    
                    {
                        // std::cout<<s[i]<<std::endl;
                        // std::cout<<"hi: "<<starto_row_for_each_segment<<" "<<starto_col<<std::endl;                        
                        A_ieqsfc(starto_row_for_each_segment, starto_col + j) = corridor[i].PolyhedronTangentArray[k].n.X * s[i];
                        A_ieqsfc(starto_row_for_each_segment, starto_col + j + 1 * onedim_ctrl_pts_per_dim) = corridor[i].PolyhedronTangentArray[k].n.Y * s[i];
                        A_ieqsfc(starto_row_for_each_segment, starto_col + j + 2 * onedim_ctrl_pts_per_dim) = corridor[i].PolyhedronTangentArray[k].n.Z * s[i];
                        
                        ub_ieqsfc(starto_row_for_each_segment) = corridor[i].PolyhedronTangentArray[k].n.X
                                                    * corridor[i].PolyhedronTangentArray[k].pt.X
                                                    +
                                                    corridor[i].PolyhedronTangentArray[k].n.Y
                                                    * corridor[i].PolyhedronTangentArray[k].pt.Y
                                                    +
                                                    corridor[i].PolyhedronTangentArray[k].n.Z
                                                    * corridor[i].PolyhedronTangentArray[k].pt.Z;
                        
                        lb_ieqsfc(starto_row_for_each_segment) = -(double)1e30;
                        starto_row_for_each_segment++;
                    }
                    
                    break;

                default:
                    ROS_ERROR("AXIS_DIM wrong, PLEASE CHECK!");
                    break;
                }            
            }
        }

        // std::cout<<A_ieqsfc<<std::endl<<std::endl;

    }

    void bernstein::setMQMFinal()
    {
        std::cout<<"now in setMQMFinal: "<<MQM_array.size()<<std::endl;

        int MQM_final_rows = MQM_array.size() * MQM_array[0].rows();
        int MQM_final_cols = MQM_array.size() * MQM_array[0].cols();

        MQM_final.resize(MQM_final_rows, MQM_final_cols);


        for(int i = 0; i < MQM_array.size(); i++)        
            MQM_final.block(i * MQM_array[i].rows(), i * MQM_array[i].cols(), MQM_array[i].rows(), MQM_array[i].cols()) = MQM_array[i];
        

        // remove("/home/patty/alan_ws/src/alan/alan_landing_planning/src/log/MQM_matrix.txt");

        // ofstream save("/home/patty/alan_ws/src/alan/alan_landing_planning/src/log/MQM_matrix.txt",ios::app);
        // save<<MQM_final<<std::endl;
    }

    void bernstein::setAFinal()
    {
        // std::cout<<A_array[0]<<std::endl;
        int A_final_rows = A_array.size() * A_array[0].rows();
        int A_final_cols = A_array.size() * A_array[0].cols();

        // std::cout<<"setAFianl"<<std::endl;

        // std::cout<<A_final_rows<<std::endl;
        // std::cout<<A_final_cols<<std::endl;

        A_final.resize(A_final_rows, A_final_cols);

        for(int i = 0; i < A_array.size(); i++)        
            A_final.block(i * A_array[i].rows(), i * A_array[i].cols(), A_array[i].rows(), A_array[i].cols()) = A_array[i];
        
    }

    void bernstein::setUbFinal()
    {
        int ub_final_rows = ub_array.size() * ub_array[0].size();
        
        ub_final.resize(ub_final_rows);

        for(int i = 0; i < ub_array.size(); i++)        
            ub_final.middleRows(ub_array[i].size() * i, ub_array[i].size()) = ub_array[i];
    }

    void bernstein::setLbFinal()
    {
        int lb_final_rows = lb_array.size() * lb_array[0].size();
        
        lb_final.resize(lb_final_rows);

        for(int i = 0; i < lb_array.size(); i++)        
            lb_final.middleRows(lb_array[i].size() * i, lb_array[i].size()) = lb_array[i];

    
    }

    void bernstein::setAFinal_polyh()
    {
        std::cout<<"\n\nhi now in AFinal_polyh..."<<std::endl;
        
        int A_final_rows = A_sfc_eq_array.size() * A_sfc_eq_array[0].rows()
                            + A_ieqsfc.rows()
                            + A_sfc_ieq_dyn_array.size() * A_sfc_ieq_dyn_array[0].rows();
        
        int A_final_cols = A_sfc_eq_array.size() * A_sfc_eq_array[0].cols();
        
        
        A_final.resize(A_final_rows, A_final_cols);
        A_final.setZero();
        std::cout<<"here are the final sizes of constraints...\n";
        std::cout<<A_sfc_eq_array.size() * A_sfc_eq_array[0].rows()<<std::endl;
        std::cout<<A_ieqsfc.rows()<<std::endl;
        std::cout<<A_sfc_ieq_dyn_array.size() * A_sfc_ieq_dyn_array[0].rows()<<std::endl;

        std::cout<<"A_final_polyh size: "<<A_final.rows()<<"  "<<A_final.cols()<<std::endl;

        for(int i = 0 ; i < A_sfc_eq_array.size(); i++)
        {
            A_final.block(i * A_sfc_eq_array[i].rows(), i * A_sfc_eq_array[i].cols(), A_sfc_eq_array[i].rows(), A_sfc_eq_array[i].cols()) = A_sfc_eq_array[i];
        }


        A_final.block(A_sfc_eq_array.size() * A_sfc_eq_array[0].rows(), 0, A_ieqsfc.rows(), A_ieqsfc.cols()) = A_ieqsfc;


        int starto = A_sfc_eq_array.size() * A_sfc_eq_array[0].rows() + A_ieqsfc.rows(); ;//


        for(int i = 0 ; i < A_sfc_ieq_dyn_array.size(); i++)
        {
            A_final.block(starto + i * A_sfc_ieq_dyn_array[i].rows(), i * A_sfc_eq_array[i].cols(), A_sfc_ieq_dyn_array[i].rows(), A_sfc_ieq_dyn_array[i].cols()) = A_sfc_ieq_dyn_array[i];
        }


        // remove("/home/patty/alan_ws/src/alan/alan_landing_planning/src/log/A_matrix.txt");

        // ofstream save("/home/patty/alan_ws/src/alan/alan_landing_planning/src/log/A_matrix.txt",ios::app);
        // save<<A_final<<std::endl;
        // save.close();
    }

    void bernstein::setUbFinal_polyh()
    {
        int ub_final_rows = ub_eq_array.size() * ub_eq_array[0].rows()
                            + ub_ieqsfc.rows()
                            + ub_ieq_array.size() * ub_ieq_array[0].rows();

        ub_final.resize(ub_final_rows);
        ub_final.setZero();

        for(int i = 0; i < ub_eq_array.size(); i++)
            ub_final.middleRows(ub_eq_array[i].size() * i, ub_eq_array[i].size()) = ub_eq_array[i];
        
        ub_final.middleRows(ub_eq_array.size() * ub_eq_array[0].rows(), ub_ieqsfc.size()) = ub_ieqsfc;

        int starto = ub_eq_array.size() * ub_eq_array[0].rows() + ub_ieqsfc.rows(); ;//
        
        for(int i = 0; i < ub_ieq_array.size(); i++)
            ub_final.middleRows(starto + i * ub_ieq_array[i].size(), ub_ieq_array[i].size()) = ub_ieq_array[i];                            

        // remove("/home/patty/alan_ws/src/alan/alan_landing_planning/src/log/ub.txt");

        // ofstream save("/home/patty/alan_ws/src/alan/alan_landing_planning/src/log/ub.txt",ios::app);
        // save<<ub_final<<std::endl;
        // save.close();
    }

    void bernstein::setLbFinal_polyh()
    {
        int lb_final_rows = lb_eq_array.size() * lb_eq_array[0].rows()
                            + lb_ieqsfc.rows()
                            + lb_ieq_array.size() * lb_ieq_array[0].rows();

        lb_final.resize(lb_final_rows);
        lb_final.setZero();

        for(int i = 0; i < lb_eq_array.size(); i++)
            lb_final.middleRows(lb_eq_array[i].size() * i, lb_eq_array[i].size()) = lb_eq_array[i];
        
        lb_final.middleRows(lb_eq_array.size() * lb_eq_array[0].rows(), lb_ieqsfc.size()) = lb_ieqsfc;

        int starto = lb_eq_array.size() * lb_eq_array[0].rows() + lb_ieqsfc.rows(); ;//
        
        for(int i = 0; i < lb_ieq_array.size(); i++)
            lb_final.middleRows(starto + i * lb_ieq_array[i].size(), lb_ieq_array[i].size()) = lb_ieq_array[i];                            


        // remove("/home/patty/alan_ws/src/alan/alan_landing_planning/src/log/lb.txt");

        // ofstream save("/home/patty/alan_ws/src/alan/alan_landing_planning/src/log/lb.txt",ios::app);
        // save<<lb_final<<std::endl;
        // save.close();

    }

    Eigen::MatrixXd bernstein::set_1_MQMSample(int axis_dim, int n_order, int m, int d_order, std::vector<double> s)
    // set 1 MQM sample
    // with different time allocations
    // utilize it in pre-requisite
    {
        Eigen::MatrixXd MQM_return;

        std::vector<Eigen::MatrixXd> MQM_array;

        // std::cout<<std::endl<<std::endl;
        // std::cout<<"hi now in setMQMSample"<<std::endl;

        // std::cout<<axis_dim<<std::endl;
        // std::cout<<n_order<<std::endl;
        // std::cout<<m<<std::endl;

        MQM_array.clear();

        for(int axis_i = 0; axis_i < axis_dim; axis_i++)
        {
            setMQM1D(axis_i, n_order, m, d_order, s);
            MQM_array.emplace_back(MQM);
        }

        int MQM_final_rows = MQM_array.size() * MQM_array[0].rows();
        int MQM_final_cols = MQM_array.size() * MQM_array[0].cols();

        MQM_return.resize(MQM_final_rows, MQM_final_cols);

        for(int i = 0; i < MQM_array.size(); i++)
            MQM_return.block(i * MQM_array[i].rows(), i * MQM_array[i].cols(), MQM_array[i].rows(), MQM_array[i].cols()) = MQM_array[i];

        // std::cout<<"hi: "<<MQM_return.rows()<<" "<<MQM_return.cols()<<std::endl;
        return MQM_return;
    }

    Eigen::MatrixXd bernstein::set_1_ASample(
        int axis_dim, 
        int n_order, 
        int m, 
        int d_order,  
        std::vector<alan_visualization::Polyhedron> sfc_list,
        std::vector<double> s
    )
    {
        Eigen::MatrixXd Aeq_1_sample = set_1_AeqSample(axis_dim, n_order, m, d_order, s);
        Eigen::MatrixXd Aieqdyn_1_sample = set_1_AieqDynSample(axis_dim, n_order, m, d_order, s);
        setAieqBieqsfc(axis_dim, sfc_list, n_order, m, d_order, s);
        
        
        
        
        
        
        
        // later after lunch, combine all these, and check dimension...
        // std::cout<<Aeq_1_sample<<Aieqdyn_1_sample<<A_ieqsfc<<std::endl;
        // std::cout<<"here!..."<<std::endl;
        // std::cout<<Aeq_1_sample.rows()<<" "<<Aeq_1_sample.cols()<<std::endl;
        // std::cout<<Aieqdyn_1_sample.rows()<<" "<<Aieqdyn_1_sample.cols()<<std::endl;
        // std::cout<<A_ieqsfc.rows()<<" "<<A_ieqsfc.cols()<<std::endl<<std::endl;;

        Eigen::MatrixXd A_return;
        int A_return_rows = Aeq_1_sample.rows() + A_ieqsfc.rows() + Aieqdyn_1_sample.rows();
        int A_return_cols = Aeq_1_sample.cols();

        A_return.resize(A_return_rows, A_return_cols);
        A_return.setZero();

        int starto_row = 0;
        int starto_col = 0;
        A_return.block(starto_row, starto_col, Aeq_1_sample.rows(), Aeq_1_sample.cols()) = 
            Aeq_1_sample;

        starto_row = Aeq_1_sample.rows();
        starto_col = 0;
        A_return.block(starto_row, starto_col, A_ieqsfc.rows(), A_ieqsfc.cols()) = 
            A_ieqsfc;
        
        starto_row = Aeq_1_sample.rows() + A_ieqsfc.rows();
        starto_col = 0;
        A_return.block(starto_row, starto_col, Aieqdyn_1_sample.rows(), Aieqdyn_1_sample.cols()) 
            = Aieqdyn_1_sample;

        // std::cout<<A_return.rows()<<std::endl;
        // std::cout<<A_return.cols()<<std::endl<<std::endl;

        return A_return;
    }

    Eigen::MatrixXd bernstein::set_1_AeqSample(int axis_dim, int n_order, int m, int d_order, std::vector<double> s)
    // set 1 Aeq sample
    // with different time allocations
    // utilize it in pre-requisite
    {
        Eigen::MatrixXd Aeq_return;

        std::vector<Eigen::MatrixXd> Aeq_array;

        // std::cout<<std::endl<<std::endl;
        // std::cout<<"hi now in setAeqSample"<<std::endl;

        // std::cout<<axis_dim<<std::endl;
        // std::cout<<n_order<<std::endl;
        // std::cout<<m<<std::endl;

        Aeq_array.clear();

        for(int axis_i = 0; axis_i < axis_dim; axis_i++)
        {
            setAeq1D(axis_i, n_order, m, d_order, s);
            Aeq_array.emplace_back(A_eq);
        }

        int Aeq_final_rows = Aeq_array.size() * Aeq_array[0].rows();
        int Aeq_final_cols = Aeq_array.size() * Aeq_array[0].cols();

        Aeq_return.resize(Aeq_final_rows, Aeq_final_cols);
        Aeq_return.setZero();

        for(int i = 0; i < Aeq_array.size(); i++)
            Aeq_return.block(i * Aeq_array[i].rows(), i * Aeq_array[i].cols(), Aeq_array[i].rows(), Aeq_array[i].cols()) = Aeq_array[i];
        
        // std::cout<<"hi...: "<<Aeq_return.rows()<<" "<<Aeq_return.cols()<<std::endl; 
        return Aeq_return;
    }

    Eigen::MatrixXd bernstein::set_1_AieqDynSample(int axis_dim, int n_order, int m, int d_order, std::vector<double> s)
    {
        Eigen::MatrixXd Aieq_return;

        std::vector<Eigen::MatrixXd> Aieq_array;

        // std::cout<<std::endl<<std::endl;
        // std::cout<<"hi now in setAeqSample"<<std::endl;

        // std::cout<<axis_dim<<std::endl;
        // std::cout<<n_order<<std::endl;
        // std::cout<<m<<std::endl;

        Aieq_array.clear();

        for(int axis_i = 0; axis_i < axis_dim; axis_i++)
        {
            setAieq1D(axis_i, n_order, m, d_order, s, "POLYH");;
            Aieq_array.emplace_back(A_ieq);
        }

        int Aieq_final_rows = Aieq_array.size() * Aieq_array[0].rows();
        int Aieq_final_cols = Aieq_array.size() * Aieq_array[0].cols();

        Aieq_return.resize(Aieq_final_rows, Aieq_final_cols);
        Aieq_return.setZero();

        for(int i = 0; i < Aieq_array.size(); i++)
            Aieq_return.block(i * Aieq_array[i].rows(), i * Aieq_array[i].cols(), Aieq_array[i].rows(), Aieq_array[i].cols()) = Aieq_array[i];
        
        // std::cout<<"hi...: "<<Aieq_return.rows()<<" "<<Aieq_return.cols()<<std::endl; 
        // std::cout<<Aieq_return<<std::endl;
        return Aieq_return;
    }

    std::tuple<Eigen::VectorXd, Eigen::VectorXd> bernstein::set_ub_lb(
        int axis_dim, 
        int n_order, 
        int m, 
        int d_order,
        dynamic_constraints d_constraints
    )
    {
        ub_eq_array.clear();
        ub_ieq_array.clear();

        lb_eq_array.clear();
        lb_ieq_array.clear();

        for(int axis_i = 0; axis_i < axis_dim; axis_i++)
        {
            endpt_cond start_temp, end_temp;

            start_temp = {
                0.0, 0.0, 0.0, 0.0
            };

            end_temp = {
                0.0, 0.0, 0.0, 0.0
            };

            setUBeq1D(axis_i, start_temp, end_temp, n_order, m, d_order);
            setLBeq1D(axis_i, start_temp, end_temp, n_order, m, d_order);
            ub_eq_array.emplace_back(ub_eq);
            lb_eq_array.emplace_back(lb_eq);


            setUBieq1D_polyh(axis_i, d_constraints, n_order, m, d_order);        
            setlBieq1D_polyh(axis_i, d_constraints, n_order, m, d_order);
            ub_ieq_array.emplace_back(ub_ieq);                
            lb_ieq_array.emplace_back(lb_ieq);            
        }

        setUbFinal_polyh();
        setLbFinal_polyh();

        return std::tuple<Eigen::VectorXd, Eigen::VectorXd>(ub_final, lb_final);

    }


    inline double bernstein::permutation(int p, int q)
    {
        return factorial(p) / factorial(q);
    }

    inline double bernstein::factorial(int r)
    {
        double _r = 1;
        for(int i = 1 ; i <= r; i++)
            _r = _r * i;
        
        return _r;
    }

    std::vector<double> bernstein::pascal_triangle(int level)
    {
        std::vector<double> _array;
        switch(level)
        {
        case 1: 
        {
            _array.push_back(1);
            break;

        }
        case 2: 
        {
            _array.push_back(1 * (-1));
            _array.push_back(1);
            break;

        }
        case 3:
        {
            _array.push_back(1);
            _array.push_back(2 * (-1));
            _array.push_back(1);
            break;

        }
        case 4:
        {
            _array.push_back(1 * (-1));
            _array.push_back(3);
            _array.push_back(3 * (-1));
            _array.push_back(1);
            break;

        }
        case 5:
        {
            _array.push_back(1);
            _array.push_back(4 * (-1));
            _array.push_back(6);
            _array.push_back(4 * (-1));
            _array.push_back(1);
            break;

        }
        case 7:
        {
            _array.push_back(1 * (-1));
            _array.push_back(5);
            _array.push_back(10* (-1));
            _array.push_back(10);
            _array.push_back(5 * (-1));
            _array.push_back(1);
            break;

        }
        }

        return _array;

    }

}