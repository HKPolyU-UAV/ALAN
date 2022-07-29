#ifndef munkres_H
#define munkres_H

#include "essential.h"

using namespace std;

namespace my_mathtool_for_alan
{
    typedef struct MOT
    {
        cv::Mat state_of_1obj;
        int id;
        int tracked_i;
        int missing;
    }MOT;

    typedef struct Match
    {
        int id;
        bool toofar;
    }Match;

    class munkres
    {
        void stp1(int &step);//reduce with the minima of row and column
        void stp2(int &step);
        void stp3(int &step);
        void stp4(int &step);
        void stp5(int &step);
        void stp6(int &step);
        void stp7();
        void find_a_zero(int& row, int& col);
        bool star_in_row(int row);
        void find_star_in_row(int row, int& col);
        void find_min(double& minval);
        void find_star_in_col(int col, int& row);
        void find_prime_in_row(int row, int& col);
        void augment_path();
        void clear_covers();
        void erase_primes();

        int step = 1;
        Eigen::MatrixXd cost, mask, path, copy;
        vector<int> cover_row;
        vector<int> cover_col;
        int path_row_0, path_col_0, path_count;

        void cost_generate(vector<cv::Point> detected, vector<cv::Point> previous);

    public:
        munkres();
        ~munkres();
        vector<Match> solution(vector<cv::Mat>measured, vector<MOT>previous);//return the corresponding ids
        vector<Match> id_match;
    };

    munkres::munkres()
    {

    }

    munkres::~munkres()
    {

    }

    vector<Match> munkres::solution(vector<cv::Mat> measured, vector<MOT> previous )
    {
        id_match.clear();
        cv::Point temp;
        vector<cv::Point> detected_pts, previous_pts;

        for(auto o:measured)
        {
            temp = cv::Point (o.at<float>(0), o.at<float>(1));
            detected_pts.push_back(temp);
        }
        for(auto o:previous)
        {
            temp = cv::Point (o.state_of_1obj.at<float>(0), o.state_of_1obj.at<float>(1));
            previous_pts.push_back(temp);
        }

        cost_generate(detected_pts,previous_pts);
        copy = cost;

        bool done = false;
        step = 1;

        while (!done)
        {
            //cout << endl << step << endl << endl;
            switch (step)
            {
            case 1:
                stp1(step);
                break;
            case 2:
                stp2(step);
                break;
            case 3:
                stp3(step);
                break;
            case 4:
                stp4(step);
                break;
            case 5:
                stp5(step);
                break;
            case 6:
                stp6(step);
                break;
            case 7:
                stp7();
                done = true;
    //            cout<<"bye"<<endl;
                break;
            }
        }

        return id_match;

    }

    void munkres::cost_generate(vector<cv::Point> detected, vector<cv::Point> previous)
    {
        if(detected.size()==previous.size())
        {
            cost.setZero(detected.size(), previous.size());
            mask.setZero(detected.size(), previous.size());
            cover_row = vector<int>(detected.size(),0);
            cover_col = vector<int>(detected.size(),0);
            path.setZero(detected.size()*2,2);
        }
        else if (detected.size()<previous.size())
        {
            cost.setZero(previous.size(), previous.size());
            mask.setZero(previous.size(), previous.size());
            cover_row = vector<int>(previous.size(),0);
            cover_col = vector<int>(previous.size(),0);
            path.setZero(previous.size()*2,2);
        }
        else if (detected.size()>previous.size())
        {
            cost.setZero(detected.size(), detected.size());
            mask.setZero(detected.size(), detected.size());
            cover_row = vector<int>(detected.size(),0);
            cover_col = vector<int>(detected.size(),0);
            path.setZero(detected.size()*2,2);
        }

        for (int i=0;i<detected.size();i++)
        {
            for (int j=0;j<previous.size();j++)
            {
                cost (i,j) = cv::norm ( detected[i] - previous[j] );
            }
        }
    }

    void munkres::stp1(int& step)
    {
        double minval;
        Eigen::MatrixXd minvals;

        minvals = cost.rowwise().minCoeff();
        for (int i = 0; i < cost.rows(); i++)
        {
            minval = minvals(i, 0);
            for (int j = 0; j < cost.cols(); j++)
            {
                cost(i, j) = cost(i, j) - minval;
            }
        }
        step = 2;
    }

    void munkres::stp2(int &step)
    {
        for (int r = 0; r < cost.rows(); r++)
        {
            for (int c = 0; c < cost.cols(); c++)
            {
                if (cost(r, c) == 0 && cover_row[r] == 0 && cover_col[c] == 0)
                {
                    mask(r, c) = 1;
                    cover_row[r] = 1;
                    cover_col[c] = 1;
                }
            }
        }
        for (int r = 0; r < cost.rows(); r++)
            cover_row[r] = 0;
        for (int c = 0; c < cost.cols(); c++)
            cover_col[c] = 0;
        step = 3;
    }

    void munkres::stp3(int &step)
    {
        int count = 0;
        for (int r = 0; r < cost.rows(); r++)
            for (int c = 0; c < cost.cols(); c++)
                if (mask(r, c) == 1)
                    cover_col[c] = 1;
        for (int c = 0; c < cost.cols(); c++)
            if (cover_col[c] == 1)
                count += 1;
        if (count == cost.cols() )
            step = 7;
        else
            step = 4;
    }

    void munkres::stp4(int &step)
    {
        int row = -1;
        int col = -1;
        bool done;
        done = false;

        while (!done)
        {
            find_a_zero(row, col);
            if (row == -1)
            {
                done = true;
                step = 6;
            }
            else
            {
                mask(row, col) = 2;
                if (star_in_row(row))
                {
                    find_star_in_row(row, col);
                    cover_row[row] = 1;
                    cover_col[col] = 0;
                }
                else
                {
                    done = true;
                    step = 5;
                    path_row_0 = row;
                    path_col_0 = col;
                }
            }
        }
    }

    void munkres::stp5(int &step)
    {
        bool done;
        int row = -1;
        int col = -1;

        path_count = 1;
        path(path_count - 1, 0) = path_row_0;
        path(path_count - 1, 1) = path_col_0;
        done = false;
        while (!done)
        {
            find_star_in_col(path(path_count - 1, 1), row);
            if (row > -1)
            {
                path_count += 1;
                path(path_count - 1, 0) = row;
                path(path_count - 1, 1) = path(path_count - 2, 1);
            }
            else
                done = true;
            if (!done)
            {
                find_prime_in_row(path(path_count - 1, 0), col);
                path_count += 1;
                path(path_count - 1, 0) = path(path_count - 2, 0);
                path(path_count - 1, 1) = col;
            }
        }
        augment_path();
        clear_covers();
        erase_primes();
        step = 3;
    }

    void munkres::stp6(int &step)
    {
        double minval = DBL_MAX;
        find_min(minval);
        for (int r = 0; r < cost.rows(); r++)
            for (int c = 0; c < cost.cols(); c++)
            {
                if (cover_row[r] == 1)
                    cost(r, c) += minval;
                if (cover_col[c] == 0)
                    cost(r, c) -= minval;
            }
        //cout<<minval<<endl;
        step = 4;
    }

    void munkres::stp7()
    {
        for(int r = 0; r<cost.rows(); r++)
        {
            for (int c = 0; c<cost.cols();c++)
            {
                if(mask(r,c) == 1 && copy(r,c) <= 100 /*&& copy(r,c) != 0*/   )
                {
                    Match temp = {c,false};
                    id_match.push_back(temp);
                }
                else if(mask(r,c) == 1 && copy(r,c) > 100 /*|| copy(r,c) == 0   )*/)
                {
                    Match temp = {c,true};
                    id_match.push_back(temp);
                }

            }
        }
    }

    void munkres::find_a_zero(int &row, int &col)
    {
        int r = 0;
        int c;
        bool done;
        row = -1;
        col = -1;
        done = false;

        while (!done)
        {
            c = 0;
            while (true)
            {
                if (cost(r, c) == 0 && cover_row[r] == 0 && cover_col[c] == 0)
                {
                    row = r;
                    col = c;
                    done = true;
                }
                c += 1;
                if (c >= cost.cols() || done)
                    break;
            }
            r += 1;
            if (r >= cost.rows())
                done = true;
        }
    }

    bool munkres::star_in_row(int row)
    {
        bool temp = false;
        for (int c = 0; c < cost.cols(); c++)
            if (mask(row, c) == 1)
            {
                temp = true;
                break;
            }
        return temp;
    }

    void munkres::find_star_in_row(int row, int &col)
    {
        col = -1;
        for (int c = 0; c < cost.cols(); c++)
        {
            if (mask(row, c) == 1)
                col = c;
        }
    }

    void munkres::find_min(double &minval)
    {
        for (int r = 0; r < cost.rows(); r++)
            for (int c = 0; c < cost.cols(); c++)
                if (cover_row[r] == 0 && cover_col[c] == 0)
                    if (minval > cost(r, c))
                        minval = cost(r, c);
    }

    void munkres::find_star_in_col(int col, int &row)
    {
        row = -1;
        for (int i = 0; i < cost.rows(); i++)
            if (mask(i, col) == 1)
                row = i;
    }

    void munkres::find_prime_in_row(int row, int &col)
    {
        for (int j = 0; j < cost.cols(); j++)
            if (mask(row, j) == 2)
                col = j;
    }

    void munkres::augment_path()
    {
        for (int p = 0; p < path_count; p++)
        {
            for (int p = 0; p < path_count; p++)
            {
                int i = path(p, 0);
                int j = path(p, 1);
                if (mask(i, j) == 1)
                    mask(i, j) = 0;
                else
                    mask(i, j) = 1;
            }

        }
    }

    void munkres::clear_covers()
    {
        for (int r = 0; r < cost.rows(); r++)
            cover_row[r] = 0;
        for (int c = 0; c < cost.cols(); c++)
            cover_col[c] = 0;
    }

    void munkres::erase_primes()
    {
        for (int r = 0; r < cost.rows(); r++)
            for (int c = 0; c < cost.cols(); c++)
                if (mask(r, c) == 2)
                    mask(r, c) = 0;
    }
}



#endif // munkres_H