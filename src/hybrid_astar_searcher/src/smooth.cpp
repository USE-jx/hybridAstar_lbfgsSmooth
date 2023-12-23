/**
 * @file smooth.cpp
 * @author jiaxier (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "hybrid_astar_searcher/smooth.h"

namespace planning
{

Smoother::Smoother(/* args */)
{
}

Smoother::~Smoother()
{
}

double Smoother::costFunction(void *ptr, 
                                const Eigen::VectorXd &x, 
                                Eigen::VectorXd &g)  {
    auto instance = reinterpret_cast<Smoother *>(ptr);
    std::vector<Vec3d> smooth_path = instance->smooth_path_;
    const int points_num = smooth_path.size() - 4;
    Eigen::Matrix2Xd opt_points;
    opt_points.resize(2, smooth_path.size());
    opt_points(0,0) = smooth_path[0](0);
    opt_points(1,0) = smooth_path[0](1);
    opt_points(0,1) = smooth_path[1](0);
    opt_points(1,1) = smooth_path[1](1);
    //std::cout << "0000" << std::endl;
    
    opt_points.block(0,2,1,points_num) = x.head(points_num).transpose();
    opt_points.block(1,2,1,points_num) = x.tail(points_num).transpose();
    //std::cout << "1111" << std::endl;
    opt_points.col(smooth_path.size()-2)(0) = smooth_path[smooth_path.size()-2](0);
    opt_points.col(smooth_path.size()-2)(1) = smooth_path[smooth_path.size()-2](1);
    opt_points.col(smooth_path.size()-1)(0) = smooth_path[smooth_path.size()-1](0);
    opt_points.col(smooth_path.size()-1)(1) = smooth_path[smooth_path.size()-1](1);
    //std::cout << "opt_points" << opt_points << std::endl;
    

    Eigen::Matrix2Xd grad;
    grad.resize(2, points_num);
    grad.setZero();
    double cost = 0.0;

    
    double max_clearance = instance->max_clearance_;
    
    //std::cout << "calculate collision cost" << std::endl;
    //collision cost
    double collision_cost = 0.0;
    Eigen::Matrix2Xd collision_grad;
    collision_grad.resize(2, points_num);
    collision_grad.setZero();

    for (int i = 2; i < opt_points.cols()-2; ++i) {
        Vec2i x_i;
        x_i(0) = static_cast<int>((opt_points(0, i) - (-25)) / 1);
        x_i(1) = static_cast<int>((opt_points(1, i) - (-25)) / 1);

        //dist to the closet obstacle    unit m  1是分辨率
        double dist2obs = 1 * instance->voronoi_.getDistance(x_i(0), x_i(1));
        //std::cout << "dist2obs:" << dist2obs << std::endl;

        Vec2d vec_o2x(x_i(0) - instance->voronoi_.data[x_i(0)][x_i(1)].obstX,
                  x_i(1) - instance->voronoi_.data[x_i(0)][x_i(1)].obstY);
        //std::cout << "vec_o2x:" << vec_o2x << std::endl;
        

        if (dist2obs - max_clearance < 0) {
            collision_cost += instance->w_obs_ * pow((dist2obs - max_clearance), 2);
            Vec2d gradient;
            gradient = instance->w_obs_ * 2 * (dist2obs - max_clearance) / dist2obs * vec_o2x;
            collision_grad(0, i-2) = gradient(0);
            collision_grad(1, i-2) = gradient(1);
        } else {
            collision_cost += 0;
            collision_grad(0, i-2) = 0;
            collision_grad(1, i-2) = 0;
        }
        
    }
    cost += collision_cost;
    grad += collision_grad;

    //std::cout << "calculate smooth cost" << std::endl;
    //smooth cost
    double smooth_cost = 0.0;
    Eigen::Matrix2Xd smooth_grad;
    smooth_grad.resize(2, points_num);
    smooth_grad.setZero();
    //std::cout << opt_points.cols()-1 << std::endl;
    for (int i = 2; i < opt_points.cols()-2; ++i)  {
        Vec2d x_p2(opt_points(0, i-2), opt_points(1, i-2));
        Vec2d x_p(opt_points(0, i-1), opt_points(1, i-1));
        Vec2d x_c(opt_points(0, i), opt_points(1, i));
        Vec2d x_n(opt_points(0, i+1), opt_points(1, i+1));
        Vec2d x_n2(opt_points(0, i+2), opt_points(1, i+2));

        
        
        Vec2d err = x_p + x_n - 2* x_c;

        smooth_cost += instance->w_smo_ * err.transpose() * err;
        //std::cout << smooth_cost << std::endl;
        
        //smooth_grad.col(i-1) = ((-4) * x_p + 8 * x_c - 4 * x_n);
        smooth_grad.col(i-2) = instance->w_smo_ * 2 * (x_p2 - 4 * x_p + 6 * x_c - 4 * x_n + x_n2);

        
    }
    //std::cout << "smooth_grad" << smooth_grad << std::endl;
    cost += smooth_cost;
    grad +=  smooth_grad;
    //std::cout << "grad" << grad << std::endl;

    //curvature cost
    double curvature_cost = 0.0;
    Eigen::Matrix2Xd curvature_grad;
    curvature_grad.resize(2, points_num);
    curvature_grad.setZero();
    //std::cout << opt_points.cols()-1 << std::endl;
    for (int i = 2; i < opt_points.cols()-2; ++i)  {
        Vec2d x_p2(opt_points(0, i-2), opt_points(1, i-2));
        Vec2d x_p(opt_points(0, i-1), opt_points(1, i-1));
        Vec2d x_c(opt_points(0, i), opt_points(1, i));
        Vec2d x_n(opt_points(0, i+1), opt_points(1, i+1));
        Vec2d x_n2(opt_points(0, i+2), opt_points(1, i+2));

        //四段线
        Vec2d delta_x_p = x_p - x_p2;
        Vec2d delta_x_c = x_c - x_p;
        Vec2d delta_x_n = x_n - x_c;
        Vec2d delta_x_n2 = x_n2 - x_n;

        if (delta_x_p.norm() > 0 && delta_x_c.norm() > 0 && delta_x_n.norm() > 0 && delta_x_n2.norm() > 0) {
            //取[-1,1]防止出现nan
            double delta_phi_p = std::acos(std::min(std::max(delta_x_p.dot(delta_x_c) / delta_x_p.norm() / delta_x_c.norm(), -1.0), 1.0));
            double delta_phi_c = std::acos(std::min(std::max(delta_x_c.dot(delta_x_n) / delta_x_c.norm() / delta_x_n.norm(), -1.0), 1.0));
            double delta_phi_n = std::acos(std::min(std::max(delta_x_n.dot(delta_x_n2) / delta_x_n.norm() / delta_x_n2.norm(), -1.0), 1.0));
            //std::cout << delta_x_p.dot(delta_x_c) / delta_x_p.norm() / delta_x_c.norm() << std::endl;
            //std::cout << "delta_phi_p:" << delta_phi_p << std::endl;

            double kappa_p = delta_phi_p / delta_x_p.norm();
            double kappa_c = delta_phi_c / delta_x_c.norm();
            double kappa_n = delta_phi_n / delta_x_n.norm();

            if (kappa_c > instance->max_kappa_ && kappa_p > 0 && kappa_n > 0) {
                auto compute_d_delta_phi = [](const double delta_phi) {
                    return -1.0 / std::sqrt(1.0 - std::pow(std::cos(delta_phi),2));
                };

                auto compute_orthogonal_complement = [](Vec2d x0, Vec2d x1) {
                    return x0 - x1 * x0.dot(x1) / std::pow(x1.norm(), 2);
                };

                double d_delta_phi_p = compute_d_delta_phi(delta_phi_p);
                Vec2d d_cos_delta_phi_p = compute_orthogonal_complement(delta_x_p, delta_x_c) 
                                          /  delta_x_p.norm() / delta_x_c.norm();
                Vec2d d_kappa_p = 1.0 / delta_x_p.norm() * d_delta_phi_p *  d_cos_delta_phi_p;
                Vec2d k_p = 2.0 * (kappa_p - instance->max_kappa_) * d_kappa_p;
                // std::cout <<  std::pow(std::cos(delta_phi_p),2) << std::endl;
                //std::cout << "d_delta_phi_p:" << d_delta_phi_p << std::endl;
                //std::cout << "d_cos_delta_phi_p:" << d_cos_delta_phi_p << std::endl;
                //std::cout << "d_kappa_p:" << d_kappa_p << std::endl;
               
                //std::cout << "kp:" << k_p << std::endl;


                double d_delta_phi_c = compute_d_delta_phi(delta_phi_c);
                Vec2d d_cos_delta_phi_c = compute_orthogonal_complement(delta_x_n, delta_x_c) 
                                          /  delta_x_c.norm() / delta_x_n.norm()
                                          -compute_orthogonal_complement(delta_x_c, delta_x_n)
                                          / delta_x_c.norm() / delta_x_n.norm();

                Vec2d d_kappa_c = 1.0 / delta_x_c.norm() * d_delta_phi_c *  d_cos_delta_phi_c 
                                  -delta_phi_c / std::pow(delta_x_c.norm(), 3) * delta_x_c;
                Vec2d k_c = 2.0 * (kappa_c - instance->max_kappa_) * d_kappa_c;

                //std::cout << "d_cos_delta_phi_c:" << d_cos_delta_phi_c << std::endl;
                //std::cout << "k_c:" << k_c << std::endl;

                double d_delta_phi_n = compute_d_delta_phi(delta_phi_n);
                Vec2d d_cos_delta_phi_n = -compute_orthogonal_complement(delta_x_n2, delta_x_n) 
                                          /  delta_x_n.norm() / delta_x_n2.norm();
                Vec2d d_kappa_n = 1.0 / delta_x_n.norm() * d_delta_phi_n *  d_cos_delta_phi_n 
                                  +delta_phi_n / std::pow(delta_x_n.norm(), 3) * delta_x_n;
                Vec2d k_n = 2.0 * (kappa_n - instance->max_kappa_) * d_kappa_n;
                //std::cout << "d_cos_delta_phi_n:" << d_cos_delta_phi_n << std::endl;
                //std::cout << "kn:" << k_n << std::endl;

                
                curvature_cost += instance->w_cur_ * std::pow(kappa_c - instance->max_kappa_, 2);

                curvature_grad.col(i-2) = instance->w_cur_ * (  0.25*k_p +  0.5*k_c + 0.25*k_n );

            } else {
                curvature_cost += 0;
                curvature_grad.col(i-2) = Vec2d(0, 0);
            }

            
        }

    }
    //std::cout << "curvature_grad" << curvature_grad << std::endl;
    cost += curvature_cost;
    grad += curvature_grad;


    // //voronoi cost
    // double voronoi_cost = 0.0;
    // Eigen::Matrix2Xd voronoi_grad;
    // voronoi_grad.resize(2, points_num);
    // voronoi_grad.setZero();
    // //std::cout << opt_points.cols()-1 << std::endl;
    // for (int i = 2; i < opt_points.cols()-2; ++i)  {

    //     Vec2i x_i;
    //     x_i(0) = static_cast<int>((opt_points(0, i) - (-25)) / 0.1);
    //     x_i(1) = static_cast<int>((opt_points(1, i) - (-25)) / 0.1);

    //     //dist to the closet obstacle    unit m
    //     double dist2obs = 0.1 * instance->voronoi_.getDistance(x_i(0), x_i(1));
    //     //std::cout << "dist2obs:" << dist2obs << std::endl;

    //     Vec2d vec_o2x(x_i(0) - instance->voronoi_.data[x_i(0)][x_i(1)].obstX,
    //                   x_i(1) - instance->voronoi_.data[x_i(0)][x_i(1)].obstY);
       
        // int x_v = x_i(0), y_v = x_i(1);
        // std::vector<Vec2i> voronoi_points;
        // for (int i = x_v - 30; i < (x_v + 30); ++i) {
        //     for (int j = y_v - 30; j < (y_v + 30); ++j) {
        //         if (instance->voronoi_.isVoronoi(x_v, y_v)) {
        //             voronoi_points.push_back({x_v, y_v});
        //         }
        //     }
        // }
        // double dist2edge;
        // Vec2d vec_e2x;
        // if (voronoi_points.empty()) {
        //     dist2edge = 3;
        //     vec_e2x  = -vec_o2x;
        // } else {
        //     int min_idx = 0;
        //     double min_dist = 10;
        //     for (int i = 0; i < voronoi_points.size(); ++i) {
                
        //         double dist = 0.1 * (x_i - voronoi_points[i]).norm();
        //         if (dist < min_dist) {
        //             min_dist = dist;
        //             min_idx = i;
        //         }
                
        //     }
        //     dist2edge = min_dist;
        //     vec_e2x(x_i(0) - voronoi_points[min_idx](0),
        //             x_i(1) - voronoi_points[min_idx](1));
        // }


    //     double alpha = instance->alpha_;

    //     if (dist2obs - max_clearance < 0) {
    //         std::cout << "求gradient:" << std::endl;

    //         voronoi_cost += instance->w_vor_ * alpha /(alpha + dist2obs)
    //                         * dist2edge / (dist2edge + dist2obs)
    //                         * pow(dist2obs - max_clearance, 2) / pow(max_clearance, 2);
            
    //         std::cout << "求gradient:" << std::endl;
            
    //         Vec2d gradient;
    //         gradient = instance->w_vor_ * 
    //                    (alpha /(alpha + dist2obs)
    //                    * dist2edge / (dist2edge + dist2obs)
    //                    * (dist2obs - max_clearance) / pow(max_clearance, 2)
    //                    * ((max_clearance - dist2obs)/(alpha + dist2obs)
    //                      -(dist2obs - max_clearance) / (dist2obs + dist2edge) + 2)
    //                    * vec_o2x / dist2obs
                       
    //                    + 
                       
    //                     alpha /(alpha + dist2obs) 
    //                    * dist2obs / pow(dist2edge + dist2obs, 2)
    //                    * pow(dist2obs - max_clearance, 2) / pow(max_clearance, 2)
    //                    * vec_e2x / dist2edge
    //                    );
                        
    //         voronoi_grad(0, i-2) = gradient(0);
    //         voronoi_grad(1, i-2) = gradient(1);
    //     } else {
    //         voronoi_cost += 0;
    //         voronoi_grad(0, i-2) = 0;
    //         voronoi_grad(1, i-2) = 0;
    //     }
         

        
    // }
    // //std::cout << "smooth_grad" << smooth_grad << std::endl;
    // cost += voronoi_cost;
    // grad +=  voronoi_grad;




    g.setZero();
    g.head(points_num) = grad.row(0).transpose();
    g.tail(points_num) = grad.row(1).transpose();
    //std::cout << "g" << g << std::endl;



    // std::cout << std::setprecision(10)
    // std::cout << "------------------------" << "\n";
    // std::cout << "Function Value: " << cost << "\n";
    // std::cout << "Gradient Inf Norm: " << g.cwiseAbs().maxCoeff() << "\n";
    // std::cout << "------------------------" << "\n";

    return cost;

}

double Smoother::optimize(DynamicVoronoi &voronoi, std::vector<Vec3d> &path) {
    smooth_path_ = path;
    voronoi_ = voronoi;

    int points_num = smooth_path_.size() - 4;
    Eigen::VectorXd x(2 * points_num);
    for (int i = 2; i < smooth_path_.size()-2; ++i) {
        x(i-2) = smooth_path_[i](0);
        x(i-2 + points_num) = smooth_path_[i](1);
    }
    

    //std::cout << "即将优化" << std::endl;
    double minCost = 0.0;
    lbfgs_params.mem_size = 256;
    lbfgs_params.past = 3;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.delta = 1.0e-5;

    int ret = lbfgs::lbfgs_optimize(x,
                                    minCost,
                                    &Smoother::costFunction,
                                    nullptr,
                                    nullptr,
                                    this,
                                    lbfgs_params);
    //std::cout << "ret:" << ret << std::endl;
    
    //std::cout << "minCost" << minCost << std::endl;

    for (int i = 2; i < smooth_path_.size() -2; ++i) {
            smooth_path_[i](0) = x(i-2);
            smooth_path_[i](1) = x(i-2 + points_num);
            smooth_path_[i-1](2) = std::atan2(smooth_path_[i](1) - smooth_path_[i-1](1),
                                         smooth_path_[i](0) - smooth_path_[i-1](0));
    }
    smooth_path_[smooth_path_.size() -3](2) = std::atan2(smooth_path_[smooth_path_.size() -2](1) - smooth_path_[smooth_path_.size() -3](1),
                                         smooth_path_[smooth_path_.size() -2](0) - smooth_path_[smooth_path_.size() -3](0));
    if (ret >= 0) {
        
        //std::cout << "smooth_path_[i-1](2)" << smooth_path_[i-1](2) << std::endl;
        
        std::cout << "Optimization success" << std::endl;
    } else {
        minCost = INFINITY;
        std::cout << "Optimization Failed: "
                    << lbfgs::lbfgs_strerror(ret)
                    << std::endl;
    }

    return minCost;
}

void Smoother::smoothPath(DynamicVoronoi &voronoi, std::vector<Vec3d> &path) {
    path_ = path;
    smooth_path_ = path;
    voronoi_ = voronoi;
    width_ = voronoi_.getSizeX();
    height_ = voronoi_.getSizeY();
    
    int iter = 0;
    int max_iter = 1000;

    double weight_sum = w_obs_ + w_cur_ + w_smo_ + w_vor_;

    while (iter < max_iter) {
        
        for (int i = 2; i < path_.size() - 2; ++i) {
            Vec2d x_p2(smooth_path_[i-2](0), smooth_path_[i-2](1));
            Vec2d x_p(smooth_path_[i-1](0), smooth_path_[i-1](1));
            Vec2d x_c(smooth_path_[i](0), smooth_path_[i](1));
            Vec2d x_n(smooth_path_[i+1](0), smooth_path_[i+1](1));
            Vec2d x_n2(smooth_path_[i+2](0), smooth_path_[i+2](1));

            Vec2d correction = Vec2d::Zero();

            correction = correction + calObstacleTerm(x_c);
            
            if (!isInMap(x_c - alpha_ * correction / weight_sum)) continue;

            
            correction = correction + calSmoothTerm(x_p2, x_p, x_c, x_n, x_n2);
            
            
            if (!isInMap(x_c - alpha_ * correction / weight_sum)) continue;


            x_c = x_c - alpha_ * correction / weight_sum;
            
            smooth_path_[i](0) = x_c(0);
            smooth_path_[i](1) = x_c(1);

            Vec2d delta_x = x_c - x_p;
            if (i > 1) {
                smooth_path_[i-1](2) = std::atan2(delta_x(1), delta_x(0));
            }
            
        }

        ++iter;
        
        
    }
    
    
    std::cout << iter << std::endl;


}
Vec2d Smoother::calObstacleTerm(Vec2d x) {
    Vec2d gradient;

    Vec2i x_i;
    x_i(0) = static_cast<int>((x(0) - (-25)) / 0.1);
    x_i(1) = static_cast<int>((x(1) - (-25)) / 0.1);

    //dist to the closet obstacle    unit m
    double dist2obs = 0.1 * voronoi_.getDistance(x_i(0), x_i(1));

    Vec2d vec_o2x(x_i(0) - voronoi_.data[x_i(0)][x_i(1)].obstX,
                  x_i(1) - voronoi_.data[x_i(0)][x_i(1)].obstY);
    
    if (dist2obs  < max_clearance_) {
        gradient = w_obs_ * 2 * (dist2obs - max_clearance_) / dist2obs * vec_o2x;
    } else {
        gradient = Vec2d::Zero();
    }
    return gradient;

}


Vec2d Smoother::calSmoothTerm(Vec2d x_p2, Vec2d x_p, Vec2d x_c, Vec2d x_n, Vec2d x_n2) {
    Vec2d gradient;
    gradient = w_smo_ * (x_p2 - 4 * x_p + 6 * x_c - 4 * x_n + x_n2);
    
    return gradient;
}

bool Smoother::isInMap(Vec2d x) {
    if (x(0) < -25 || x(1) < -25 || x(0) >= 25 || x(1) >= 25) {
        return false;
    }
    return true;
}

} // namespace planning


