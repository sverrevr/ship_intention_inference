#define DO_PLOT
//#define PLOT_BELIEVES
#define SPAM

#include "intention_model.h"
#include "cpu/kinematic_ship_models_cpu.h"
//#include "cpu/psbmpc_cpu.h"
//#include "cpu/utilities_cpu.h"
#include <Eigen/Dense>
#include <iostream>
#include "math.h"
#include <map>

#ifdef DO_PLOT
#include "engine.h"
#endif

//Trajectory er [x,y,chi,u]
int main()
{
    static constexpr enum { HO,
                            HO_WRONG,
                            HO_RISK,
                            CR_PRIO,
                            CR_PRIORS,
                            CR_OT,
                            CR_GW_WRONG,
                            OT,
                            OT_AND_HO,
                            risk_of_collison } situation = CR_PRIO;

    //Lag 3 skip
    //const double T_U, const double  T_chi, const double R_a, const double LOS_LD, const double LOS_K_i
    PSBMPC_LIB::CPU::Obstacle_Ship shipModel(10, 50, 30, 600, 0);

    double simulation_dt = 0.5;
    double simualtion_display_step = 120; //number of timesteps
    double simulation_T = 800;
    size_t simulation_N = std::round(simulation_T / simulation_dt);

    double prediction_dt = 0.5;
    //double prediction_T = 800;
    //double prediction_T = 1000;
    double prediction_T = 2200;
    size_t prediction_N = std::round(prediction_T / prediction_dt);
    struct offset
    {
        double angle;
        double vel;
    };
    const std::vector<offset> offsets = {(offset){-90 * DEG2RAD, 1}, (offset){-60 * DEG2RAD, 1}, (offset){-30 * DEG2RAD, 1}, (offset){0, 1}, (offset){30 * DEG2RAD, 1}, (offset){60 * DEG2RAD, 1}, (offset){90 * DEG2RAD, 1}, (offset){0, 0.5}};
    //const std::vector<offset> offsets = {(offset){-90*DEG2RAD,1},(offset){-60*DEG2RAD,1}, (offset){-30*DEG2RAD,1}, (offset){0,1}, (offset){30*DEG2RAD,1}, (offset){60*DEG2RAD,1},(offset){90*DEG2RAD,1}};
    //const std::vector<offset> offsets = {(offset){-90*DEG2RAD,1},(offset){-60*DEG2RAD,1}, (offset){0,1}, (offset){60*DEG2RAD,1}, (offset){90*DEG2RAD,1},(offset){0,0.5}};
    //const std::vector<offset> offsets = {(offset){0,1}};

    std::vector<Eigen::MatrixXd> observed_trajectories;
    observed_trajectories.push_back(Eigen::MatrixXd(4, simulation_N));
    observed_trajectories.push_back(Eigen::MatrixXd(4, simulation_N));

    //Generate true trajectories:
    std::vector<Eigen::VectorXd> ship_offsets;
    std::vector<Eigen::VectorXd> ship_offset_times;

    std::vector<std::map<std::string, std::string>> priors;
    priors.push_back({{"relative_ship_size_to_ship0", "similar"}});
    priors.push_back({{"relative_ship_size_to_ship0", "similar"}});

    switch (situation)
    {
    case CR_OT:
        observed_trajectories[0].col(0) << 0, 0, 90 * DEG2RAD, 3;         //ot/cr
        observed_trajectories[1].col(0) << 2300, -1000, 115 * DEG2RAD, 4; //ot/cr
        //SHip1
        ship_offsets.push_back(Eigen::VectorXd(4));
        ship_offset_times.push_back(Eigen::VectorXd(2));
        ship_offsets[0] << 1, 0, 1, 0 * DEG2RAD; //U, chi
        ship_offset_times[0] << 0, 90;
        //Ship2
        ship_offsets.push_back(Eigen::VectorXd(4));
        ship_offset_times.push_back(Eigen::VectorXd(2));
        ship_offsets[1] << 1, 0, 1, 60 * DEG2RAD; //U, chi
        ship_offset_times[1] << 0, 90;
        break;
    case OT:
        observed_trajectories[0].col(0) << 0, 0, 0 * DEG2RAD, 5;
        observed_trajectories[1].col(0) << 500, 0, 0 * DEG2RAD, 2;
        //Ship1
        ship_offsets.push_back(Eigen::VectorXd(4));
        ship_offset_times.push_back(Eigen::VectorXd(2));
        ship_offsets[0] << 1, 0, 1, -30 * DEG2RAD; //U, chi
        ship_offset_times[0] << 0, 60;
        //Ship2
        ship_offsets.push_back(Eigen::VectorXd(4));
        ship_offset_times.push_back(Eigen::VectorXd(2));
        ship_offsets[1] << 1, 0, 1, 00 * DEG2RAD; //U, chi
        ship_offset_times[1] << 0, 60;

        break;
    case HO:
        observed_trajectories[0].col(0) << 0, 0, 0 * DEG2RAD, 5;
        observed_trajectories[1].col(0) << 5000, -30, 180 * DEG2RAD, 5;
        //Ship1
        ship_offsets.push_back(Eigen::VectorXd(4));
        ship_offset_times.push_back(Eigen::VectorXd(2));
        ship_offsets[0] << 1, 0, 1, -30 * DEG2RAD; //U, chi
        ship_offset_times[0] << 0, 50;
        //Ship2
        ship_offsets.push_back(Eigen::VectorXd(4));
        ship_offset_times.push_back(Eigen::VectorXd(2));
        ship_offsets[1] << 1, 0, 1, -30 * DEG2RAD; //U, chi
        ship_offset_times[1] << 0, 50;
        break;
    case HO_RISK:
        observed_trajectories[0].col(0) << 0, 850, 0 * DEG2RAD, 5;
        observed_trajectories[1].col(0) << 3000, 0, 180 * DEG2RAD, 5;
        //Ship1
        ship_offsets.push_back(Eigen::VectorXd(2));
        ship_offset_times.push_back(Eigen::VectorXd(1));
        ship_offsets[0] << 1, 0; //U, chi
        ship_offset_times[0] << 0;
        //Ship2
        ship_offsets.push_back(Eigen::VectorXd(2));
        ship_offset_times.push_back(Eigen::VectorXd(1));
        ship_offsets[1] << 1, 0; //U, chi
        ship_offset_times[1] << 0;
        break;
    case HO_WRONG:
        observed_trajectories[0].col(0) << 0, 0, 0 * DEG2RAD, 5;
        observed_trajectories[1].col(0) << 5000, -30, 180 * DEG2RAD, 5;
        //Ship1
        ship_offsets.push_back(Eigen::VectorXd(4));
        ship_offset_times.push_back(Eigen::VectorXd(2));
        ship_offsets[0] << 1, 0, 1, 30 * DEG2RAD; //U, chi
        ship_offset_times[0] << 0, 50;
        //Ship2
        ship_offsets.push_back(Eigen::VectorXd(4));
        ship_offset_times.push_back(Eigen::VectorXd(2));
        ship_offsets[1] << 1, 0, 1, 30 * DEG2RAD; //U, chi
        ship_offset_times[1] << 0, 50;
        break;
    case CR_PRIO:
        observed_trajectories[0].col(0) << 0, 0, 0 * DEG2RAD, 5;
        observed_trajectories[1].col(0) << 2500, -2600, 90 * DEG2RAD, 5;

        //Ship1
        ship_offsets.push_back(Eigen::VectorXd(2));
        ship_offset_times.push_back(Eigen::VectorXd(1));
        ship_offsets[0] << 1, 0; //U, chi
        ship_offset_times[0] << 0;

        //Ship2
        ship_offsets.push_back(Eigen::VectorXd(6));
        ship_offset_times.push_back(Eigen::VectorXd(3));
        ship_offsets[1] << 1, 0, 0.5, -85 * DEG2RAD, 1, 0; //U, chi
        ship_offset_times[1] << 0, 360, 650;
        break;
    case CR_PRIORS:
        observed_trajectories[0].col(0) << 0, 0, 0 * DEG2RAD, 5;
        observed_trajectories[1].col(0) << 2500, -2600, 90 * DEG2RAD, 5;

        //Ship1
        ship_offsets.push_back(Eigen::VectorXd(2));
        ship_offset_times.push_back(Eigen::VectorXd(1));
        ship_offsets[0] << 1, 0; //U, chi
        ship_offset_times[0] << 0;
        priors[0]["relative_ship_size_to_ship0"] = "larger";

        //Ship2
        ship_offsets.push_back(Eigen::VectorXd(6));
        ship_offset_times.push_back(Eigen::VectorXd(3));
        ship_offsets[1] << 1, 0, 0.5, -85 * DEG2RAD, 1, 0; //U, chi
        ship_offset_times[1] << 0, 320, 650;
        priors[1]["relative_ship_size_to_ship0"] = "smaller";
        break;
    case CR_GW_WRONG:
        observed_trajectories[0].col(0) << 0, 100, 0 * DEG2RAD, 5;
        observed_trajectories[1].col(0) << 1500, -1800, 90 * DEG2RAD, 5;

        //Ship1
        ship_offsets.push_back(Eigen::VectorXd(6));
        ship_offset_times.push_back(Eigen::VectorXd(3));
        ship_offsets[0] << 1, 0, 1, 45 * DEG2RAD, 1, 0; //U, chi
        ship_offset_times[0] << 0, 200, 500;

        //Ship2
        ship_offsets.push_back(Eigen::VectorXd(2));
        ship_offset_times.push_back(Eigen::VectorXd(1));
        ship_offsets[1] << 1, 0; //U, chi
        ship_offset_times[1] << 0;
        break;
    case OT_AND_HO:
        observed_trajectories.push_back(Eigen::MatrixXd(4, simulation_N));
        observed_trajectories[0].col(0) << 0, 0, 0 * DEG2RAD, 3;
        observed_trajectories[1].col(0) << 750, 0, 0 * DEG2RAD, 2;
        observed_trajectories[2].col(0) << 2300, 0, 180 * DEG2RAD, 2;

        ship_offsets.push_back(Eigen::VectorXd(4));
        ship_offset_times.push_back(Eigen::VectorXd(2));
        ship_offsets[0] << 1, 0, 1, 45 * DEG2RAD; //U, chi
        ship_offset_times[0] << 0, 100;

        ship_offsets.push_back(Eigen::VectorXd(4));
        ship_offset_times.push_back(Eigen::VectorXd(2));
        ship_offsets[1] << 1, 0, 1, 0 * DEG2RAD; //U, chi
        ship_offset_times[1] << 0, 100;

        ship_offsets.push_back(Eigen::VectorXd(4));
        ship_offset_times.push_back(Eigen::VectorXd(2));
        ship_offsets[2] << 1, 0, 1, 45 * DEG2RAD; //U, chi
        ship_offset_times[2] << 0, 100;
        priors.push_back({{"relative_ship_size_to_ship0", "similar"}});
        priors[0]["relative_ship_size_to_ship1"] = "similar";
        priors[1]["relative_ship_size_to_ship1"] = "similar";
        priors[2]["relative_ship_size_to_ship1"] = "similar";

        break;
    case risk_of_collison:
        observed_trajectories[0].col(0) << 0, 1100, 0 * DEG2RAD, 3;
        observed_trajectories[1].col(0) << 7000, 0, -180 * DEG2RAD, 3;

        //Ship1
        ship_offsets.push_back(Eigen::VectorXd(4));
        ship_offset_times.push_back(Eigen::VectorXd(2));
        ship_offsets[0] << 1, 0, 1, -70 * DEG2RAD; //U, chi
        ship_offset_times[0] << 0, 50;

        //Ship2
        ship_offsets.push_back(Eigen::VectorXd(4));
        ship_offset_times.push_back(Eigen::VectorXd(2));
        ship_offsets[1] << 1, 0, 1, -70 * DEG2RAD; //U, chi
        ship_offset_times[1] << 0, 50;
    default:
        break;
    }

    std::vector<size_t> ship_list;
    for (size_t ship = 0; ship < observed_trajectories.size(); ++ship)
    {
        ship_list.push_back(ship);
    }

    std::vector<Eigen::Matrix2Xd> waypoint_list;
    for (auto ship : observed_trajectories)
    {
        waypoint_list.push_back(Eigen::Matrix2Xd(2, 2));
        waypoint_list.back().col(0) = ship.col(0).head(2);
        Eigen::Vector2d velocity;
        velocity(0) = std::cos(ship(CHI, 0)) * ship(U, 0);
        velocity(1) = std::sin(ship(CHI, 0)) * ship(U, 0);
        waypoint_list.back().col(1) = ship.col(0).head(2) + velocity * 2 * prediction_T;
    }
    for (size_t ship = 0; ship < observed_trajectories.size(); ++ship)
    {
        shipModel.predict_trajectory(observed_trajectories[ship],
                                     ship_offsets[ship], ship_offset_times[ship],
                                     observed_trajectories[ship].col(0)(U), observed_trajectories[ship].col(0)(CHI),
                                     waypoint_list[ship],
                                     PSBMPC_LIB::ERK1, PSBMPC_LIB::LOS,
                                     simulation_T, simulation_dt);
    }

    std::vector<IntentionModel> models;
    for (size_t ship = 0; ship < observed_trajectories.size(); ++ship)
    {
        models.push_back(IntentionModel(ship, ship_list, priors[ship]));
    }

#ifdef DO_PLOT
    Engine *ep = engOpen(NULL);
    if (ep == NULL)
    {
        std::cout << "engine start failed!" << std::endl;
        return 1;
    }
#endif

    bool do_stop = false;
    for (long t = 0; !do_stop; t += simualtion_display_step)
    {
        std::vector<Eigen::Vector4d> current_ship_states;
        for (auto ship : observed_trajectories)
        {
            if (ship.cols() <= std::round(t / simulation_dt))
            {
                do_stop = true;
                break;
            }
            current_ship_states.push_back(ship.col(std::round(t / simulation_dt)));
        }
        if (do_stop)
            break;
        std::cout << "\nT = " << t << std::endl;

#ifdef DO_PLOT
        //Plot trajectory in matlab
        engEvalString(ep, "figure(1); clf; hold on; axis equal;");
        //engEvalString(ep, "clf");
        //engEvalString(ep, "hold on");
        // for(size_t ship=0; ship<observed_trajectories.size(); ++ship){
        //     mxArray* true_trajectory_mx = mxCreateDoubleMatrix(observed_trajectories[ship].rows(),observed_trajectories[ship].cols(),mxREAL);
        //     double* p_true_trajectory_mx = mxGetDoubles(true_trajectory_mx);
        //     Eigen::Map<Eigen::MatrixXd> map_true_trajectory(p_true_trajectory_mx,observed_trajectories[ship].rows(),observed_trajectories[ship].cols());
        //     map_true_trajectory = observed_trajectories[ship];
        //     engPutVariable(ep,"true_trajectory",true_trajectory_mx);
        //     std::string str ="plot(true_trajectory(1,:),true_trajectory(2,:),'SeriesIndex',"+std::to_string(ship+1)+")";
        //     engEvalString(ep,str.c_str());
        //     engEvalString(ep, "hold on");
        //     mxDestroyArray(true_trajectory_mx);
        // }
#endif

        //Insert observation
        for (size_t ship = 0; ship < observed_trajectories.size(); ++ship)
        {
            models[ship].insertObservation(current_ship_states);
        }

#ifdef SPAM
        for (size_t ship = 0; ship < observed_trajectories.size(); ++ship)
        {
            auto ship_intentions = models[ship].getNodeOfInterest();

            std::cout << "\nShip" << ship << " - Intentions\n";
            for (auto node : ship_intentions)
            {
                std::cout << node.first << ": ";
                for (auto result : node.second)
                {
                    std::cout << result.first << "=" << result.second << ", ";
                }
                std::cout << "\n";
            }

#ifdef DO_PLOT
#ifdef PLOT_BELIEVES
            size_t max_number_of_outputs = 0;
            for (auto node : ship_intentions)
            {
                if (node.second.size() > max_number_of_outputs)
                {
                    max_number_of_outputs = node.second.size();
                }
            }

            mxArray *name_mx;
            engEvalString(ep, "name_list = strings");

            Eigen::MatrixXd intentions_matrix = Eigen::MatrixXd::Zero(ship_intentions.size(), max_number_of_outputs);
            size_t node_itterator = 0;
            for (auto node : ship_intentions)
            {
                size_t output_itterator = 0;

                name_mx = mxCreateString(node.first.c_str());
                engPutVariable(ep, "name", name_mx);
                engEvalString(ep, "name_list(end+1) = join(name,\"\")");

                for (auto result : node.second)
                {
                    intentions_matrix(node_itterator, output_itterator) = result.second;
                    ++output_itterator;
                }
                ++node_itterator;
                mxDestroyArray(name_mx);
            }
            mxArray *mx = mxCreateDoubleMatrix(ship_intentions.size(), max_number_of_outputs, mxREAL);
            double *p_mx = mxGetDoubles(mx);
            Eigen::Map<Eigen::MatrixXd> map(p_mx, ship_intentions.size(), max_number_of_outputs);
            map = intentions_matrix;
            std::string str = "figure(" + std::to_string(ship + 2) + ")";
            engEvalString(ep, str.c_str());
            engPutVariable(ep, "intention_values", mx);
            engEvalString(ep, "bar(intention_values,'stacked')");
            engEvalString(ep, "name_list =name_list(2:end)");
            engEvalString(ep, "xticklabels(name_list), xtickangle(20), ylim([0,1])");
            mxDestroyArray(mx);

            node_itterator = 0;
            for (auto node : ship_intentions)
            {
                size_t output_itterator = 0;
                double y_val = 0;

                for (auto result : node.second)
                {
                    y_val += result.second;
                    std::string str = "text(" + std::to_string(node_itterator + 0.7) + "," + std::to_string(y_val - result.second / 2) + ",'" + result.first + "')";
                    engEvalString(ep, str.c_str());
                    ++output_itterator;
                }
                ++node_itterator;
            }
#endif
#endif
        }
#endif

        for (size_t ship = 0; ship < observed_trajectories.size(); ++ship)
        {
            std::cout << "\nShip: " << ship << std::endl;
            for (auto offset : offsets)
            {
                std::cout << "-- Offset " << std::to_string(offset.angle) << ", " << std::to_string(offset.vel) << " --" << std::endl;
                //Generate trajectory
                Eigen::MatrixXd prediction(4, prediction_N);
                prediction.col(0) = current_ship_states[ship];
                Eigen::VectorXd offset_sequence(2);
                offset_sequence << offset.vel, offset.angle; //U, chi
                Eigen::VectorXd offset_times(1);
                offset_times << 0;

                shipModel.predict_trajectory(prediction,
                                             offset_sequence, offset_times,
                                             current_ship_states[ship](U), current_ship_states[ship](CHI),
                                             waypoint_list[ship],
                                             PSBMPC_LIB::ERK1, PSBMPC_LIB::LOS,
                                             prediction_T, prediction_dt);

                //Evaluate trajecotory
                double probability_of_trajecotry_legality = models[ship].evaluateTrajectory(prediction, current_ship_states, prediction_dt);

                //Print trajectory and probability
                std::cout << "Ligality: " << probability_of_trajecotry_legality << std::endl;
#ifdef SPAM
                auto ship_intentions = models[ship].getNodeOfInterest();
                for (auto node : ship_intentions)
                {
                    std::cout << "\t\t" << node.first << ": ";
                    for (auto result : node.second)
                    {
                        std::cout << result.first << "=" << result.second << ", ";
                    }
                    std::cout << "\n";
                }
#endif

#ifdef DO_PLOT
                mxArray *planned_trajectory_mx = mxCreateDoubleMatrix(prediction.rows(), prediction.cols(), mxREAL);
                double *p_planned_trajectory_mx = mxGetDoubles(planned_trajectory_mx);
                Eigen::Map<Eigen::MatrixXd> map_planned_trajectory(p_planned_trajectory_mx, prediction.rows(), prediction.cols());
                map_planned_trajectory = prediction;
                engPutVariable(ep, "planned_trajectory", planned_trajectory_mx);
                std::string str = "figure(1);";
                //str += "colorvec=[25, 118, 180; 255 127 0; 39 160 42]/255;";
                std::string marker = "--";
                if (offset.vel == 0.5)
                {
                    str += "colorvec=[0, 0.4470, 0.7410; 0.8500, 0.3250, 0.0980; 0.4660, 0.6740, 0.1880]*0.5+0.5;";
                }
                else
                {
                    str += "colorvec=[0, 0.4470, 0.7410; 0.8500, 0.3250, 0.0980; 0.4660, 0.6740, 0.1880];";
                }
                str += "chosen_color = colorvec(" + std::to_string(ship + 1) + ");";
                str += "plot(planned_trajectory(1,:),planned_trajectory(2,:),'" + marker + "','Color', colorvec(" + std::to_string(ship + 1) + ",:), 'LineWidth'," + std::to_string(4 * probability_of_trajecotry_legality + 0.0001) + ");";
                //str+= "plot(planned_trajectory(1,end),planned_trajectory(2,end),'|','SeriesIndex',"+std::to_string(ship+1)+", 'LineWidth',"+std::to_string(4*probability_of_trajecotry_legality+0.0001)+");";
                str += "text(planned_trajectory(1,end),planned_trajectory(2,end),'" + std::to_string((int)std::round(probability_of_trajecotry_legality * 100)) + "%');";
                str += "xlabel('m'); ylabel('m');";
                engEvalString(ep, str.c_str());
                mxDestroyArray(planned_trajectory_mx);
#endif
            }
        }
#ifdef DO_PLOT
        std::string ship_plot_string = "";
        //std::vector<std::string> ship_colors{"[0, 0.4470, 0.7410]", "[0.8500, 0.3250, 0.0980]", "[0.4660, 0.6740, 0.1880]"};
        std::vector<std::string> ship_colors{"[0, 0.4470, 0.7410]*0.5+0.5", "[0.8500, 0.3250, 0.0980]*0.5+0.5", "[0.4660, 0.6740, 0.1880]*0.5+0.5"};
        const std::string scale = "175";
        const std::string alpha = "1";
        for (size_t ship = 0; ship < current_ship_states.size(); ++ship)
        {
            ship_plot_string += "ship_plotter(" + std::to_string(current_ship_states[ship](PX)) + "," +
                                std::to_string(current_ship_states[ship](PY)) + "," +
                                std::to_string(current_ship_states[ship](CHI) * RAD2DEG) + "," +
                                scale + "," + ship_colors[ship] + "," + alpha + ");";
        }
        engEvalString(ep, ship_plot_string.c_str());
#endif
        //wait for input to continue
        std::getchar();
    }
#ifdef DO_PLOT
    engClose(ep);
#endif

    return 0;
}
