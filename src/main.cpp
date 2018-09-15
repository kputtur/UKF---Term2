#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"
#include "tools.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // Create a Kalman Filter instance
  UKF ukf;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;



  h.onMessage([&ukf,&tools,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data));
      if (s != "") {
      	
        auto j = json::parse(s);

        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          string sensor_measurment = j[1]["sensor_measurement"];
          
          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);
    	  long long timestamp;

    	  // reads first element from the current line
    	  string sensor_type;
    	  iss >> sensor_type;

    	  if (sensor_type.compare("L") == 0) {
      	  		meas_package.sensor_type_ = MeasurementPackage::LASER;
          		meas_package.raw_measurements_ = VectorXd(2);
          		float px;
      	  		float py;
          		iss >> px;
          		iss >> py;
          		meas_package.raw_measurements_ << px, py;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
          } else if (sensor_type.compare("R") == 0) {

      	  		meas_package.sensor_type_ = MeasurementPackage::RADAR;
          		meas_package.raw_measurements_ = VectorXd(3);
          		float ro;
      	  		float theta;
      	  		float ro_dot;
          		iss >> ro;
          		iss >> theta;
          		iss >> ro_dot;
          		meas_package.raw_measurements_ << ro,theta, ro_dot;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
          }
          float x_gt;
    	  float y_gt;
    	  float vx_gt;
    	  float vy_gt;
    	  iss >> x_gt;
    	  iss >> y_gt;
    	  iss >> vx_gt;
    	  iss >> vy_gt;
    	  VectorXd gt_values(4);
    	  gt_values(0) = x_gt;
    	  gt_values(1) = y_gt; 
    	  gt_values(2) = vx_gt;
    	  gt_values(3) = vy_gt;
    	  ground_truth.push_back(gt_values);
          
          //Call ProcessMeasurment(meas_package) for Kalman filter
    	  ukf.ProcessMeasurement(meas_package);    	  

    	  //Push the current estimated x,y positon from the Kalman filter's state vector

    	  VectorXd estimate(4);

    	  double p_x = ukf.x_(0);
    	  double p_y = ukf.x_(1);
    	  double v  = ukf.x_(2);
    	  double yaw = ukf.x_(3);
          double yawd = ukf.x_(4);

    	  double v1 = cos(yaw)*v;
    	  double v2 = sin(yaw)*v;

    	  estimate(0) = p_x;
    	  estimate(1) = p_y;
    	  estimate(2) = v1;
    	  estimate(3) = v2;
    	  
	 
    	  ukf.vis_out_file << x_gt << "\t" ;  // p1
	  ukf.vis_out_file << y_gt << "\t" ;  // p2
	  ukf.vis_out_file << v << "\t" ;  
	  ukf.vis_out_file << yaw << "\t" ;
	  ukf.vis_out_file << yawd << "\t" ; 
	  ukf.vis_out_file << v1 << "\t" ;  // x_ v estimate - x component
	  ukf.vis_out_file << v2 << "\t" ;  // x_ v estimate - y component
	  ukf.vis_out_file << vx_gt << "\t" ;  // v1_gt
	  ukf.vis_out_file << vy_gt << "\t" ;  // v2_gt
	  ukf.vis_out_file << ukf.NIS_lidar_ << "\t" ;
	  ukf.vis_out_file << ukf.NIS_radar_ << "\t" ;
	  ukf.vis_out_file << ukf.P_(0, 0) << "\t" ;
	  ukf.vis_out_file << ukf.P_(1, 1) << "\t" ;
	  ukf.vis_out_file << ukf.P_(2, 2) << "\t" ;
	  ukf.vis_out_file << ukf.P_(3, 3) << "\t" ;
	  ukf.vis_out_file << ukf.P_(4, 4) << "\n" ;
 
	  estimations.push_back(estimate);
	  
    	  VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
	  

	  //print the values of p_x, p_y, v1 and v2
	  //printf("p_x = %lf p_y = %lf v1 = %lf  v2=%lf\n",p_x, p_y, v1, v2);
	  //printf("x_gt = %f y_gt = %f vx_gt = %f vy_gt=%f \n", x_gt, y_gt, vx_gt, vy_gt);
	  //cout<< "rmse_x = "<< to_string(RMSE(0))<< "rmse_y = "<< to_string(RMSE(1)) << "rmse_vx = "<< to_string(RMSE(2)) << "rmse_vy = "<< to_string(RMSE(4)) << endl;
	  //cout<<" rmse_x = "<<RMSE(0)<< " rmse_y = "<<RMSE(1)<<" rmse_vx ="<<RMSE(2)<<" rmse_vy ="<<RMSE(3)<<endl;

	  //Check if it meets or within the Rubric values
          /*
	  if ( RMSE(0) > 0.09 || RMSE(1) > 0.10 || RMSE(2) > 0.40 || RMSE(3) > 0.30 ) { 
	 	cout << "rmse_x failed to meet rubric spec"<<endl;
	 	printf("p_x = %lf p_y = %lf v1 = %lf  v2=%lf\n",p_x, p_y, v1, v2);
		cout<<" rmse_x = "<<RMSE(0)<< " rmse_y = "<<RMSE(1)<<" rmse_vx ="<<RMSE(2)<<" rmse_vy ="<<RMSE(3)<<endl; 
		
		} */

          json msgJson;
          msgJson["estimate_x"] = p_x;
          msgJson["estimate_y"] = p_y;
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	
	  // output values to  file: obj_pose-laser-radar-ukf-output.txt
	  /* 
  	  ofstream out;
  	  out.open("obj_pose-laser-radar-ukf-output.txt", ofstream::out | ofstream::app);
	  
          out << sensor_type << "\t";
          out << p_x << "\t";
          out << p_y << "\t";
          out << v1 << "\t";
          out << v2 << "\t";
        //  out << px_meas << "\t";
         // out << py_meas << "\t";
          out << x_gt << "\t";
          out << y_gt << "\t";
          out << vx_gt << "\t";
          out << vy_gt << "\t";
          
          // output the NIS values
          if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            std::cout << "NIS Laser" << "\n";
            std::cout << ukf.NIS_laser_ << "\n";
            out << ukf.NIS_laser_ << "\t";
          } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            std::cout << "NIS Radar" << "\n";
            std::cout << ukf.NIS_radar_ << "\n";
            out << ukf.NIS_radar_ << "\t";
          }
          
          out << RMSE(0) << "\t";
          out << RMSE(1) << "\t";
          out << RMSE(2) << "\t";
          out << RMSE(3) << "\t";
          
          out << scientific << ukf.acceleration_x_ << "\t";
          out << scientific << ukf.acceleration_y_ << "\t";
          out << ukf.dt_;
          
	  out << endl;
          std::flush(out);
	  out.close()
 	  */
	} 
       
      } else {
        
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }

  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();

}
