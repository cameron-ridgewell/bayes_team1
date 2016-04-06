#include <ros/ros.h>
#include <filter/extendedkalmanfilter.h>

#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>

#include <iostream>
#include <fstream>
#include <vector>
#include "std_msgs/Float32MultiArray.h"


using namespace MatrixWrapper;
using namespace BFL;
using namespace std;

static const float MU_SYSTEM_NOISE_X = 0;
static const float MU_SYSTEM_NOISE_Y = 0;
static const float SIGMA_SYSTEM_NOISE_X = 0;
static const float SIGMA_SYSTEM_NOISE_Y = 0;
static const float FOV_ANGLE = 66.666667;
static const float w = 1280;

float PRIOR_MU_X, PRIOR_MU_Y, PRIOR_COV_X, PRIOR_COV_Y = 1;

float d1 = 1.0;
float x_pos = 0.0;
float y_pos = 0.0;
float d_theta = 0.0;

class KalmanP
{
  ros::NodeHandle nh_;
  ros::Subscriber kalman_sub;
  ros::Publisher mean_pub;

  public:
  KalmanP()
  {
    // Subscribe to input video feed and publish output video feed
    kalman_sub = nh_.subscribe("/kalman_processor", 10, 
      &KalmanP::kalmanProcessor, this);
    mean_pub = nh_.advertise<std_msgs::Float32MultiArray>("/kalman_output", 100);
  }

  void kalmanProcessor(const std_msgs::Float32MultiArray::ConstPtr& msg)
  {
    if ((msg->data.size()) > 1)
    {
      d1 = msg->data[0];
      float d2 = msg->data[0]+msg->data[3];
      d_theta = msg->data[5];
      PRIOR_MU_X = msg->data[1] - w/2;
      PRIOR_MU_Y = msg->data[2] - 1024/2;

      // Create the matrices A and B for the linear system model
      Matrix A(2,2);
      A(1,1) = d2 / d1;
      A(1,2) = 0.0;
      A(2,1) = 0.0;
      A(2,2) = d2 / d1;
      Matrix B(2,2);
      B(1,1) = 0.0;
      B(1,2) = d_theta * w / FOV_ANGLE;
      B(2,1) = 0.0;
      B(2,2) = 0.0;

      vector<Matrix> AB(2);
      AB[0] = A;
      AB[1] = B;

      // create gaussian
      ColumnVector sysNoise_Mu(2);
      sysNoise_Mu(1) = MU_SYSTEM_NOISE_X;
      sysNoise_Mu(2) = MU_SYSTEM_NOISE_Y;

      SymmetricMatrix sysNoise_Cov(2);
      sysNoise_Cov = 0.0;
      sysNoise_Cov(1,1) = SIGMA_SYSTEM_NOISE_X;
      sysNoise_Cov(1,2) = 0.0;
      sysNoise_Cov(2,1) = 0.0;
      sysNoise_Cov(2,2) = SIGMA_SYSTEM_NOISE_Y;

      Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);

      // create the model
      LinearAnalyticConditionalGaussian sys_pdf(AB, system_Uncertainty);
      LinearAnalyticSystemModelGaussianUncertainty sys_model(&sys_pdf);

      /****************************
       * Linear prior DENSITY     *
       ***************************/
       // Continuous Gaussian prior (for Kalman filters)
      ColumnVector prior_Mu(2);
      prior_Mu(1) = PRIOR_MU_X;
      prior_Mu(2) = PRIOR_MU_Y;
      SymmetricMatrix prior_Cov(2);
      prior_Cov(1,1) = PRIOR_COV_X;
      prior_Cov(1,2) = 0.0;
      prior_Cov(2,1) = 0.0;
      prior_Cov(2,2) = PRIOR_COV_Y;
      Gaussian prior(prior_Mu,prior_Cov);

      /******************************
       * Construction of the Filter *
       ******************************/
      ExtendedKalmanFilter filter(&prior);

      ColumnVector input(2);
      input(1) = d2 - d1;
      input(2) = d_theta;

      filter.Update(&sys_model, input);

      Pdf<ColumnVector> * posterior = filter.PostGet();

      cout << " Posterior Mean = " << endl << posterior->ExpectedValueGet() << endl
           << " Covariance = " << endl << posterior->CovarianceGet() << "" << endl;
      PRIOR_COV_X = posterior->CovarianceGet()(1,1);
      PRIOR_COV_Y = posterior->CovarianceGet()(2,2);

      std_msgs::Float32MultiArray msg;
      std::vector<float> tmp;
      tmp.push_back(posterior->ExpectedValueGet()(1));
      tmp.push_back(posterior->ExpectedValueGet()(2));
      msg.data = tmp;
      mean_pub.publish(msg);
    }
  }
};

int main(int argc, char** argv)
{
  /****************************
   * Linear system model      *
   ***************************/
  ros::init(argc, argv, "wrench_linear_kalman");
  KalmanP kP;
  ros::spin();

  return 0;
}