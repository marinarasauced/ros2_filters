# This expresses the state and covariance estimates from a state
# estimation filter, such as a KF, EKF, UKF, etc.
#
# This message is mostly used by packages in the ros2_state_estimators
# repository. Values are stored in arrays due to the templated
# nature of the filter class definitions.

#
std_msgs/Header header

#
float64[] state
float64[] cov