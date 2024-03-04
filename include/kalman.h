// kalman filter shenanigans

template <typename Input, typename Output> class Controller {
    Output update(Input);
};

template <typename State>
class KalmanConfig {
  public:
    State initialEstimate;
    double modelCovariance;
    double sensorCovariance;
    double initialCovariance;
    double initialKalmanGainGuess;
};

template <typename Input, typename State>
class KalmanFilter : Controller<Input, State> {
  protected:
    Input prevInput;

  public:
    State update(Input in) {
      // do some math

      Input prevInput = in;
    };
};