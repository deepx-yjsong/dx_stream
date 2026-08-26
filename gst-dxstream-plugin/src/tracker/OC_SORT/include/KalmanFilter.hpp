#ifndef OC_SORT_CPP_KALMANFILTER_HPP
#define OC_SORT_CPP_KALMANFILTER_HPP
#include <eigen3/Eigen/Dense>
#include <map>
#include <vector>

namespace ocsort {
class KalmanFilterNew {
  public:
    KalmanFilterNew();
    KalmanFilterNew(int dim_x_, int dim_z_);
    void predict();
    void update(const Eigen::VectorXf &z_);
    void freeze();
    void unfreeze();
    KalmanFilterNew &operator=(const KalmanFilterNew &) = default;

    // Getters
    const Eigen::VectorXf& get_x() const { return x; }
    const Eigen::MatrixXf& get_P() const { return P; }
    const Eigen::MatrixXf& get_Q() const { return Q; }
    const Eigen::Matrix<float, 7, 7>& get_F() const { return F; }
    const Eigen::Matrix<float, 4, 7>& get_H() const { return H; }
    const Eigen::Matrix<float, 4, 4>& get_R() const { return R; }

    // Setters
    void set_F(const Eigen::Matrix<float, 7, 7>& F_) { F = F_; }
    void set_H(const Eigen::Matrix<float, 4, 7>& H_) { H = H_; }
    void set_R(const Eigen::Matrix<float, 4, 4>& R_) { R = R_; }
    void set_P(const Eigen::MatrixXf& P_) { P = P_; }
    void set_Q(const Eigen::MatrixXf& Q_) { Q = Q_; }
    void set_x(const Eigen::VectorXf& x_) { x = x_; }

  private:
    /// state: This is the Kalman state variable [7,1].
    Eigen::VectorXf x;
    // P: Covariance matrix. Initially declared as an identity matrix. Data type
    // is float. [7,7].
    Eigen::MatrixXf P;
    // Q: Process noise covariance matrix. [7,7].
    Eigen::MatrixXf Q;
    // B: Control matrix. Not used in target tracking. [n,n].
    Eigen::MatrixXf B;
    // F: Prediction matrix / state transition matrix. [7,7].
    Eigen::Matrix<float, 7, 7> F;
    // H: Observation model / matrix. [4,7].
    Eigen::Matrix<float, 4, 7> H;
    // R: Observation noise covariance matrix. [4,4].
    Eigen::Matrix<float, 4, 4> R;
    // _alpha_sq: Fading memory control, controlling the update weight. Float.
    float _alpha_sq = 1.0;
    // M: Measurement matrix, converting state vector x to measurement vector z.
    // [7,4]. It has the opposite effect of matrix H.
    Eigen::MatrixXf M;
    // z: Measurement vector. [4,1].
    Eigen::VectorXf z;
    /* The following variables are intermediate variables used in calculations
     */
    // K: Kalman gain. [7,4].
    Eigen::MatrixXf K;
    // y: Measurement residual. [4,1].
    Eigen::MatrixXf y;
    // S: Measurement residual covariance.
    Eigen::MatrixXf S;
    // SI: Transpose of measurement residual covariance (simplified for
    // subsequent calculations).
    Eigen::MatrixXf SI;
    // Identity matrix of size [dim_x,dim_x], used for convenient calculations.
    // This cannot be changed.
    const Eigen::MatrixXf I;
    // There will always be a copy of x, P after predict() is called.
    // If there is a need to assign values between two Eigen matrices, the
    // precondition is that they should be initialized properly, as this ensures
    // that the number of columns and rows are compatible.
    Eigen::VectorXf x_prior;
    Eigen::MatrixXf P_prior;
    // there will always be a copy of x,P after update() is called
    Eigen::VectorXf x_post;
    Eigen::MatrixXf P_post;
    // keeps all observations. When there is a 'z', it is directly pushed back.
    std::vector<Eigen::VectorXf> history_obs;
    // The following is newly added by ocsort.
    // Used to mark the tracking state (whether there is still a target matching
    // this trajectory), default value is false.
    bool observed = false;

    struct Data {
        Eigen::VectorXf x;
        Eigen::MatrixXf P;
        Eigen::MatrixXf Q;
        Eigen::MatrixXf B;
        Eigen::MatrixXf F;
        Eigen::MatrixXf H;
        Eigen::MatrixXf R;
        float _alpha_sq = 1.;
        Eigen::MatrixXf M;
        Eigen::VectorXf z;
        Eigen::MatrixXf K;
        Eigen::MatrixXf y;
        Eigen::MatrixXf S;
        Eigen::MatrixXf SI;
        Eigen::VectorXf x_prior;
        Eigen::MatrixXf P_prior;
        Eigen::VectorXf x_post;
        Eigen::MatrixXf P_post;
        // 동결 시점의 이력 길이. 업스트림은 여기서 리스트를 얕게 복사하지만
        // (list(self.history_obs), kalmanfilter.py:407 — 원소는 공유되고 포인터만 복사)
        // C++ 로 그대로 옮기면 Eigen 벡터 n개를 깊은 복사하게 되고, 깜빡이는 트랙에서는
        // 공백마다 그 비용을 다시 낸다(측정: 깜빡임 주기 4프레임에서 프레임당 비용이
        // 트랙 수명에 따라 3.18배 증가). 동결과 해동 사이 history_obs 는 추가만 되므로
        // (KalmanFilterNew::update 의 push_back 이 유일한 변경) 스냅샷은 접두
        // history_obs[0, history_obs_len) 과 정확히 같다 — 길이만 기억하면 복사가 없다.
        size_t history_obs_len = 0;
        bool observed = false;
        // The following is to determine whether the data has been saved due to
        // freezing.
        bool IsInitialized = false;
    };
    struct Data attr_saved;
};

} // namespace ocsort

#endif // OC_SORT_CPP_KALMANFILTER_HPP