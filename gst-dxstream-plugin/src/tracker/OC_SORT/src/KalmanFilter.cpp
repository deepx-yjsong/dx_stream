#include "../include/KalmanFilter.hpp"
#include <iostream>
namespace ocsort {
KalmanFilterNew::KalmanFilterNew() : I(Eigen::MatrixXf::Identity(7, 7)) {};
KalmanFilterNew::KalmanFilterNew(int dim_x_, int dim_z_) : I(Eigen::MatrixXf::Identity(dim_x_, dim_x_)) {
    x = Eigen::VectorXf::Zero(dim_x_, 1);
    P = Eigen::MatrixXf::Identity(dim_x_, dim_x_);
    Q = Eigen::MatrixXf::Identity(dim_x_, dim_x_);
    B = Eigen::MatrixXf::Identity(dim_x_, dim_x_);
    F = Eigen::MatrixXf::Identity(dim_x_, dim_x_);
    H = Eigen::MatrixXf::Zero(dim_z_, dim_x_);
    R = Eigen::MatrixXf::Identity(dim_z_, dim_z_);
    M = Eigen::MatrixXf::Zero(dim_x_, dim_z_);
    z = Eigen::VectorXf::Zero(dim_z_, 1);
    K = Eigen::MatrixXf::Zero(dim_x_, dim_z_);
    y = Eigen::VectorXf::Zero(dim_x_, 1);
    S = Eigen::MatrixXf::Zero(dim_z_, dim_z_);
    SI = Eigen::MatrixXf::Zero(dim_z_, dim_z_);

    x_prior = x;
    P_prior = P;
    x_post = x;
    P_post = P;
};
void KalmanFilterNew::predict() {
    x = F * x;
    P = _alpha_sq * (F * P * F.transpose()) + Q;
    x_prior = x;
    P_prior = P;
}
void KalmanFilterNew::update(const Eigen::VectorXf &z_) {
    history_obs.push_back(z_);
    if (z_.size() == 0) {
        if (true == observed)
            freeze();
        observed = false;
        z = Eigen::VectorXf::Zero(H.rows(), 1);
        x_post = x;
        P_post = P;
        y = Eigen::VectorXf::Zero(H.rows(), 1);
        return;
    }
    if (false == observed)
        unfreeze();
    observed = true;
    y = z_ - H * x;
    auto PHT = P * H.transpose();
    S = H * PHT + R;
    K = PHT * SI;
    x = x + K * y;
    auto I_KH = I - K * H;
    P = ((I_KH * P) * I_KH.transpose()) + ((K * R) * K.transpose());
    z = z_;
    x_post = x;
    P_post = P;
}
void KalmanFilterNew::freeze() {
    attr_saved.IsInitialized = true;
    attr_saved.x = x;
    attr_saved.P = P;
    attr_saved.Q = Q;
    attr_saved.B = B;
    attr_saved.F = F;
    attr_saved.H = H;
    attr_saved.R = R;
    attr_saved._alpha_sq = _alpha_sq;
    attr_saved.M = M;
    attr_saved.z = z;
    attr_saved.K = K;
    attr_saved.y = y;
    attr_saved.S = S;
    attr_saved.SI = SI;
    attr_saved.x_prior = x_prior;
    attr_saved.P_prior = P_prior;
    attr_saved.x_post = x_post;
    attr_saved.P_post = P_post;
    attr_saved.history_obs_len = history_obs.size();
}
void KalmanFilterNew::unfreeze() {
    if (!attr_saved.IsInitialized) {
        return;
    }

    // 가상 궤적 생성에 필요한 것은 "가장 최근 실관측 두 건과 그 사이 프레임 수" 뿐이다.
    // 업스트림은 이 값을 리스트 별칭(new_history = self.history_obs,
    // noahcao/OC_SORT kalmanfilter.py:421)으로 읽고, 호출이 끝나면 별칭이 사라진다 —
    // 복사가 아니다. 이식은 이것을 멤버에 대한 전량 깊은 복사로 바꿔 트랙 수명 내내
    // 이력 사본을 하나 더 들고 있었다. 되감기 전에 제자리에서 읽으면 내용은 같고 복사는 없다.
    int lastNotNullIndex = -1;
    int secondLastNotNullIndex = -1;
    Eigen::VectorXf box1;
    Eigen::VectorXf box2;

    for (int i = static_cast<int>(history_obs.size()) - 1; i >= 0; --i) {
        if (history_obs[i].size() == 0) {
            continue;
        }
        if (lastNotNullIndex == -1) {
            lastNotNullIndex = i;
            box2 = history_obs[i];
        } else if (secondLastNotNullIndex == -1) {
            secondLastNotNullIndex = i;
            box1 = history_obs[i];
            break;
        }
    }

    x = attr_saved.x;
    P = attr_saved.P;
    Q = attr_saved.Q;
    B = attr_saved.B;
    F = attr_saved.F;
    H = attr_saved.H;
    R = attr_saved.R;
    _alpha_sq = attr_saved._alpha_sq;
    M = attr_saved.M;
    z = attr_saved.z;
    K = attr_saved.K;
    y = attr_saved.y;
    S = attr_saved.S;
    SI = attr_saved.SI;
    x_prior = attr_saved.x_prior;
    P_prior = attr_saved.P_prior;
    x_post = attr_saved.x_post;

    // 이력을 동결 시점 스냅샷에서 마지막 한 건을 뺀 상태로 되감는다(업스트림 :424 + :426).
    // 공백 구간에 쌓인 비관측 항목이 여기서 버려지므로 이력이 공백마다 늘어나지 않는다.
    // 스냅샷이 접두이므로(위 history_obs_len 주석의 불변식) resize 가 곧 되감기다 —
    // 복사도, 보관도 없다. 이식은 되감기 없이 현재 이력에서 한 건만 빼고 스냅샷을 영구
    // 보관해, 트랙 하나가 전량 이력을 셋(현재 + new_history + 스냅샷) 들고 있었다.
    // history_obs_len = 0 은 업스트림의 attr_saved = None(:425) 에 해당한다.
    if (attr_saved.history_obs_len > 0) {
        history_obs.resize(attr_saved.history_obs_len - 1);
    } else {
        history_obs.clear();
    }
    attr_saved.history_obs_len = 0;
    attr_saved.IsInitialized = false;

    if (lastNotNullIndex == -1 || secondLastNotNullIndex == -1) {
        return;
    }

    int time_gap = lastNotNullIndex - secondLastNotNullIndex;
    if (time_gap <= 0) {
        return;
    }

    double x1 = box1[0];
    double y1 = box1[1];
    double x2 = box2[0];
    double y2 = box2[1];

    double w1 = std::sqrt(box1[2] * box1[3]);
    double h1 = std::sqrt(box1[2] / box1[3]);
    double w2 = std::sqrt(box2[2] * box2[3]);
    double h2 = std::sqrt(box2[2] / box2[3]);

    double dx = (x2 - x1) / time_gap;
    double dy = (y2 - y1) / time_gap;
    double dw = (w2 - w1) / time_gap;
    double dh = (h2 - h1) / time_gap;

    for (int i = 0; i < time_gap; ++i) {
        double x_interp = x1 + (i + 1) * dx;
        double y_interp = y1 + (i + 1) * dy;
        double w_interp = w1 + (i + 1) * dw;
        double h_interp = h1 + (i + 1) * dh;

        double s = w_interp * h_interp;
        double r = w_interp / h_interp;

        Eigen::VectorXf new_box(4);
        new_box << static_cast<float>(x_interp), static_cast<float>(y_interp), static_cast<float>(s), static_cast<float>(r);

        y = new_box - H * x;
        Eigen::MatrixXf PHT = P * H.transpose();
        S = H * PHT + R;
        SI = S.inverse();
        K = PHT * SI;
        x = x + K * y;

        Eigen::MatrixXf I_KH = I - K * H;
        P = (I_KH * P) * I_KH.transpose() + (K * R) * K.transpose();

        z = new_box;
        x_post = x;
        P_post = P;

        if (i != time_gap - 1) {
            predict();
        }
    }
}

} // namespace ocsort