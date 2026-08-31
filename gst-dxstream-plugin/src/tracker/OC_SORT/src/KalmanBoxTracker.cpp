#include <algorithm>
#include "../include/KalmanBoxTracker.hpp"
#include <utility>
namespace ocsort {
KalmanBoxTracker::KalmanBoxTracker(Eigen::VectorXf bbox_, int cls_, int idx_,
                                   uint64_t id_count_, int delta_t_) 
    : bbox(std::move(bbox_)),
      id(static_cast<int>(id_count_)),
      conf(bbox(4)),
      cls(cls_),
      idx(idx_),
      delta_t(delta_t_) {
    Eigen::Matrix<float, 7, 7> F_temp;
    F_temp << 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 1;
    kf->set_F(F_temp);

    Eigen::Matrix<float, 4, 7> H_temp;
    H_temp << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0;
    kf->set_H(H_temp);
    Eigen::Matrix<float, 4, 4> R_temp = kf->get_R();
    R_temp.block(2, 2, 2, 2) *= 10.0;
    kf->set_R(R_temp);

    Eigen::MatrixXf P_temp = kf->get_P();
    P_temp.block(4, 4, 3, 3) *= 1000.0;
    P_temp *= 10.0;
    kf->set_P(P_temp);

    Eigen::MatrixXf Q_temp = kf->get_Q();
    Q_temp.bottomRightCorner(1, 1)(0, 0) *= 0.01f;
    Q_temp.block(4, 4, 3, 3) *= 0.01f;
    kf->set_Q(Q_temp);

    Eigen::VectorXf x_temp = kf->get_x();
    x_temp.head<4>() = convert_bbox_to_z(bbox);
    kf->set_x(x_temp);
}

void KalmanBoxTracker::update(Eigen::VectorXf *bbox_, int cls_, int idx_) {
    if (bbox_ == nullptr) {
        Eigen::VectorXf tmp;
        kf->update(tmp);
        return;
    }

    conf = (*bbox_)[4];
    cls = cls_;
    idx = idx_;

    if (int(last_observation.sum()) >= 0) {
        Eigen::VectorXf previous_box_tmp;

        for (int i = 0; i < delta_t; ++i) {
            int dt = delta_t - i;
            if (observations.count(age - dt) > 0) {
                previous_box_tmp = observations[age - dt];
                break;
            }
        }

        if (previous_box_tmp.size() == 0) {
            previous_box_tmp = last_observation;
        }

        velocity = speed_direction(previous_box_tmp, *bbox_);
    }

    last_observation = *bbox_;
    observations[age] = *bbox_;

    // 다시 읽히지 않을 관측을 버린다.
    //
    // `observations` 를 읽는 곳은 코드 전체에서 셋뿐이고, 전부 **최근 delta_t 개**만 본다:
    //   ① 바로 위 이 함수의 `observations[age - dt]`  (dt = 1..delta_t)
    //   ② `k_previous_obs()` 의 `observations_.at(cur_age - dt)` (dt = 1..delta_t)
    //   ③ `k_previous_obs()` 의 폴백 `max_element` — 방금 넣은 이 항목이 최대 키다
    // `age` 는 단조 증가하므로 `age - delta_t` 보다 오래된 키는 **영원히 안 읽힌다.**
    // 업스트림(noahcao/OC_SORT)은 이것을 지우지 않는데, 30~60초짜리 MOT 클립에서는
    // 무해하기 때문이다. 24/7 파이프라인에서는 트랙 하나가 프레임마다 한 건씩 영원히
    // 쌓아 올린다 — 10fps 로 하루면 86만 건이다.
    //
    // 보존 개수는 **delta_t 로 정해진다** — 상수로 박으면 설정을 바꾼 순간 조용히 틀린다.
    // 음수 delta_t 는 설정 파일에서 막히지 않으므로(OCSort.cpp 의 stoi) 여기서 0으로 막는다;
    // 그러지 않으면 방금 넣은 항목까지 지워 폴백이 빈 맵을 훑게 된다.
    const int keep_from = age - std::max(0, delta_t);
    for (; oldest_obs_age < keep_from; ++oldest_obs_age)
        observations.erase(oldest_obs_age);
    time_since_update = 0;
    history.clear();
    hits += 1;
    hit_streak += 1;

    Eigen::VectorXf tmp = convert_bbox_to_z(*bbox_);
    kf->update(tmp);
}

Eigen::RowVectorXf KalmanBoxTracker::predict() {
    Eigen::VectorXf x_temp = kf->get_x();
    if (x_temp(6) + x_temp(2) <= 0)
        x_temp(6) *= 0.0f;
    kf->set_x(x_temp);
    kf->predict();
    age += 1;
    if (time_since_update > 0)
        hit_streak = 0;
    time_since_update += 1;
    history.push_back(convert_x_to_bbox(kf->get_x()));
    return convert_x_to_bbox(kf->get_x());
}
Eigen::VectorXf KalmanBoxTracker::get_state() const {
    return convert_x_to_bbox(kf->get_x());
}
} // namespace ocsort