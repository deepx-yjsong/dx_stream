#ifndef OC_SORT_CPP_KALMANBOXTRACKER_HPP
#define OC_SORT_CPP_KALMANBOXTRACKER_HPP
////////////// KalmanBoxTracker /////////////
#include "KalmanFilter.hpp"
#include "Utilities.hpp"
#include "iostream"
#include <memory>
/*
This class represents the internal state of individual
tracked objects observed as bbox.
*/
namespace ocsort {

class KalmanBoxTracker {
  public:
    /*method*/
    KalmanBoxTracker() = default;
    KalmanBoxTracker(Eigen::VectorXf bbox_, int cls_, int idx_,
                     uint64_t id_count_, int delta_t_ = 3);
    void update(Eigen::VectorXf *bbox_, int cls_, int idx_);
    Eigen::RowVectorXf predict();
    Eigen::VectorXf get_state() const;
    ~KalmanBoxTracker() = default;

    // Getters
    Eigen::RowVectorXf get_velocity() const { return velocity; }
    Eigen::RowVectorXf get_last_observation() const { return last_observation; }
    const std::unordered_map<int, Eigen::VectorXf>& get_observations() const { return observations; }
    int get_age() const { return age; }
    int get_time_since_update() const { return time_since_update; }
    int get_hit_streak() const { return hit_streak; }
    int get_id() const { return id; }
    int get_cls() const { return cls; }
    float get_conf() const { return conf; }
    int get_idx() const { return idx; }

  private:
    /*variable*/
    Eigen::VectorXf bbox; // [5,1]
    std::unique_ptr<KalmanFilterNew> kf = std::make_unique<KalmanFilterNew>(7, 4);
    int time_since_update = 0;
    int id;
    std::vector<Eigen::VectorXf> history;
    int hits = 0;
    int hit_streak = 0;
    int age = 0;
    float conf;
    int cls;
    int idx;
    // 업스트림의 `np.array([-1,-1,-1,-1,-1])` 이다 (ocsort.py KalmanBoxTracker.__init__).
    // **0 이 아니라 -1 이어야 한다.** 이 값은 두 곳에서 "아직 관측이 없다"의 표식으로 쓰인다:
    //   ① `update()` 의 `if (last_observation.sum() >= 0)` — 0 벡터면 첫 매칭에서 참이 되어
    //      원점에서 검출 중심으로 향하는 **가짜 속도**를 만든다. 업스트림은 여기서 속도를
    //      계산하지 않고 None 으로 두며, OCSort 가 (0,0) 으로 대체해 방향 항을 0 으로 만든다.
    //   ② OCSort 의 출력 선택 — 0 벡터면 "유효한 마지막 관측" 으로 읽혀 한 번도 갱신되지 않은
    //      트랙이 **(0,0,0,0) 상자**로 나간다. 업스트림은 그때 `get_state()` 를 쓴다.
    Eigen::RowVectorXf last_observation = Eigen::RowVectorXf::Constant(5, -1.0f);
    // 아직 지우지 않은 가장 오래된 관측 키. 잘라내기를 O(지운 개수)로 만든다.
    int oldest_obs_age = 0;
    std::unordered_map<int, Eigen::VectorXf> observations;
    // The reference implementation keeps a `history_observations` list alongside
    // `observations`; it is deliberately not ported. Its only consumer there is
    // Head Padding (noahcao/OC_SORT ocsort.py:416), which cannot be expressed in
    // this output contract: rows have no frame-offset column and their last field
    // indexes the CURRENT frame's detection list, while dxtracker edits buffers in
    // place and has already forwarded the earlier ones. Storing it here cost 20 MB
    // per 12-hour track at 10 fps and was never read. If Head Padding is ever
    // implemented, note that it reads only indices -2 and -3, so `min_hits`
    // entries suffice -- it does not need to be unbounded.
    Eigen::RowVectorXf velocity = Eigen::RowVectorXf::Zero(2); // [2,1]
    int delta_t;
};
} // namespace ocsort

#endif // OC_SORT_CPP_KALMANBOXTRACKER_HPP