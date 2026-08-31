#include "../include/Association.hpp"
#include <iomanip>
#include <iostream>

namespace ocsort {
std::tuple<Eigen::MatrixXf, Eigen::MatrixXf>
speed_direction_batch(const Eigen::MatrixXf &dets,
                      const Eigen::MatrixXf &tracks) {
    Eigen::VectorXf CX1 = (dets.col(0) + dets.col(2)) / 2.0;
    Eigen::VectorXf CY1 = (dets.col(1) + dets.col(3)) / 2.f;
    Eigen::MatrixXf CX2 = (tracks.col(0) + tracks.col(2)) / 2.f;
    Eigen::MatrixXf CY2 = (tracks.col(1) + tracks.col(3)) / 2.f;
    Eigen::MatrixXf dx = CX1.transpose().replicate(tracks.rows(), 1) -
                         CX2.replicate(1, dets.rows());
    Eigen::MatrixXf dy = CY1.transpose().replicate(tracks.rows(), 1) -
                         CY2.replicate(1, dets.rows());
    Eigen::MatrixXf norm =
        (dx.array().square() + dy.array().square()).sqrt() + 1e-6f;
    dx = dx.array() / norm.array();
    dy = dy.array() / norm.array();
    return std::make_tuple(dy, dx);
}
Eigen::MatrixXf iou_batch(const Eigen::MatrixXf &bboxes1,
                          const Eigen::MatrixXf &bboxes2) {
    Eigen::Matrix<float, Eigen::Dynamic, 1> a =
        bboxes1.col(0); // bboxes1[..., 0] (n1,1)
    Eigen::Matrix<float, 1, Eigen::Dynamic> b =
        bboxes2.col(0); // bboxes2[..., 0] (1,n2)
    Eigen::MatrixXf xx1 =
        (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));
    a = bboxes1.col(1); // bboxes1[..., 1]
    b = bboxes2.col(1); // bboxes2[..., 1]
    Eigen::MatrixXf yy1 =
        (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));
    a = bboxes1.col(2); // bboxes1[..., 2]
    b = bboxes2.col(2); // bboxes1[..., 2]
    Eigen::MatrixXf xx2 =
        (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
    a = bboxes1.col(3); // bboxes1[..., 3]
    b = bboxes2.col(3); // bboxes1[..., 3]
    Eigen::MatrixXf yy2 =
        (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
    Eigen::MatrixXf w = (xx2 - xx1).cwiseMax(0);
    Eigen::MatrixXf h = (yy2 - yy1).cwiseMax(0);
    Eigen::MatrixXf wh = w.array() * h.array();
    a = (bboxes1.col(2) - bboxes1.col(0)).array() *
        (bboxes1.col(3) - bboxes1.col(1)).array();
    b = (bboxes2.col(2) - bboxes2.col(0)).array() *
        (bboxes2.col(3) - bboxes2.col(1)).array();
    Eigen::MatrixXf part1_ = a.replicate(1, b.cols());
    Eigen::MatrixXf part2_ = b.replicate(a.rows(), 1);
    Eigen::MatrixXf Sum = part1_ + part2_ - wh;
    return wh.cwiseQuotient(Sum);
}

Eigen::MatrixXf giou_batch(const Eigen::MatrixXf &bboxes1,
                           const Eigen::MatrixXf &bboxes2) {
    Eigen::Matrix<float, Eigen::Dynamic, 1> a =
        bboxes1.col(0); // bboxes1[..., 0] (n1,1)
    Eigen::Matrix<float, 1, Eigen::Dynamic> b =
        bboxes2.col(0); // bboxes2[..., 0] (1,n2)
    Eigen::MatrixXf xx1 =
        (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));
    a = bboxes1.col(1); // bboxes1[..., 1]
    b = bboxes2.col(1); // bboxes2[..., 1]
    Eigen::MatrixXf yy1 =
        (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));
    a = bboxes1.col(2); // bboxes1[..., 2]
    b = bboxes2.col(2); // bboxes1[..., 2]
    Eigen::MatrixXf xx2 =
        (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
    a = bboxes1.col(3); // bboxes1[..., 3]
    b = bboxes2.col(3); // bboxes1[..., 3]
    Eigen::MatrixXf yy2 =
        (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
    Eigen::MatrixXf w = (xx2 - xx1).cwiseMax(0);
    Eigen::MatrixXf h = (yy2 - yy1).cwiseMax(0);
    Eigen::MatrixXf wh = w.array() * h.array();
    a = (bboxes1.col(2) - bboxes1.col(0)).array() *
        (bboxes1.col(3) - bboxes1.col(1)).array();
    b = (bboxes2.col(2) - bboxes2.col(0)).array() *
        (bboxes2.col(3) - bboxes2.col(1)).array();
    Eigen::MatrixXf part1_ = a.replicate(1, b.cols());
    Eigen::MatrixXf part2_ = b.replicate(a.rows(), 1);
    Eigen::MatrixXf Sum = part1_ + part2_ - wh;
    Eigen::MatrixXf iou = wh.cwiseQuotient(Sum);

    a = bboxes1.col(0);
    b = bboxes2.col(0);
    Eigen::MatrixXf xxc1 =
        (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
    a = bboxes1.col(1); // bboxes1[..., 1]
    b = bboxes2.col(1); // bboxes2[..., 1]
    Eigen::MatrixXf yyc1 =
        (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
    a = bboxes1.col(2); // bboxes1[..., 2]
    b = bboxes2.col(2); // bboxes1[..., 2]
    Eigen::MatrixXf xxc2 =
        (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));
    a = bboxes1.col(3); // bboxes1[..., 3]
    b = bboxes2.col(3); // bboxes1[..., 3]
    Eigen::MatrixXf yyc2 =
        (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));

    Eigen::MatrixXf wc = xxc2 - xxc1;
    Eigen::MatrixXf hc = yyc2 - yyc1;

    // 업스트림의 `assert((wc > 0).all() and (hc > 0).all())` 자리다 (association.py).
    // **사전조건 확인이지 분기가 아니다** — 이식본은 이것을 뒤집힌 if 로 옮겨,
    // 조건이 참인 정상 경로에서 GIoU 대신 **평범한 IoU 를 돌려주고** 있었다.
    // 유효한 상자라면 xxc2 > xxc1 이므로 조건은 사실상 항상 참이고, 결과적으로
    // 이 함수는 이름과 달리 IoU 함수였다. 퇴화 상자에서는 0 나눗셈이 되므로
    // 죽는 대신 IoU 로 물러난다 — GStreamer 파이프라인에서 abort 는 선택지가 아니다.
    if (!((wc.array() > 0).all() && (hc.array() > 0).all()))
        return iou;

    Eigen::MatrixXf area_enclose = wc.array() * hc.array();
    // 빼는 것은 **합집합**(위 `Sum`)이다. 이식본은 교집합(`wh`)을 빼고 있었다.
    Eigen::MatrixXf giou =
        iou.array() - (area_enclose.array() - Sum.array()) / area_enclose.array();
    giou = (giou.array() + 1) / 2.0;
    return giou;
}

void collectSimpleMatches(
    const Eigen::MatrixXf &a,
    Eigen::Matrix<float, Eigen::Dynamic, 2> &matched_indices) {
    for (int i = 0; i < a.rows(); ++i) {
        for (int j = 0; j < a.cols(); ++j) {
            if (a(i, j) > 0) {
                Eigen::RowVectorXf row(2);
                row << static_cast<float>(i), static_cast<float>(j);
                matched_indices.conservativeResize(matched_indices.rows() + 1,
                                                   Eigen::NoChange);
                matched_indices.row(matched_indices.rows() - 1) = row;
            }
        }
    }
}

void fillCostIouMatrix(const Eigen::MatrixXf &cost_matrix,
                       std::vector<std::vector<float>> &cost_iou_matrix) {
    for (int i = 0; i < cost_matrix.rows(); ++i) {
        for (int j = 0; j < cost_matrix.cols(); ++j) {
            cost_iou_matrix[i][j] = -cost_matrix(i, j);
        }
    }
}

void collectHungarianMatches(
    const std::vector<int> &rowsol, Eigen::Matrix<float, Eigen::Dynamic, 2> &matched_indices) {
    for (size_t i = 0; i < rowsol.size(); ++i) {
        if (rowsol[i] >= 0) {
            Eigen::RowVectorXf row(2);
            row << static_cast<float>(i), static_cast<float>(rowsol[i]);
            matched_indices.conservativeResize(matched_indices.rows() + 1, Eigen::NoChange);
            matched_indices.row(matched_indices.rows() - 1) = row;
        }
    }
}

std::tuple<std::vector<Eigen::Matrix<int, 1, 2>>, std::vector<int>,
           std::vector<int>>
associate(Eigen::MatrixXf detections, Eigen::MatrixXf trackers,
          float iou_threshold, Eigen::MatrixXf velocities,
          Eigen::MatrixXf previous_obs_, float vdc_weight) {

    if (trackers.rows() == 0) {
        std::vector<int> unmatched_dets(detections.rows());
        for (int i = 0; i < detections.rows(); ++i) {
            unmatched_dets[i] = i;
        }
        return std::make_tuple(std::vector<Eigen::Matrix<int, 1, 2>>(),
                               unmatched_dets, std::vector<int>());
    }

    Eigen::MatrixXf Y;
    Eigen::MatrixXf X;
    std::tie(Y, X) = speed_direction_batch(detections, previous_obs_);

    Eigen::MatrixXf inertia_Y = velocities.col(0);
    Eigen::MatrixXf inertia_X = velocities.col(1);
    Eigen::MatrixXf inertia_Y_ = inertia_Y.replicate(1, Y.cols());
    Eigen::MatrixXf inertia_X_ = inertia_X.replicate(1, X.cols());

    Eigen::MatrixXf diff_angle_cos =
        inertia_X_.array() * X.array() + inertia_Y_.array() * Y.array();
    diff_angle_cos = diff_angle_cos.array().min(1).max(-1).matrix();

    Eigen::MatrixXf diff_angle = Eigen::acos(diff_angle_cos.array());
    diff_angle = ((PI / 2.0f - diff_angle.array().abs()) / PI).matrix();

    Eigen::Array<bool, 1, Eigen::Dynamic> valid_mask =
        Eigen::Array<bool, Eigen::Dynamic, 1>::Ones(previous_obs_.rows());
    valid_mask *= (previous_obs_.col(4).array() >= 0).transpose();

    Eigen::MatrixXf iou_matrix = iou_batch(detections, trackers);

    // 검출 신뢰도. 업스트림은 5열 [x1,y1,x2,y2,score] 에서 `detections[:,-1]` 을 쓴다
    // (association.py `associate`). 이 이식본의 검출 행은 element 가 결과를 원래
    // object_meta 로 되돌려야 해서 열이 **둘** 늘어난 7열
    // [x1,y1,x2,y2,conf,label,input_idx] 인데(gst-dxtracker.cpp), 인덱스는 `-1 → -2` 로
    // **하나만** 밀려 있었다. 그래서 신뢰도가 아니라 **라벨**을 읽었고,
    // `angle_diff_cost = valid_mask * diff_angle * vdc_weight * scores` 이므로
    // **label == 0(우리 운영의 person)이면 항 전체가 0** 이 되어 OC-SORT 의
    // 관측 중심 방향 일치(OCM)가 통째로 꺼져 있었다. 최초 릴리스 커밋부터 그랬다.
    //
    // 이 파일 밖의 모든 접근은 절대 인덱스다(OCSort.cpp: col(4)=conf, (_,5)=cls,
    // (_,6)=input_idx). 여기만 상대 인덱스라 열이 늘 때 조용히 어긋났다 — 맞춰 둔다.
    Eigen::MatrixXf scores =
        detections.col(4).replicate(1, trackers.rows());

    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> valid_mask_ =
        valid_mask.transpose().replicate(1, X.cols());

    Eigen::MatrixXf angle_diff_cost =
        ((valid_mask_.cast<float>().array() * diff_angle.array()) * vdc_weight)
            .transpose()
            .array() *
        scores.array();

    Eigen::Matrix<float, Eigen::Dynamic, 2> matched_indices(0, 2);

    if (iou_matrix.rows() == 0 || iou_matrix.cols() == 0) {
        matched_indices.resize(0, 2);
    } else {
        Eigen::MatrixXf a = (iou_matrix.array() > iou_threshold).cast<float>();
        float sum1 = a.rowwise().sum().maxCoeff();
        float sum0 = a.colwise().sum().maxCoeff();

        if (std::abs(sum1 - 1) < 1e-12f && std::abs(sum0 - 1) < 1e-12f) {
            collectSimpleMatches(a, matched_indices);
        } else {
            Eigen::MatrixXf cost_matrix = iou_matrix + angle_diff_cost;
            std::vector<std::vector<float>> cost_iou_matrix(
                cost_matrix.rows(), std::vector<float>(cost_matrix.cols()));
            fillCostIouMatrix(cost_matrix, cost_iou_matrix);

            std::vector<int> rowsol;
            std::vector<int> colsol;
            execLapjv(cost_iou_matrix, rowsol, colsol, true, 0.01f, true);

            collectHungarianMatches(rowsol, matched_indices);
        }
    }

    std::vector<int> unmatched_detections;
    for (int i = 0; i < detections.rows(); ++i) {
        if ((matched_indices.col(0).array() == static_cast<float>(i)).sum() == 0) {
            unmatched_detections.push_back(i);
        }
    }

    std::vector<int> unmatched_trackers;
    for (int i = 0; i < trackers.rows(); ++i) {
        if ((matched_indices.col(1).array() == static_cast<float>(i)).sum() == 0) {
            unmatched_trackers.push_back(i);
        }
    }

    std::vector<Eigen::Matrix<int, 1, 2>> matches;
    for (int i = 0; i < matched_indices.rows(); ++i) {
        Eigen::Matrix<int, 1, 2> tmp = matched_indices.row(i).cast<int>();
        if (iou_matrix(tmp(0), tmp(1)) < iou_threshold) {
            unmatched_detections.push_back(tmp(0));
            unmatched_trackers.push_back(tmp(1));
        } else {
            matches.push_back(tmp);
        }
    }

    if (matches.empty()) {
        matches.clear();
    }

    return std::make_tuple(matches, unmatched_detections, unmatched_trackers);
}
} // namespace ocsort