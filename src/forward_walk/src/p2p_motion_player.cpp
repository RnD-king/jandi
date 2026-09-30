#include "p2p_motion_player.hpp"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <utility>

P2PMotionPlayer::P2PMotionPlayer(Dxl* dxl) : dxl_(dxl) {}

// motion JSON을 읽어서 실행에 필요한 keyframe으로 변환한다.
P2PMotionPlayer::Motion P2PMotionPlayer::LoadMotion(
    const std::filesystem::path& json_path) const
{
    using boost::property_tree::ptree;
    ptree root;
    boost::property_tree::read_json(json_path.string(), root);

    Motion motion;
    motion.name = root.get<std::string>("name", json_path.string());

    std::vector<int> motor_ids;
    if (const auto ids = root.get_child_optional("motor_ids")) {
        for (const auto& entry : *ids) {
            motor_ids.push_back(entry.second.get_value<int>());
        }
    }

    if (const auto gains = root.get_child_optional("pd_gains")) {
        const auto parse_gain = [&](const ptree& source,
                                    const std::string& label) {
            const int p_gain = source.get<int>("p_gain");
            const int d_gain = source.get<int>("d_gain");
            if (p_gain < 0 || p_gain > 16383 ||
                d_gain < 0 || d_gain > 16383) {
                throw std::runtime_error(
                    "PD gain is outside 0..16383 at " + label);
            }
            return PositionPDGain{
                static_cast<std::uint16_t>(p_gain),
                static_cast<std::uint16_t>(d_gain)};
        };

        if (const auto default_gain = gains->get_child_optional("default")) {
            const PositionPDGain value =
                parse_gain(*default_gain, "pd_gains.default");
            for (const int motor_id : motor_ids) {
                motion.pd_gains[motor_id] = value;
            }
        }
        for (const auto& entry : *gains) {
            if (entry.first == "default") continue;
            const int motor_id = std::stoi(entry.first);
            motion.pd_gains[motor_id] =
                parse_gain(entry.second, "pd_gains." + entry.first);
        }
    }

    const auto frames = root.get_child_optional("keyframes");
    if (!frames) {
        throw std::runtime_error(
            "motion JSON has no keyframes array: " + json_path.string());
    }

    for (const auto& frame_entry : *frames) {
        const ptree& source = frame_entry.second;
        Keyframe frame;
        frame.name = source.get<std::string>("name", "unnamed");
        frame.duration_sec = source.get<double>("duration_sec", 1.0);
        frame.hold_sec = source.get<double>("hold_sec", 0.0);
        frame.max_speed_deg_s = source.get<double>("max_speed_deg_s", 30.0);

        const std::string kind =
            source.get<std::string>("interpolation", "smoothstep");
        if (kind == "linear") frame.interpolation = Interpolation::kLinear;
        else if (kind == "smoothstep") frame.interpolation = Interpolation::kSmoothStep;
        else if (kind == "minimum_jerk") frame.interpolation = Interpolation::kMinimumJerk;
        else throw std::runtime_error("unknown interpolation: " + kind);

        if (frame.duration_sec <= 0.0 || frame.hold_sec < 0.0 ||
            frame.max_speed_deg_s <= 0.0) {
            throw std::runtime_error("invalid timing/speed in keyframe: " + frame.name);
        }

        const auto positions = source.get_child_optional("positions");
        if (!positions) throw std::runtime_error("keyframe has no positions: " + frame.name);
        for (const auto& position : *positions) {
            frame.positions.emplace(std::stoi(position.first),
                                    position.second.get_value<int32_t>());
        }
        motion.keyframes.push_back(std::move(frame));
    }
    if (motion.keyframes.empty()) {
        throw std::runtime_error(
            "motion JSON contains no keyframes: " + json_path.string());
    }
    return motion;
}

// program JSON이 motion이면 하나만, mission이면 참조하는 motion 전체를
// 미리 파싱한다. 그래야 중간 파일 오타를 로봇이 움직인 뒤에 발견하지 않는다.
void P2PMotionPlayer::LoadProgram(
    const std::filesystem::path& json_path,
    const std::filesystem::path& motion_directory)
{
    using boost::property_tree::ptree;
    ptree root;
    boost::property_tree::read_json(json_path.string(), root);

    motions_.clear();
    if (root.get_child_optional("keyframes")) {
        motions_.push_back(LoadMotion(json_path));
        return;
    }

    const auto mission_motions = root.get_child_optional("motions");
    if (!mission_motions) {
        throw std::runtime_error(
            "JSON is neither a motion nor a mission: " + json_path.string());
    }
    for (const auto& entry : *mission_motions) {
        const std::string filename = entry.second.get<std::string>("filename");
        const std::filesystem::path motion_path =
            motion_directory / std::filesystem::path(filename);
        motions_.push_back(LoadMotion(motion_path));
    }
    if (motions_.empty()) {
        throw std::runtime_error(
            "mission JSON contains no motions: " + json_path.string());
    }
}

void P2PMotionPlayer::AppendProgram(
    const std::filesystem::path& json_path,
    const std::filesystem::path& motion_directory)
{
    std::vector<Motion> preceding_motions = std::move(motions_);
    LoadProgram(json_path, motion_directory);
    preceding_motions.insert(preceding_motions.end(),
                             motions_.begin(), motions_.end());
    motions_ = std::move(preceding_motions);
}

void P2PMotionPlayer::ApplyStartOptions(
    const StartOptions& options,
    const std::filesystem::path& motion_directory)
{
    if (options.repeat_count == 0) {
        throw std::runtime_error("repeat_count must be greater than zero");
    }
    if (options.duration_override_sec &&
        *options.duration_override_sec <= 0.0) {
        throw std::runtime_error(
            "duration_override_sec must be greater than zero");
    }

    if (options.final_keyframe_only) {
        if (motions_.empty() || motions_.back().keyframes.empty()) {
            throw std::runtime_error(
                "final keyframe requested from an empty program");
        }
        Motion final_pose;
        final_pose.name = motions_.back().name + "_final_pose";
        final_pose.keyframes.push_back(motions_.back().keyframes.back());
        final_pose.pd_gains = motions_.back().pd_gains;
        motions_.clear();
        motions_.push_back(std::move(final_pose));
    }

    const std::vector<Motion> original_motions = motions_;
    motions_.clear();
    for (std::size_t repeat = 0; repeat < options.repeat_count; ++repeat) {
        motions_.insert(motions_.end(), original_motions.begin(),
                        original_motions.end());
    }

    if (!options.position_offsets.empty()) {
        if (motions_.empty()) {
            throw std::runtime_error(
                "position offsets require at least one primary motion");
        }

        Keyframe restore_frame;
        if (options.restore_offsets) {
            restore_frame = motions_.back().keyframes.back();
            restore_frame.name += "_offset_restore";
            restore_frame.duration_sec = 0.5;
            restore_frame.hold_sec = 0.0;
            restore_frame.max_speed_deg_s = 30.0;
            restore_frame.interpolation = Interpolation::kSmoothStep;
        }

        for (auto& motion : motions_) {
            for (auto& frame : motion.keyframes) {
                for (const auto& offset : options.position_offsets) {
                    const auto position = frame.positions.find(offset.first);
                    if (position == frame.positions.end()) {
                        throw std::runtime_error(
                            "keyframe '" + frame.name +
                            "' is missing offset motor ID " +
                            std::to_string(offset.first));
                    }
                    const std::int64_t target =
                        static_cast<std::int64_t>(position->second) +
                        offset.second;
                    if (target < 0 || target > 4095) {
                        throw std::runtime_error(
                            "offset target is outside 0..4095 for motor ID " +
                            std::to_string(offset.first));
                    }
                    position->second = static_cast<int32_t>(target);
                }
            }
        }

        if (options.restore_offsets) {
            motions_.back().keyframes.push_back(std::move(restore_frame));
        }
    }

    for (const auto& trailing_path : options.trailing_program_paths) {
        AppendProgram(trailing_path, motion_directory);
    }
    if (options.duration_override_sec) {
        for (auto& motion : motions_) {
            for (auto& frame : motion.keyframes) {
                frame.duration_sec = *options.duration_override_sec;
            }
        }
    }
    if (motions_.empty()) {
        throw std::runtime_error("program contains no motions after options");
    }
}

void P2PMotionPlayer::SelectMotion(std::size_t index)
{
    motion_index_ = index;
    motion_name_ = motions_.at(index).name;
    keyframes_ = motions_.at(index).keyframes;
    keyframe_index_ = 0;
    if (!motions_.at(index).pd_gains.empty()) {
        dxl_->SyncWritePositionPDGains(motions_.at(index).pd_gains);
    }
}

bool P2PMotionPlayer::Start(
    const std::string& json_path,
    const std::string& motion_directory,
    const StartOptions& options)
{
    Stop();
    error_.clear();
    try {
        if (dxl_ == nullptr) throw std::runtime_error("Dxl is null");
        LoadProgram(json_path, motion_directory);
        ApplyStartOptions(options, motion_directory);
        SelectMotion(0);

        start_positions_ = dxl_->GetRawPositions();

        // ACK 전에 mission의 모든 motion/frame을 검사한다.
        for (const auto& motion : motions_) {
            for (const auto& frame : motion.keyframes) {
                for (const auto& motor : start_positions_) {
                    if (frame.positions.find(motor.first) == frame.positions.end()) {
                        throw std::runtime_error(
                            "keyframe '" + frame.name +
                            "' is missing motor ID " +
                            std::to_string(motor.first));
                    }
                }
            }
        }
        playing_ = true;
        BeginKeyframe(Clock::now());
        return true;
    } catch (const std::exception& e) {
        error_ = e.what();
        playing_ = false;
        return false;
    }
}

void P2PMotionPlayer::BeginKeyframe(Clock::time_point now)
{
    constexpr double kControlHz = 50.0;
    const Keyframe& frame = keyframes_.at(keyframe_index_);
    double max_delta_deg = 0.0;

    // 모든 관절 중 이동량이 가장 큰 관절을 기준으로 최소 필요시간을 구한다.
    // MX 계열 1회전=4096 tick이므로 tick 차이를 degree로 변환한다.
    for (const auto& motor : start_positions_) {
        const int32_t target = frame.positions.at(motor.first);
        const double delta_deg =
            std::abs(static_cast<double>(target) - motor.second) * 360.0 / 4096.0;
        max_delta_deg = std::max(max_delta_deg, delta_deg);
    }
    // 사용자가 지정한 시간보다 속도 제한에 필요한 시간이 길면 자동으로 늘린다.
    effective_duration_sec_ = std::max(
        frame.duration_sec, max_delta_deg / frame.max_speed_deg_s);
    // P2P 사이트와 동일한 방식: duration * control_hz를 반올림하고
    // 아주 짧은 모션도 최소 2개의 목표점을 순서대로 전송한다.
    move_step_count_ = std::max<std::size_t>(
        2, static_cast<std::size_t>(
               std::llround(effective_duration_sec_ * kControlHz)));
    move_step_ = 0;
    phase_started_at_ = now;
    holding_ = false;
}

double P2PMotionPlayer::Interpolate(Interpolation kind, double x)
{
    // 타이머 지연으로 x가 1보다 커질 수 있으므로 반드시 0~1로 제한한다.
    x = std::clamp(x, 0.0, 1.0);
    if (kind == Interpolation::kLinear) return x;
    if (kind == Interpolation::kMinimumJerk) {
        return 10.0*x*x*x - 15.0*x*x*x*x + 6.0*x*x*x*x*x;
    }
    return x*x*(3.0 - 2.0*x);
}

P2PMotionPlayer::UpdateResult P2PMotionPlayer::Update()
{
    if (!playing_) return UpdateResult::kIdle;
    try {
        const auto now = Clock::now();
        const double elapsed =
            std::chrono::duration<double>(now - phase_started_at_).count();
        const Keyframe& frame = keyframes_.at(keyframe_index_);

        // 목표 keyframe에는 이미 도착했고 hold_sec만큼 자세를 유지하는 단계다.
        // hold 중에는 새 목표값을 계속 쓰지 않고 시간만 확인한다.
        if (holding_) {
            if (elapsed < frame.hold_sec) return UpdateResult::kRunning;
            // 다음 keyframe 보간의 시작점은 방금 끝난 keyframe 목표값이다.
            start_positions_ = frame.positions;
            ++keyframe_index_;
            if (keyframe_index_ >= keyframes_.size()) {
                if (motion_index_ + 1 >= motions_.size()) {
                    Stop();
                    return UpdateResult::kFinished;
                }
                SelectMotion(motion_index_ + 1);
            }
            BeginKeyframe(now);
            return UpdateResult::kRunning;
        }

        // 실제 경과시간으로 진행률을 뛰어넘지 않고 사이트처럼 매 timer마다
        // 정확히 한 step씩 진행한다. timer 지연 시 전체 실행시간이 늘어난다.
        ++move_step_;
        const double progress = static_cast<double>(move_step_) /
            static_cast<double>(move_step_count_);
        const double alpha = Interpolate(frame.interpolation, progress);
        RawPositions command;

        // 각 motor ID별로 start와 target 사이의 이번 timer tick 목표를 만든다.
        for (const auto& motor : start_positions_) {
            const int32_t target = frame.positions.at(motor.first);
            const double value = static_cast<double>(motor.second) +
                alpha * (static_cast<double>(target) - motor.second);
            command.emplace(motor.first, static_cast<int32_t>(std::lround(value)));
        }
        // 23개 목표를 하나의 GroupSyncWrite 패킷으로 동시에 전송한다.
        dxl_->SyncWriteRawPositions(command);

        if (move_step_ < move_step_count_) return UpdateResult::kRunning;
        start_positions_ = frame.positions;
        // 이동이 끝났어도 hold가 있으면 아직 전체 keyframe 완료가 아니다.
        if (frame.hold_sec > 0.0) {
            holding_ = true;
            phase_started_at_ = now;
            return UpdateResult::kRunning;
        }
        // hold가 없으면 바로 다음 keyframe을 준비한다.
        ++keyframe_index_;
        if (keyframe_index_ >= keyframes_.size()) {
            if (motion_index_ + 1 >= motions_.size()) {
                Stop();
                return UpdateResult::kFinished;
            }
            SelectMotion(motion_index_ + 1);
        }
        BeginKeyframe(now);
        return UpdateResult::kRunning;
    } catch (const std::exception& e) {
        error_ = e.what();
        playing_ = false;
        return UpdateResult::kError;
    }
}

void P2PMotionPlayer::Stop()
{
    // 통신 포트나 Torque 상태는 건드리지 않고 player 내부 상태만 초기화한다.
    playing_ = false;
    holding_ = false;
    motion_index_ = 0;
    keyframe_index_ = 0;
    effective_duration_sec_ = 0.0;
    move_step_ = 0;
    move_step_count_ = 0;
}
