#include "nura/flight/apps/apogee_detector_app.h"

#include <cmath>

namespace nura::flight
{
namespace c = nura::contracts;

namespace
{
bool barometerFaulted(const c::BarometerSample &barometer)
{
    return (barometer.header.status_flags & c::SAMPLE_STATUS_FAULT) != 0U ||
           barometer.fault_flags != c::BAROMETER_FAULT_NONE;
}

bool barometerValid(const c::BarometerSample &barometer)
{
    return (barometer.header.status_flags & c::SAMPLE_STATUS_VALID) != 0U &&
           (barometer.header.status_flags & c::SAMPLE_STATUS_REFERENCE_VALID) != 0U &&
           !barometerFaulted(barometer) && barometer.header.sample_time_ms != 0U;
}
} // namespace

c::AppId ApogeeDetectorApp::id() const { return c::AppId::APOGEE_DETECTOR; }
c::FlightState ApogeeDetectorApp::state() const { return c::FlightState::COAST; }

StateAppOutput ApogeeDetectorApp::onEnter(const c::FlightStatus &status,
                                          const c::FlightInputs &inputs,
                                          uint32_t now_ms)
{
    (void)status;
    (void)now_ms;
    resetPrediction();
    max_coast_altitude_m_ = inputs.barometer.filtered_altitude_m;
    return {};
}

StateAppOutput ApogeeDetectorApp::step(const c::FlightStatus &status,
                                       const c::FlightInputs &inputs,
                                       uint32_t now_ms)
{
    StateAppOutput output{};
    const uint32_t coast_elapsed_ms = now_ms - status.coast_ms;
    if (coast_elapsed_ms >= FlightPolicy::kApogeeTimeoutMs)
    {
        (void)output.addDecision(makeDecision(
            status.state, c::DecisionKind::APOGEE_TIMER, c::DecisionResult::ACCEPT,
            c::DECISION_REASON_TIMEOUT, now_ms, static_cast<float>(coast_elapsed_ms),
            static_cast<float>(FlightPolicy::kApogeeTimeoutMs),
            inputs.barometer.filtered_altitude_m, max_coast_altitude_m_, 0U, 0U));
        output.requestTransition(c::FlightState::APOGEE, now_ms, id());
        return output;
    }

    if (status.barometer_stuck_fault_latched || barometerFaulted(inputs.barometer))
    {
        if (attitudeFallback(status, inputs, now_ms, output))
        {
            output.requestTransition(c::FlightState::APOGEE, now_ms, id());
        }
        return output;
    }

    if (!consumeBarometer(inputs.barometer, output))
    {
        return output;
    }

    const float current_altitude_m = inputs.barometer.filtered_altitude_m;
    if (current_altitude_m > max_coast_altitude_m_)
    {
        max_coast_altitude_m_ = current_altitude_m;
    }

    const uint32_t launch_elapsed_ms = inputs.barometer.header.sample_time_ms - status.launch_ms;
    if (launch_elapsed_ms < FlightPolicy::kApogeeMinFlightTimeMs)
    {
        prediction_confirmation_count_ = 0U;
        descent_confirmation_count_ = 0U;
        (void)output.addDecision(makeDecision(
            status.state, c::DecisionKind::APOGEE_PREDICTION, c::DecisionResult::REJECT,
            c::DECISION_REASON_TOO_EARLY, inputs.barometer.header.sample_time_ms,
            current_altitude_m, static_cast<float>(launch_elapsed_ms),
            static_cast<float>(FlightPolicy::kApogeeMinFlightTimeMs),
            max_coast_altitude_m_, prediction_confirmation_count_,
            descent_confirmation_count_));
        return output;
    }

    if (predictionReady(inputs.barometer, current_altitude_m, output))
    {
        ++prediction_confirmation_count_;
    }
    else
    {
        prediction_confirmation_count_ = 0U;
    }

    if ((max_coast_altitude_m_ - current_altitude_m) >=
        FlightPolicy::kApogeeDropThresholdM)
    {
        ++descent_confirmation_count_;
    }
    else
    {
        descent_confirmation_count_ = 0U;
    }

    const bool prediction_accepted =
        prediction_confirmation_count_ >= FlightPolicy::kApogeeConfirmSamples;
    const bool descent_accepted =
        descent_confirmation_count_ >= FlightPolicy::kApogeeDescentConfirmSamples;
    if (prediction_accepted)
    {
        (void)output.addDecision(makeDecision(
            status.state, c::DecisionKind::APOGEE_PREDICTION, c::DecisionResult::ACCEPT,
            c::DECISION_REASON_CONFIRMATION_MET, inputs.barometer.header.sample_time_ms,
            current_altitude_m, max_coast_altitude_m_, 0.0f, 0.0f,
            prediction_confirmation_count_, descent_confirmation_count_));
    }
    else
    {
        (void)output.addDecision(makeDecision(
            status.state, c::DecisionKind::APOGEE_DESCENT,
            descent_accepted ? c::DecisionResult::ACCEPT : c::DecisionResult::OBSERVE,
            descent_accepted ? c::DECISION_REASON_CONFIRMATION_MET
                             : c::DECISION_REASON_THRESHOLD_NOT_MET,
            inputs.barometer.header.sample_time_ms, current_altitude_m,
            max_coast_altitude_m_, max_coast_altitude_m_ - current_altitude_m,
            FlightPolicy::kApogeeDropThresholdM, prediction_confirmation_count_,
            descent_confirmation_count_));
    }

    if (prediction_accepted || descent_accepted)
    {
        output.requestTransition(c::FlightState::APOGEE,
                                 inputs.barometer.header.sample_time_ms, id());
    }
    return output;
}

bool ApogeeDetectorApp::consumeBarometer(const c::BarometerSample &barometer,
                                         StateAppOutput &output)
{
    if (!barometerValid(barometer) ||
        barometer.header.sample_time_ms == last_barometer_time_ms_)
    {
        return false;
    }

    if (last_barometer_time_ms_ != 0U &&
        (barometer.header.sample_time_ms - last_barometer_time_ms_) >
            FlightPolicy::kApogeeMaxBarometerSampleGapMs)
    {
        altitude_head_ = 0U;
        altitude_count_ = 0U;
        prediction_head_ = 0U;
        prediction_count_ = 0U;
        prediction_confirmation_count_ = 0U;
    }

    last_barometer_time_ms_ = barometer.header.sample_time_ms;
    trackStuck(barometer.header.sample_time_ms, barometer.filtered_altitude_m, output);
    if (output.latch_barometer_stuck_fault)
    {
        return false;
    }

    pushAltitude(barometer.header.sample_time_ms, barometer.filtered_altitude_m);
    return true;
}

bool ApogeeDetectorApp::attitudeFallback(const c::FlightStatus &status,
                                         const c::FlightInputs &inputs,
                                         uint32_t now_ms, StateAppOutput &output)
{
    const uint32_t launch_elapsed_ms = now_ms - status.launch_ms;
    if (launch_elapsed_ms < FlightPolicy::kBaroFaultAttitudeFallbackMinFlightTimeMs)
    {
        attitude_confirmation_count_ = 0U;
        return false;
    }

    const auto &imu = inputs.low_g;
    if (imu.header.sample_time_ms == 0U ||
        imu.header.sample_time_ms == last_attitude_time_ms_)
    {
        return false;
    }

    last_attitude_time_ms_ = imu.header.sample_time_ms;
    if (!imu.tilt_valid ||
        (now_ms - imu.header.sample_time_ms) >
            FlightPolicy::kBaroFaultAttitudeFallbackMaxSampleAgeMs)
    {
        attitude_confirmation_count_ = 0U;
        return false;
    }

    if (imu.tilt_angle_deg >= FlightPolicy::kBaroFaultAttitudeFallbackTiltDeg)
    {
        ++attitude_confirmation_count_;
    }
    else
    {
        attitude_confirmation_count_ = 0U;
    }

    const bool accepted = attitude_confirmation_count_ >=
                          FlightPolicy::kBaroFaultAttitudeFallbackConfirmSamples;
    (void)output.addDecision(makeDecision(
        status.state, c::DecisionKind::BAROMETER_FAULT_TILT,
        accepted ? c::DecisionResult::ACCEPT : c::DecisionResult::OBSERVE,
        static_cast<uint16_t>(c::DECISION_REASON_SENSOR_FAULT |
                              (accepted ? c::DECISION_REASON_CONFIRMATION_MET
                                        : c::DECISION_REASON_THRESHOLD_NOT_MET)),
        imu.header.sample_time_ms, imu.tilt_angle_deg,
        FlightPolicy::kBaroFaultAttitudeFallbackTiltDeg,
        static_cast<float>(launch_elapsed_ms),
        static_cast<float>(FlightPolicy::kBaroFaultAttitudeFallbackMinFlightTimeMs),
        attitude_confirmation_count_, 0U));
    return accepted;
}

bool ApogeeDetectorApp::predictionReady(const c::BarometerSample &barometer,
                                        float current_altitude_m,
                                        StateAppOutput &output)
{
    if (altitude_count_ < FlightPolicy::kApogeeFitWindowSamples ||
        current_altitude_m < FlightPolicy::kMinApogeeDetectAltM)
    {
        (void)output.addDecision(makeDecision(
            c::FlightState::COAST, c::DecisionKind::APOGEE_PREDICTION,
            c::DecisionResult::REJECT, c::DECISION_REASON_QUALITY_REJECT,
            barometer.header.sample_time_ms, current_altitude_m,
            static_cast<float>(altitude_count_), FlightPolicy::kMinApogeeDetectAltM,
            0.0f, prediction_confirmation_count_, descent_confirmation_count_));
        return false;
    }

    QuadraticFit fit{};
    if (!solveQuadratic(fit) ||
        fit.a >= -FlightPolicy::kApogeeMinCurvature ||
        fit.a <= -FlightPolicy::kApogeeMaxCurvature ||
        fit.rmse_m > FlightPolicy::kApogeeMaxFitRmseM)
    {
        (void)output.addDecision(makeDecision(
            c::FlightState::COAST, c::DecisionKind::APOGEE_PREDICTION,
            c::DecisionResult::REJECT, c::DECISION_REASON_QUALITY_REJECT,
            barometer.header.sample_time_ms, current_altitude_m, fit.a, fit.rmse_m,
            FlightPolicy::kApogeeMaxFitRmseM, prediction_confirmation_count_,
            descent_confirmation_count_));
        return false;
    }

    const uint8_t last_index = static_cast<uint8_t>(
        (altitude_head_ + FlightPolicy::kApogeeFitWindowSamples - 1U) %
        FlightPolicy::kApogeeFitWindowSamples);
    const uint8_t first_index = altitude_head_;
    const float last_t = static_cast<float>(
        altitude_samples_[last_index].sample_time_ms -
        altitude_samples_[first_index].sample_time_ms) /
        1000.0f;
    const float t_apogee = -fit.b / (2.0f * fit.a);
    if (!std::isfinite(t_apogee) || t_apogee <= last_t ||
        (t_apogee - last_t) > FlightPolicy::kApogeeMaxPredictAheadS)
    {
        (void)output.addDecision(makeDecision(
            c::FlightState::COAST, c::DecisionKind::APOGEE_PREDICTION,
            c::DecisionResult::REJECT, c::DECISION_REASON_QUALITY_REJECT,
            barometer.header.sample_time_ms, current_altitude_m, t_apogee, last_t,
            FlightPolicy::kApogeeMaxPredictAheadS, prediction_confirmation_count_,
            descent_confirmation_count_));
        return false;
    }

    const float raw_apogee_m = fit.c - ((fit.b * fit.b) / (4.0f * fit.a));
    const float raw_margin_m = raw_apogee_m - current_altitude_m;
    if (!std::isfinite(raw_apogee_m) || raw_margin_m < 0.0f ||
        raw_margin_m > FlightPolicy::kApogeeMaxAltMarginM ||
        !pushPrediction(raw_apogee_m))
    {
        (void)output.addDecision(makeDecision(
            c::FlightState::COAST, c::DecisionKind::APOGEE_PREDICTION,
            c::DecisionResult::REJECT, c::DECISION_REASON_QUALITY_REJECT,
            barometer.header.sample_time_ms, current_altitude_m, raw_apogee_m,
            raw_margin_m, FlightPolicy::kApogeeMaxAltMarginM,
            prediction_confirmation_count_, descent_confirmation_count_));
        return false;
    }

    float aggregated_apogee_m = 0.0f;
    if (!plusTwoSigma(aggregated_apogee_m))
    {
        (void)output.addDecision(makeDecision(
            c::FlightState::COAST, c::DecisionKind::APOGEE_PREDICTION,
            c::DecisionResult::REJECT, c::DECISION_REASON_QUALITY_REJECT,
            barometer.header.sample_time_ms, current_altitude_m, raw_apogee_m,
            raw_margin_m, static_cast<float>(prediction_count_),
            prediction_confirmation_count_, descent_confirmation_count_));
        return false;
    }

    const float aggregated_margin_m = aggregated_apogee_m - current_altitude_m;
    const bool ready = std::isfinite(aggregated_apogee_m) &&
                       aggregated_margin_m >= 0.0f &&
                       aggregated_margin_m <= FlightPolicy::kApogeeDeployAltMarginM &&
                       aggregated_margin_m <= FlightPolicy::kApogeeMaxAltMarginM;
    (void)output.addDecision(makeDecision(
        c::FlightState::COAST, c::DecisionKind::APOGEE_PREDICTION,
        ready ? c::DecisionResult::ACCEPT : c::DecisionResult::OBSERVE,
        ready ? c::DECISION_REASON_THRESHOLD_MET
              : c::DECISION_REASON_THRESHOLD_NOT_MET,
        barometer.header.sample_time_ms, current_altitude_m, aggregated_apogee_m,
        aggregated_margin_m, fit.rmse_m, prediction_confirmation_count_,
        descent_confirmation_count_));
    return ready;
}

void ApogeeDetectorApp::pushAltitude(uint32_t sample_time_ms, float altitude_m)
{
    altitude_samples_[altitude_head_] = {sample_time_ms, altitude_m};
    altitude_head_ = static_cast<uint8_t>((altitude_head_ + 1U) %
                                          FlightPolicy::kApogeeFitWindowSamples);
    if (altitude_count_ < FlightPolicy::kApogeeFitWindowSamples)
    {
        ++altitude_count_;
    }
}

bool ApogeeDetectorApp::pushPrediction(float prediction_m)
{
    if (!std::isfinite(prediction_m))
    {
        return false;
    }
    if (prediction_count_ > 0U)
    {
        const uint8_t last_index = static_cast<uint8_t>(
            (prediction_head_ + FlightPolicy::kApogeePredictionHistorySamples - 1U) %
            FlightPolicy::kApogeePredictionHistorySamples);
        if (std::fabs(prediction_m - predictions_[last_index]) >
            FlightPolicy::kApogeeMaxPredictionJumpM)
        {
            return false;
        }
    }

    predictions_[prediction_head_] = prediction_m;
    prediction_head_ = static_cast<uint8_t>((prediction_head_ + 1U) %
                                            FlightPolicy::kApogeePredictionHistorySamples);
    if (prediction_count_ < FlightPolicy::kApogeePredictionHistorySamples)
    {
        ++prediction_count_;
    }
    return true;
}

bool ApogeeDetectorApp::plusTwoSigma(float &prediction_m) const
{
    if (prediction_count_ < FlightPolicy::kApogeePredictionHistorySamples)
    {
        return false;
    }

    float sum = 0.0f;
    for (float value : predictions_)
    {
        sum += value;
    }
    const float mean = sum /
                       static_cast<float>(FlightPolicy::kApogeePredictionHistorySamples);

    float variance = 0.0f;
    for (float value : predictions_)
    {
        const float error = value - mean;
        variance += error * error;
    }
    variance /= static_cast<float>(FlightPolicy::kApogeePredictionHistorySamples);
    const float sigma = std::sqrt(variance);
    if (!std::isfinite(mean) || !std::isfinite(sigma) ||
        sigma > FlightPolicy::kApogeeMaxPredictionSigmaM)
    {
        return false;
    }

    prediction_m = mean + (FlightPolicy::kApogeeAggregationSigmaMultiplier * sigma);
    return std::isfinite(prediction_m);
}

bool ApogeeDetectorApp::solveQuadratic(QuadraticFit &fit) const
{
    const uint8_t n = FlightPolicy::kApogeeFitWindowSamples;
    const uint8_t first_index = altitude_head_;
    const uint32_t t0 = altitude_samples_[first_index].sample_time_ms;
    float s0 = static_cast<float>(n);
    float s1 = 0.0f;
    float s2 = 0.0f;
    float s3 = 0.0f;
    float s4 = 0.0f;
    float y0 = 0.0f;
    float y1 = 0.0f;
    float y2 = 0.0f;

    for (uint8_t i = 0U; i < n; ++i)
    {
        const uint8_t index = static_cast<uint8_t>((first_index + i) % n);
        const float x = static_cast<float>(altitude_samples_[index].sample_time_ms - t0) /
                        1000.0f;
        const float y = altitude_samples_[index].altitude_m;
        const float x2 = x * x;
        s1 += x;
        s2 += x2;
        s3 += x2 * x;
        s4 += x2 * x2;
        y0 += y;
        y1 += x * y;
        y2 += x2 * y;
    }

    float matrix[3][4] = {
        {s4, s3, s2, y2},
        {s3, s2, s1, y1},
        {s2, s1, s0, y0},
    };
    if (!solve3x3(matrix, fit.a, fit.b, fit.c))
    {
        return false;
    }

    float squared_error_sum = 0.0f;
    for (uint8_t i = 0U; i < n; ++i)
    {
        const uint8_t index = static_cast<uint8_t>((first_index + i) % n);
        const float x = static_cast<float>(altitude_samples_[index].sample_time_ms - t0) /
                        1000.0f;
        const float predicted = (fit.a * x * x) + (fit.b * x) + fit.c;
        const float error = altitude_samples_[index].altitude_m - predicted;
        squared_error_sum += error * error;
    }
    fit.rmse_m = std::sqrt(squared_error_sum / static_cast<float>(n));
    return std::isfinite(fit.rmse_m);
}

bool ApogeeDetectorApp::solve3x3(float matrix[3][4], float &x0, float &x1, float &x2)
{
    constexpr float kEpsilon = 1.0e-6f;
    for (uint8_t col = 0U; col < 3U; ++col)
    {
        uint8_t pivot = col;
        float best = std::fabs(matrix[col][col]);
        for (uint8_t row = static_cast<uint8_t>(col + 1U); row < 3U; ++row)
        {
            const float candidate = std::fabs(matrix[row][col]);
            if (candidate > best)
            {
                best = candidate;
                pivot = row;
            }
        }
        if (best < kEpsilon)
        {
            return false;
        }
        if (pivot != col)
        {
            for (uint8_t j = col; j < 4U; ++j)
            {
                const float temporary = matrix[col][j];
                matrix[col][j] = matrix[pivot][j];
                matrix[pivot][j] = temporary;
            }
        }
        const float divisor = matrix[col][col];
        for (uint8_t j = col; j < 4U; ++j)
        {
            matrix[col][j] /= divisor;
        }
        for (uint8_t row = 0U; row < 3U; ++row)
        {
            if (row == col)
            {
                continue;
            }
            const float factor = matrix[row][col];
            for (uint8_t j = col; j < 4U; ++j)
            {
                matrix[row][j] -= factor * matrix[col][j];
            }
        }
    }
    x0 = matrix[0][3];
    x1 = matrix[1][3];
    x2 = matrix[2][3];
    return std::isfinite(x0) && std::isfinite(x1) && std::isfinite(x2);
}

void ApogeeDetectorApp::resetPrediction()
{
    altitude_head_ = 0U;
    altitude_count_ = 0U;
    prediction_head_ = 0U;
    prediction_count_ = 0U;
    prediction_confirmation_count_ = 0U;
    descent_confirmation_count_ = 0U;
    attitude_confirmation_count_ = 0U;
    last_barometer_time_ms_ = 0U;
    last_attitude_time_ms_ = 0U;
    max_coast_altitude_m_ = 0.0f;
    resetStuckMonitor();
}

void ApogeeDetectorApp::resetStuckMonitor()
{
    stuck_window_active_ = false;
    stuck_window_start_ms_ = 0U;
    last_stuck_sample_ms_ = 0U;
    stuck_min_altitude_m_ = 0.0f;
    stuck_max_altitude_m_ = 0.0f;
}

void ApogeeDetectorApp::trackStuck(uint32_t sample_time_ms, float altitude_m,
                                   StateAppOutput &output)
{
    if (!std::isfinite(altitude_m))
    {
        resetStuckMonitor();
        return;
    }
    if (!stuck_window_active_ ||
        (last_stuck_sample_ms_ != 0U &&
         (sample_time_ms - last_stuck_sample_ms_) >
             FlightPolicy::kApogeeMaxBarometerSampleGapMs))
    {
        stuck_window_active_ = true;
        stuck_window_start_ms_ = sample_time_ms;
        stuck_min_altitude_m_ = altitude_m;
        stuck_max_altitude_m_ = altitude_m;
        last_stuck_sample_ms_ = sample_time_ms;
        return;
    }

    if (altitude_m < stuck_min_altitude_m_)
    {
        stuck_min_altitude_m_ = altitude_m;
    }
    if (altitude_m > stuck_max_altitude_m_)
    {
        stuck_max_altitude_m_ = altitude_m;
    }
    last_stuck_sample_ms_ = sample_time_ms;

    const float range_m = stuck_max_altitude_m_ - stuck_min_altitude_m_;
    if ((sample_time_ms - stuck_window_start_ms_) >= FlightPolicy::kBarometerStuckWindowMs &&
        range_m <= FlightPolicy::kBarometerStuckRangeM)
    {
        output.latch_barometer_stuck_fault = true;
        return;
    }
    if ((sample_time_ms - stuck_window_start_ms_) >= FlightPolicy::kBarometerStuckWindowMs)
    {
        stuck_window_start_ms_ = sample_time_ms;
        stuck_min_altitude_m_ = altitude_m;
        stuck_max_altitude_m_ = altitude_m;
    }
}

} // namespace nura::flight
