#pragma once

#include <cstdlib>
#include <initializer_list>
#include <sstream>
#include <string>

#include <glog/logging.h>

namespace lightning {

struct ObservationMetric {
    const char* key;
    double value;
    const char* unit;
    const char* source;
    const char* state = "normal";
};

inline bool ObservabilityEnabled() {
    static const bool enabled = []() {
        const char* value = std::getenv("LIGHTNING_OBS_LOG");
        return value != nullptr && std::string(value) != "0";
    }();
    return enabled;
}

inline std::string EscapeObservationJson(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (const char character : value) {
        switch (character) {
            case '\\':
                escaped += "\\\\";
                break;
            case '"':
                escaped += "\\\"";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            default:
                escaped += static_cast<unsigned char>(character) < 0x20 ? '?' : character;
                break;
        }
    }
    return escaped;
}

inline void LogObservation(const std::string& stage, std::initializer_list<ObservationMetric> metrics,
                           const std::string& timer_name = "") {
    if (!ObservabilityEnabled()) {
        return;
    }

    std::ostringstream output;
    output << "{\"schemaVersion\":1,\"stage\":\"" << EscapeObservationJson(stage) << "\"";
    if (!timer_name.empty()) {
        output << ",\"timerName\":\"" << EscapeObservationJson(timer_name) << "\"";
    }
    output << ",\"metrics\":{";
    bool first = true;
    for (const auto& metric : metrics) {
        if (!first) {
            output << ',';
        }
        first = false;
        output << '"' << EscapeObservationJson(metric.key) << "\":{";
        output << "\"value\":" << metric.value;
        output << ",\"unit\":\"" << EscapeObservationJson(metric.unit) << "\"";
        output << ",\"source\":\"" << EscapeObservationJson(metric.source) << "\"";
        output << ",\"state\":\"" << EscapeObservationJson(metric.state) << "\"}";
    }
    output << "}}";
    LOG(INFO) << "LIGHTNING_OBS " << output.str();
}

inline void LogObservationTimer(const std::string& timer_name, double time_usage_ms) {
    LogObservation("timer", {{"timer_ms", time_usage_ms, "ms", "Timer::Evaluate"}}, timer_name);
}

}  // namespace lightning
